import math

MOVEWHEEL1 = 0
MOVEWHEEL2 = 1
MOVEWHEEL3 = 2
MOVEWHEEL4 = 3

class AGV_CAL:
    def __init__(self):
        self.sin_w_angle = math.sin(math.atan2(1.6, 0.6))
        self.cos_w_angle = math.cos(math.atan2(1.6, 0.6))
        self.R2DEG = 180.0 / math.pi
        self.Vxspeed = 0.0
        self.Vyspeed = 0.0
        self.Vzspeed = 0.0
        
    def AGV_input_update(self, remote):
        self.Vxspeed = remote.target_Vx
        self.Vyspeed = remote.target_Vy
        self.Vzspeed = remote.target_Vw

    def optimize_swerve(self, target_angle, current_angle, target_speed):
        """
        1:1 字面还原 C 语言版本的舵轮优劣弧归位逻辑
        """
        # 计算初始误差
        err = target_angle - current_angle
        
        err = (err + 180.0) % 360.0 - 180.0
        
        # 初始化 turnflag
        turnflag = 0

        if err > 90.0 and err < 180.0:
            err = err - 180.0
            turnflag = 1
        elif err < -90.0 and err > -180.0:
            err = err + 180.0
            turnflag = 1
        elif err > -90.0 and err < 90.0:
            turnflag = 0

        # 3. 根据 turnflag 决定是否反转速度
        if turnflag == 1:
            target_speed = -target_speed

        # 4. 计算最终送给 MuJoCo 的连续目标角度
        optimized_angle = current_angle + err
        
        return optimized_angle, target_speed

    def AGV_cal_kinematics(self, current_angles=[0.0, 0.0, 0.0, 0.0]):
        """
        :param current_angles: 包含4个轮子当前实际角度的列表(单位: 度)。
                               在调用时，请从 MuJoCo 的 sensor/info 中读取并传入。
        """
        x_axis = [0.0] * 4
        y_axis = [0.0] * 4
    
        y_axis[MOVEWHEEL1] = self.Vyspeed + self.Vzspeed * self.cos_w_angle
        x_axis[MOVEWHEEL1] = self.Vxspeed + self.Vzspeed * self.sin_w_angle
        
        y_axis[MOVEWHEEL2] = self.Vyspeed - self.Vzspeed * self.cos_w_angle
        x_axis[MOVEWHEEL2] = self.Vxspeed + self.Vzspeed * self.sin_w_angle
        
        y_axis[MOVEWHEEL3] = self.Vyspeed + self.Vzspeed * self.cos_w_angle
        x_axis[MOVEWHEEL3] = self.Vxspeed - self.Vzspeed * self.sin_w_angle

        y_axis[MOVEWHEEL4] = self.Vyspeed - self.Vzspeed * self.cos_w_angle
        x_axis[MOVEWHEEL4] = self.Vxspeed - self.Vzspeed * self.sin_w_angle

        # 1. 计算原始速度
        raw_v1 = math.hypot(x_axis[MOVEWHEEL1], y_axis[MOVEWHEEL1])
        raw_v2 = math.hypot(x_axis[MOVEWHEEL2], y_axis[MOVEWHEEL2])
        raw_v3 = math.hypot(x_axis[MOVEWHEEL3], y_axis[MOVEWHEEL3])
        raw_v4 = math.hypot(x_axis[MOVEWHEEL4], y_axis[MOVEWHEEL4])

        # 2. 计算原始角度
        raw_angle1 = math.atan2(x_axis[MOVEWHEEL1], y_axis[MOVEWHEEL1]) * self.R2DEG
        raw_angle2 = math.atan2(x_axis[MOVEWHEEL2], y_axis[MOVEWHEEL2]) * self.R2DEG
        raw_angle3 = math.atan2(x_axis[MOVEWHEEL3], y_axis[MOVEWHEEL3]) * self.R2DEG
        raw_angle4 = math.atan2(x_axis[MOVEWHEEL4], y_axis[MOVEWHEEL4]) * self.R2DEG

        # 3. 引入优劣弧与反转逻辑 (依赖当前角度)
        self.chassis_angle1, self.chassis_v1 = self.optimize_swerve(raw_angle1, current_angles[MOVEWHEEL1], raw_v1)
        self.chassis_angle2, self.chassis_v2 = self.optimize_swerve(raw_angle2, current_angles[MOVEWHEEL2], raw_v2)
        self.chassis_angle3, self.chassis_v3 = self.optimize_swerve(raw_angle3, current_angles[MOVEWHEEL3], raw_v3)
        self.chassis_angle4, self.chassis_v4 = self.optimize_swerve(raw_angle4, current_angles[MOVEWHEEL4], raw_v4)

        return self