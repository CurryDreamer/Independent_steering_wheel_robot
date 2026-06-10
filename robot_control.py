import math
import time

import mujoco
import mujoco.viewer
from remote.xbox_controller import PygameController
from AGV_update.AGV_cal import AGV_CAL
import AGV_update.robot_info_update as robot_info_update

if __name__ == "__main__":
    dt = 0.001

    m = mujoco.MjModel.from_xml_path('mjcf/robot.xml')
    d = mujoco.MjData(m)
    
    # 1. 实例化遥控器和解算器
    teleop = PygameController()
    agv_solver = AGV_CAL()  # 正确的实例化方式

    with mujoco.viewer.launch_passive(m, d) as viewer:
        while viewer.is_running():
            step_start = time.time()

            # 2. 读取 MuJoCo 中的传感器/关节信息
            info = robot_info_update.robot_info_update(d)

            # =================== 【核心修改点 1】 ===================
            # 从 MuJoCo 读取当前的真实物理反馈（弧度），并转换至算法坐标系（角度）
            # 必须乘以 -1 来匹配你输出控制量时使用的负号，确保形成负反馈闭环
            current_angles_deg = [
                -math.degrees(info['pos']['steering_j1']),
                -math.degrees(info['pos']['steering_j2']),
                -math.degrees(info['pos']['steering_j3']),
                -math.degrees(info['pos']['steering_j4'])
            ]
            # =======================================================

            # 3. 更新手柄/遥控器输入
            remote = teleop.update(teleop)
            
            # 4. 将遥控器的输入速度同步给解算器
            agv_solver.AGV_input_update(remote)
            
            # =================== 【核心修改点 2】 ===================
            # 执行四轮舵轮运动学解算，将上面提取到的真实传感器角度传入，激活优劣弧闭环判断
            agv_solver.AGV_cal_kinematics(current_angles=current_angles_deg)
            # =======================================================

            # 6. 将解算结果映射到 MuJoCo 的控制向量 d.ctrl 中
            # 1号轮
            d.ctrl[0] = -math.radians(agv_solver.chassis_angle1) # 转向角 (rad)
            d.ctrl[1] = agv_solver.chassis_v1                   # 轮速
            
            # 2号轮
            d.ctrl[2] = -math.radians(agv_solver.chassis_angle2) # 转向角 (rad)
            d.ctrl[3] = agv_solver.chassis_v2                   # 轮速
            
            # 3号轮
            d.ctrl[4] = -math.radians(agv_solver.chassis_angle3) # 转向角 (rad)
            d.ctrl[5] = agv_solver.chassis_v3                   # 轮速
            
            # 4号轮
            d.ctrl[6] = -math.radians(agv_solver.chassis_angle4) # 转向角 (rad)
            d.ctrl[7] = agv_solver.chassis_v4                   # 轮速

            # 仿真步进
            mujoco.mj_step(m, d)
            viewer.sync()
            
            time_until_next_step = dt - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)