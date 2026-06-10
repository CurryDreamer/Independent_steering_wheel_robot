import pygame
import numpy as np

class PygameController:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((400, 300))
        pygame.display.set_caption("Robot Control (Focus Here!)")
        self.font = pygame.font.SysFont(None, 32)

        # 初始化手柄
        self.joystick = None
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print(f"检测到手柄: {self.joystick.get_name()}")
        else:
            print("未检测到手柄，继续使用键盘控制")
            
        self.target_Vy = 0.0    
        self.target_Vx = 0.0
        self.target_Vw = 0.0
    def update(self, remote):
        """
        更新事件、读取键盘/手柄，并刷新 UI。
        传入主程序中的实时运行数据用于界面显示。
        """
        # --- 泵送事件队列，确保手柄/键盘状态更新 ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit(0)

        # --- 读取手柄轴数据 ---
        if self.joystick is not None:
            deadzone = 0.1

            # 轴 0：左摇杆左右 → target_Vx（左右平移）
            axis_0 = self.joystick.get_axis(1)
            if abs(axis_0) > deadzone:
                remote.target_Vy = axis_0 * 10.0
            else:
                remote.target_Vy = 0.0

            # 轴 1：左摇杆上下 → target_Vy（前后移动）
            axis_3 = self.joystick.get_axis(0)
            if abs(axis_3) > deadzone:
                remote.target_Vx = -axis_3 * 10.0
            else:
                remote.target_Vx = 0.0

            # 轴 3：右摇杆左右 → target_Vw（旋转）
            axis_5 = self.joystick.get_axis(3)
            if abs(axis_5) > deadzone:
                remote.target_Vw = -axis_5 * 10.0
            else:
                remote.target_Vw = 0.0



        # --- 刷新控制面板 UI ---
        self.screen.fill((30, 30, 30))
        text1 = self.font.render(f"(ENTER) : {'ON'}", True, (255, 255, 255))
        # text2 = self.font.render(f"Target V (UP/DOWN): {self.target_Vy:.2f} | Cur: {current_v:.2f}", True, (255, 255, 255))
        # text3 = self.font.render(f"Leg L: {target_length_left:.3f} | R: {target_length_right:.3f} | Roll: {roll:.2f}", True, (255, 255, 255))
        # text4 = self.font.render(f"Paused (SPACE)    : {self.paused}", True, (255, 255, 255))
        
        # self.screen.blit(text1, (20, 30))
        # self.screen.blit(text2, (20, 80))
        # self.screen.blit(text3, (20, 130))
        # self.screen.blit(text4, (20, 180))
        pygame.display.flip()
        return remote