# 独立舵轮机器人仿真 (Independent Steering Wheel Robot)

本项目是一个基于 **MuJoCo** 物理引擎的四轮独立转向（4WIS / 舵轮 / Swerve Drive）移动机器人仿真与控制框架。机器人每个轮子均可独立转向和独立驱动，能够实现前后移动、左右平移和原地旋转等全向运动，并支持通过 Xbox 手柄进行实时遥控。

## 目录结构

```text
Independent_steering_wheel_robot/
├── robot_control.py              # [主入口] MuJoCo 仿真主循环 + 被动渲染窗口
├── AGV_update/
│   ├── AGV_cal.py                # 舵轮运动学解算器（正向/逆向 + 优劣弧优化）
│   └── robot_info_update.py      # MuJoCo 传感器信息读取（关节位置/速度 + IMU 欧拉角 + 陀螺仪）
├── remote/
│   └── xbox_controller.py        # Pygame 遥控器（Xbox 手柄 / 控制面板窗口）
├── mjcf/
│   ├── robot.xml                 # MuJoCo 机器人模型描述文件
│   └── obstacles.xml             # 测试场景（5 个可调角度斜坡）
└── README.md
```

## 环境依赖

本项目基于 **Ubuntu 22.04** 开发与运行，系统默认 Python 版本为 **Python 3.10+**。不需要单独创建虚拟环境，运行前安装系统依赖和 Python 依赖即可。

### 1. 安装系统依赖

```bash
sudo apt update
sudo apt install -y python3 python3-pip python3-dev \
    libgl1-mesa-glx libglfw3 libglew2.2 libosmesa6 \
    libxrandr2 libxinerama1 libxcursor1 libxi6
```

### 2. 安装 Python 依赖

```bash
python3 -m pip install --user mujoco
python3 -m pip install --user numpy
python3 -m pip install --user pygame
```

如果系统中存在多个 Python 版本，请确认运行程序时使用的是安装了上述依赖的 `python3`。

## 快速开始

请在**工程根目录**运行主程序，因为程序会从当前目录加载 `mjcf/robot.xml`：

```bash
cd /home/stg/Independent_steering_wheel_robot
python3 robot_control.py
```

程序启动后会打开 MuJoCo 查看器和一个 Pygame 控制面板窗口。

## 操控说明

### Xbox 手柄

| 摇杆 | 功能 | 对应指令 |
|------|------|----------|
| 左摇杆 ↑ / ↓ | 前进 / 后退 | `target_Vy` |
| 左摇杆 ← / → | 左右平移 | `target_Vx` |
| 右摇杆 ← / → | 原地旋转 | `target_Vw` |

手柄输入带有 0.1 的死区滤波，用于避免摇杆轻微漂移。

### 无手柄时

如果没有检测到 Xbox 手柄，程序仍会打开 Pygame 控制面板和 MuJoCo 仿真窗口，但不会产生遥控速度输入。

## 系统架构

```text
Xbox 手柄 / Pygame 控制面板
      │
      ▼
remote/xbox_controller.py     ──── 读取摇杆输入 → target_Vx, target_Vy, target_Vw
      │
      ▼
AGV_update/AGV_cal.py         ──── 舵轮运动学解算
                                  ├─ 输入: 底盘速度指令 (Vx, Vy, Vw)
                                  ├─ 输入: 4 个转向关节当前角度（来自 MuJoCo 传感器反馈）
                                  ├─ 核心: optimize_swerve() — 优劣弧优化，最小化转向路径
                                  └─ 输出: 4 个转向目标角度 + 4 个轮速
      │
      ▼
robot_control.py              ──── 将解算结果写入 MuJoCo d.ctrl → mj_step → viewer.sync
      │
      ▼
AGV_update/robot_info_update.py
                              ──── 从 MuJoCo 传感器读取：
                                  ├─ steering_j{1-4}_pos / wheel_j{1-4}_pos  (关节位置)
                                  ├─ steering_j{1-4}_vel / wheel_j{1-4}_vel  (关节速度)
                                  ├─ IMU 欧拉角 (roll, pitch, yaw)
                                  └─ 三轴陀螺仪角速度
```

## 运动学解算说明

`AGV_update/AGV_cal.py` 实现了四轮独立舵轮（Swerve Drive）运动学解算：

1. **速度分配**：根据底盘目标速度 \((V_x, V_y, \omega)\) 和轮子几何位置，计算每个轮子的 x、y 方向速度分量。
2. **角度与速度求解**：使用 `math.hypot()` 计算轮速，使用 `math.atan2()` 计算转向角度。
3. **优劣弧优化**：`optimize_swerve()` 会根据当前转向角度选择更短的转向路径。当目标角度与当前角度偏差超过 ±90° 时，自动采用轮速反向 + 角度补偿的方式，减少舵轮不必要的大角度旋转。

## 模型参数 (`mjcf/robot.xml`)

| 参数 | 值 |
|------|-----|
| 底盘尺寸 | 2.0 m × 1.0 m（矩形） |
| 轮距 × 轴距 | 0.6 m × 1.6 m |
| 车轮半径 | 0.15 m |
| 轮胎宽度 | 0.1 m |
| 仿真步长 | 0.001 s (1 ms) |

转向关节使用 **position 控制器**（kp=5），驱动轮使用 **velocity 控制器**（kv=10），关节均带阻尼（damping=0.1）。

## 修改为自定义同构型机器人

可以通过修改 `mjcf/robot.xml` 来调整机器人尺寸、质量、惯量、关节参数和执行器参数。修改模型后，`robot_control.py` 会继续从 `mjcf/robot.xml` 加载最新模型进行仿真。

如果改变轮子位置、轮距、轴距或车轮半径，需要同步检查 `AGV_update/AGV_cal.py` 中的舵轮运动学参数，保证控制解算与 MuJoCo 模型一致。

## TODO

- [ ] 增加键盘遥控输入
- [ ] 增加更多地形与障碍物测试场景
- [ ] 进行路径规划