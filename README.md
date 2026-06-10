# 独立舵轮机器人仿真 (Independent Steering Wheel Robot)

基于 **MuJoCo** 的四轮独立转向（4WIS / 舵轮 / Swerve Drive）移动机器人仿真项目。

## 项目概述

本项目在 MuJoCo 物理引擎中构建了一台**四轮独立舵轮（Swerve Drive）机器人**，每个轮子均可独立转向和独立驱动，实现全向移动能力。代码包含完整的运动学解算、闭环反馈读取、手柄/键盘遥控器以及带斜坡的测试场景。

## 文件结构

当前工程根目录为 `Independent_steering_wheel_robot`，主要文件结构如下：

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
├── .venv/                        # 本地 Python 虚拟环境（重新创建后生成）
├── .gitignore                    # Git 忽略规则
└── README.md                     # 本文件
```

> `__pycache__/` 为 Python 自动生成的缓存目录，不属于项目源码；如果出现可以忽略或删除。

## 系统架构

```text
Xbox 手柄 / 控制面板窗口
      │
      ▼
remote/xbox_controller.py     ──── 读取摇杆输入 → target_Vx, target_Vy, target_Vw
      │
      ▼
AGV_update/AGV_cal.py         ──── 舵轮运动学解算
                                  ├─ 输入:  底盘速度指令 (Vx, Vy, Vw)
                                  ├─ 输入:  4 个转向关节当前角度（来自 MuJoCo 传感器反馈）
                                  ├─ 核心:  optimize_swerve() — 优劣弧优化，最小化转向路径
                                  └─ 输出:  4 个转向目标角度 + 4 个轮速
      │
      ▼
robot_control.py              ──── 将解算写入 MuJoCo d.ctrl → mj_step → viewer.sync
      │
      ▼
AGV_update/robot_info_update.py
                              ──── 从 MuJoCo 传感器读取：
                                  ├─ steering_j{1-4}_pos / wheel_j{1-4}_pos  (关节位置)
                                  ├─ steering_j{1-4}_vel / wheel_j{1-4}_vel  (关节速度)
                                  ├─ IMU 欧拉角 (roll, pitch, yaw)
                                  └─ 三轴陀螺仪角速度
```

## 依赖

- Python ≥ 3.10
- [MuJoCo](https://mujoco.org/) (`mujoco`)
- [Pygame](https://www.pygame.org/) (`pygame`)
- NumPy (`numpy`)

## 虚拟环境与安装

在工程根目录执行：

```bash
cd /home/stg/Independent_steering_wheel_robot
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install mujoco pygame numpy
```

后续每次运行前只需要重新激活虚拟环境：

```bash
cd /home/stg/Independent_steering_wheel_robot
source .venv/bin/activate
```

## 快速开始

请在**工程根目录**运行主程序，因为程序会从根目录加载 `mjcf/robot.xml`：

```bash
cd /home/stg/Independent_steering_wheel_robot
source .venv/bin/activate
python robot_control.py
```

启动后会自动弹出 MuJoCo 被动渲染窗口，同时弹出 Pygame 控制面板窗口。

### 控制方式

**Xbox 手柄：**

| 摇杆 | 功能 | 对应指令 |
|------|------|----------|
| 左摇杆 ↑↓ | 前后移动 | target_Vy (\(v_y\)) |
| 左摇杆 ←→ | 左右平移 | target_Vx (\(v_x\)) |
| 右摇杆 ←→ | 原地旋转 | target_Vw (\(\omega\)) |

> 手柄自带 0.1 死区滤波，无需额外配置。

**无手柄时：** 控制面板窗口依然会打开，摇杆读取自动跳过。

## 运动学解算说明

`AGV_update/AGV_cal.py` 实现了舵轮（Swerve Drive）的四轮独立运动学解算：

1. **速度分配**：根据底盘目标速度 \((V_x, V_y, \omega)\) 和轮子几何位置，计算每个轮子的 \(x\)、\(y\) 方向速度分量。
2. **角度与速度求解**：`math.hypot()` 计算轮速，`math.atan2()` 计算转向角度。
3. **优劣弧优化**（`optimize_swerve`）：如果目标转向角度与当前角度偏差超过 ±90°，自动选择反向路径（轮速取反 + 角度补偿），确保转向总在 ±90° 范围内，减少不必要的扫过角度。

## 模型参数 (`mjcf/robot.xml`)

| 参数 | 值 |
|------|-----|
| 底盘尺寸 | 2.0 m × 1.0 m（矩形） |
| 轮距 × 轴距 | 0.6 m × 1.6 m |
| 车轮半径 | 0.15 m |
| 轮胎宽度 | 0.1 m |
| 仿真步长 | 0.001 s (1 ms) |

转向关节使用 **position 控制器**（kp=5），驱动轮使用 **velocity 控制器**（kv=10），关节均带阻尼（damping=0.1）。

## 测试场景 (`mjcf/obstacles.xml`)

场景中沿 X 轴依次放置了 5 个斜坡，默认坡度约 **9°**（`euler` 中的 0.157 rad）。可通过修改 `obstacles.xml` 中 `<body>` 的 `euler` 参数调整坡度，用于测试机器人的爬坡性能。

## 常见问题

### `ModuleNotFoundError: No module named 'mujoco'`

说明当前没有激活虚拟环境，或依赖未安装。请执行：

```bash
source .venv/bin/activate
python -m pip install mujoco pygame numpy
```

### 找不到 `mjcf/robot.xml`

请确认运行命令是在工程根目录执行的：

```bash
pwd
# 应显示 /home/stg/Independent_steering_wheel_robot
```

## 许可

本项目仅供学习和研究参考。
