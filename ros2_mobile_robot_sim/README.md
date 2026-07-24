# ROS 2 四轮移动机器人仿真系统

一个用于验证四轮差速（滑移转向）底盘基础运动控制的 ROS 2 + Gazebo Classic 仿真项目。项目包含：

- Xacro/URDF 机器人模型：底盘、四个驱动轮与惯性参数；
- Gazebo Classic 场景与四轮滑移转向插件；
- `cmd_vel` 速度控制接口；
- C++ 轨迹指令节点：直行、定点转向、"8" 字轨迹，并对速度指令进行一阶低通滤波；
- RViz 可视化与 TF 发布。

## 环境

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11

安装依赖：

```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-gazebo-ros-pkgs ros-humble-xacro
```

## 构建

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <你的仓库地址> mobile_robot_sim
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select mobile_robot_sim
source install/setup.bash
```

## 运行

启动 Gazebo、机器人模型和 RViz：

```bash
ros2 launch mobile_robot_sim simulation.launch.py
```

另开终端并执行以下任一命令。

### 直行

```bash
ros2 run mobile_robot_sim trajectory_commander --ros-args -p mode:=straight
```

### 原地转向

```bash
ros2 run mobile_robot_sim trajectory_commander --ros-args -p mode:=rotate
```

### "8" 字运动

```bash
ros2 run mobile_robot_sim trajectory_commander --ros-args -p mode:=figure_eight
```

`filter_alpha` 参数范围建议为 0 到 1；值越小，速度变化越平滑但响应越慢：

```bash
ros2 run mobile_robot_sim trajectory_commander --ros-args -p mode:=figure_eight -p filter_alpha:=0.18
```

停止机器人：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
```

## 关键话题

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 发布线速度与角速度控制指令 |
| `/odom` | `nav_msgs/msg/Odometry` | Gazebo 驱动插件发布的里程计信息 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 机器人坐标变换 |

## 项目结构

```text
mobile_robot_sim/
├── launch/                 # Gazebo、模型生成、RViz 启动文件
├── src/                    # C++ 轨迹指令节点
├── urdf/                   # Xacro/URDF 模型
├── worlds/                 # Gazebo 场景
├── rviz/                   # RViz 配置
├── CMakeLists.txt
└── package.xml
```

## 表述边界

本项目验证的是四轮底盘的模型加载、速度控制、里程计与基础轨迹运动；未实现 SLAM、Nav2 路径规划或真实传感器融合。
