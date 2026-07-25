# ROS 2 四轮移动机器人仿真系统

基于 **ROS 2 Humble + Gazebo Classic** 的四轮差速（滑移转向）移动机器人仿真项目。

## 功能

- 四轮底盘 Xacro / URDF 建模
- Gazebo Classic 物理仿真与滑移转向驱动
- ROS 2 `cmd_vel` 速度控制与里程计输出
- C++ 轨迹指令节点：直行、原地转向、“8”字轨迹
- 一阶低通滤波，平滑速度指令
- RViz 可视化与 TF 发布

## 项目源码

完整源码、环境配置、构建与运行说明请见：

[ros2_mobile_robot_sim/README.md](./ros2_mobile_robot_sim/README.md)

## 环境

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11

  ## 运行演示

已在 Ubuntu 22.04、ROS 2 Humble 与 Gazebo Classic 11 环境完成运行验证。

- [观看 v0.1.0 仿真演示视频](https://github.com/xhb123456-oss/ros2_mobile_robot/releases/tag/v0.1.0)

演示内容：四轮机器人模型加载、直行、原地转向与“8”字轨迹控制。
