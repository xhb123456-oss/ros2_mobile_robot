# ROS 2 / Nav2 移动机器人导航仿真系统

在 Ubuntu 22.04、ROS 2 Humble 和 Gazebo Classic 11 中，完成四轮差速底盘建模、基础运动控制、激光定位和固定地图自主导航，并通过录包分析排查到点误差与规划故障。

## 当前阶段结果（2026-09-23）

最终显式配置：NavFn A*、AMCL 更新阈值 0.02 m / 0.02 rad、TF 容差 0.2 s、到点时间检查修复、TF2 修复工作区和每目标 0.2 Hz 重规划行为树。

- 固定 A=(1,-1)、B=(1.5,2) 路线：短测 2/2、长测 20/20、导航及仿真进程重启复验 2/2，共 24/24 段通过。
- 原验收标准：导航成功，且返回结果后 +0.5 s 位置误差不超过 0.15 m、角度误差不超过 0.15 rad；+1、+2、+3 s 同时留样。
- 三组停车后采样最大位置误差 9.5085 cm、最大角度误差 5.4228°；采样反馈恢复次数均为零，汇总日志筛选未发现告警。
- 重启复验 560 次 DWB 评估未发现无有效候选周期；本配置长测尚未提供完整 DWB 解码结果，不将此结论扩展到所有长测评估。

这是当前房间仿真的阶段基线，不是任意地图、动态障碍或实物机器人的可靠性保证。里程计使用 Gazebo 世界状态；停车误差由地图坐标系定位/TF 计算，不等于独立真值测量。源码修复和脚本开发有 AI 工具协助，运行验证在 Ubuntu 虚拟机中完成。

## 演示视频

[观看一轮 B→A 导航演示（约 81 秒）](media/navigation_demo_20260923.mp4)

![演示结束时的测试结果](media/demo_result.png)

本次演示终端显示 2/2 通过，最终停车采样误差 3.92 cm / 4.471°。
演示结果目录为 `roundtrip_20260923_160838_186370`，与上面的三组 24 段阶段验收独立记录。
视频已检查关键帧和结束画面；不是对每一帧、音频或完整日志的审核。

## 阅读入口

- [配置、完整证据及固定启动方式](AMCL_UPDATE_TRIAL.md)
- [交付清单、演示步骤和面试提纲](PROJECT_HANDOFF.md)
- [导航与历史诊断](NAVIGATION.md)
- [地图构建](MAPPING.md) / [定位](LOCALIZATION.md)

在现有已配置好的虚拟机中启动：

```bash
bash /mnt/hgfs/robot_project/scripts/start_amcl_update002_trial.sh
```

此命令依赖 `~/tf2_fix_ws`、`~/nav2_goal_fix_ws`、`~/ros2_ws` 及已保存地图。仅下载本项目不能自动获得两个已编译的修复工作区。每次导航测试仍需显式选择 `config/replanning_02hz_trial.xml`，启动脚本不会改写默认行为树。

---

## 基础仿真与历史实现说明

一个用于验证四轮差速（滑移转向）底盘基础运动控制的 ROS 2 + Gazebo Classic 仿真项目。项目包含：

- Xacro/URDF 机器人模型：底盘、四个驱动轮与惯性参数；
- Gazebo Classic 场景与配置为两对轮子的 ROS 2 差速驱动插件；
- `cmd_vel` 速度控制接口；
- C++ 轨迹指令节点：直行、定点转向、"8" 字轨迹，并对速度指令进行一阶低通滤波；
- RViz 可视化与 TF 发布。

## 环境

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11

### Humble 驱动兼容性修正（2026-09-17）

新虚拟机中已确认安装了 `libgazebo_ros_diff_drive.so`，没有原配置引用的
`libgazebo_ros_skid_steer_drive.so`。当前模型改用官方差速驱动插件，设置
`num_wheel_pairs=2`，按前、后顺序配置左右轮关节，并分别填写两对轮子的轮距和轮径。

输入为 `/cmd_vel`（`geometry_msgs/msg/Twist`）；配置启用 `/odom` 和
`odom -> base_footprint` TF。这里显式使用 `odometry_source=1`，里程计来自
Gazebo 世界状态，不是编码器积分，也不是自行实现的定位算法。
另用 Gazebo 关节状态插件发布四个轮子的 `/joint_states`，由
`robot_state_publisher` 发布车轮 TF，避免驱动插件重复发布这些 TF。

这次修改由助手依据官方插件源码协助完成。配置已做 XML 和关节映射静态检查。
用户随后在 Ubuntu 虚拟机中完成编译、模型加载、`/odom` 读取、手动直行、
零速度停车及原地转向验证，并提供四轮 `/joint_states` 截图。
这些是交互式检查，尚未形成重复性能测试；具体证据和待验证项目见 [VALIDATION.md](VALIDATION.md)。
参考：[官方 3.9.0 驱动源码](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/3.9.0/gazebo_plugins/src/gazebo_ros_diff_drive.cpp)。

安装依赖：

```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-gazebo-ros-pkgs ros-humble-xacro
```

## 构建

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/xhb123456-oss/ros2_mobile_robot.git
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

### 雷达最小验证场景

助手协助添加官方 `libgazebo_ros_ray_sensor.so` 插件，使用 CPU ray 传感器，
输出 `/scan`。固定坐标系 `laser_link` 相对 `base_link` 为 `[0, 0, 0.22]` m。
配置为一圈 360 个采样点、10 Hz、量程 0.12–8 m，暂不加噪声。
配置格式参考[官方 ray 插件说明](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/3.9.0/gazebo_plugins/include/gazebo_plugins/gazebo_ros_ray_sensor.hpp)。

关闭旧仿真，再运行：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch mobile_robot_sim simulation.launch.py use_rviz:=false world_file:=lidar_test.world
```

保持机器人静止，另开终端：

```bash
source /opt/ros/humble/setup.bash
python3 ~/ros2_ws/src/ros2_mobile_robot_sim/scripts/check_scan.py
ros2 run tf2_ros tf2_echo base_link laser_link
```

测试墙中心 x=2 m、厚度 0.2 m，近表面在 x=1.90 m。
初始位置前方最近于零角度的一束应约为 1.90 m；没有障碍物的方向出现 `inf` 正常。
检查脚本使用消息时间戳计算接收频率，读取 11 帧后退出，最多等 20 秒；
打印的是测量数据与预期对照，不自动判定通过。TF 预期平移 `[0, 0, 0.22]` m、无旋转。
默认场景仍为 `empty_track.world`，只有指定 `world_file` 才会加载测试墙。

2026-09-17 用户截图确认：360 点、前方距离 1.900 m、按消息时间戳计算频率
10.00 Hz、雷达 TF 平移 `[0, 0, 0.22]` m 且无旋转。详细证据见 VALIDATION.md。

现有仿真保持运行时，可以单独启动 RViz：

```bash
source /opt/ros/humble/setup.bash
rviz2 -d ~/ros2_ws/src/ros2_mobile_robot_sim/rviz/mobile_robot.rviz --ros-args -p use_sim_time:=true
```

配置固定坐标系为 `odom`；模型订阅 `/robot_description`，扫描订阅 `/scan`。
扫描用红点显示；测试墙的可见表面应形成一排点。RViz 不直接显示 Gazebo 的墙模型。
2026-09-17 用户截图已确认模型、红色墙面扫描点可见，整体状态为 Ok；随后用户反馈短暂转向期间扫描仍对齐墙面。
配置字段参考 [Nav2 Humble 官方 RViz 配置](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/rviz/nav2_default_view.rviz)。

### 话题列表

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 发布线速度与角速度控制指令 |
| `/odom` | `nav_msgs/msg/Odometry` | Gazebo 驱动插件发布的里程计信息 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 机器人坐标变换 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 仿真二维雷达，静态测距与频率已检查 |

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

## 简历表述边界

已验证模型加载、手动速度控制、仿真里程计、关节反馈、雷达静态测距及 RViz 显示；C++ 轨迹节点尚未在当前环境验证。
已集成官方 SLAM Toolbox，用户截图确认房间地图及两个障碍物轮廓显示，PGM/YAML 保存成功，操作见 [MAPPING.md](MAPPING.md)。
map_server 与 AMCL 的地图重载、节点激活及运动后的位姿更新已验证，操作见 [LOCALIZATION.md](LOCALIZATION.md)。
已集成官方 Nav2，完成近距离目标和静态障碍绕行目标，均返回 SUCCEEDED 且停车 TF 在配置容差内。曾出现返程 ABORTED：诊断发现全局代价地图使用旧位姿，自动规划从旧位置开始。完整 launch 重启后，返程也返回 SUCCEEDED，停车位置偏差约 13.80 cm、朝向偏差 8.064°，均在容差内。尚未证明长期稳定性，旧位姿故障的触发原因仍待查；见 [NAVIGATION.md](NAVIGATION.md)。
未实现真实传感器融合。简历中不要把未验证的导航功能写成已完成；应区分官方算法、助手协助配置和用户独立工作。
