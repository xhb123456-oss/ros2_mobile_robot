# SLAM Toolbox 建图练习

当前状态：2026-09-17 用户截图已确认房间、机器人和地图显示，RViz 整体状态为 Ok，地图中可见房间边界和两个障碍物轮廓。随后确认 room_01.pgm 与 room_01.yaml 保存成功；重新加载与定位见 LOCALIZATION.md，尚待运行验证。

## 输入与输出

- 输入：仿真 `/scan`，以及 `odom -> base_footprint -> base_link -> laser_link` TF。
- 输出：SLAM Toolbox 发布的 `/map`（OccupancyGrid）和 `map -> odom` TF。
- 场景：四面墙的中心线围成 8×6 m 房间，净空 7.8×5.8 m，内部两个不同位置和尺寸的障碍物。
- 地图分辨率 0.05 m/像素，更新间隔 2 秒；采样位移/角度阈值为 0.1 m/0.1 rad。
- 读取已安装 SLAM Toolbox 的在线异步默认 YAML，再叠加项目 config/slam_mapping.yaml。

算法使用官方 SLAM Toolbox；场景、启动与参数适配由助手协助完成。
当前里程计仍是 Gazebo 世界真实状态，所以这次练习不代表真实轮式里程计漂移下的定位性能。

参考：[Humble 官方启动文件](https://github.com/SteveMacenski/slam_toolbox/blob/humble/launch/online_async_launch.py)、[默认参数](https://github.com/SteveMacenski/slam_toolbox/blob/humble/config/mapper_params_online_async.yaml)、[键盘节点](https://github.com/ros2/teleop_twist_keyboard/blob/humble/teleop_twist_keyboard.py)。

## 操作

先用 Ctrl+C 停止旧仿真、旧 RViz 及速度发布程序，避免重复节点。

```bash
sudo apt update
sudo apt install -y ros-humble-slam-toolbox ros-humble-teleop-twist-keyboard
```

Windows 源码已更新，通过现有 VMware 共享复制：

```bash
cp -r /mnt/hgfs/robot_project/. ~/ros2_ws/src/ros2_mobile_robot_sim/
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select mobile_robot_sim --symlink-install
```

编译成功后启动整个建图环境：

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch mobile_robot_sim mapping.launch.py
```

等场景与机器人加载，RViz 应出现初始黑白地图、机器人及红色扫描点。
启动初期尚未建立 map TF 时可能暂时出现等待；持续没有地图应检查启动终端报错。

另开终端，输入法切英文，运行：

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.15 -p turn:=0.4
```

保持键盘焦点在该终端：i 前进、逗号后退、j 左转、l 右转、k 停止。
松开按键不会自动停车；切换窗口前先按 k。可短距离走动、停下、原地转向后继续走，绕开障碍物。
通过移动观察被障碍物遮住的区域；单次原地旋转不能看到障碍物背面。

如果键盘窗口意外退出但车仍运动，可在普通终端执行：

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
```

## 本阶段验收

1. RViz 的 Map 显示收到地图，整体状态最终为 Ok。
2. 移动过程中地图逐渐补全，墙面轮廓和两个障碍物与 Gazebo 场景对应。
3. 停车后提供 RViz 地图截图；保留启动终端报错（如果有）。

地图不会自动保存。首次启动未自动打开 RViz 的参数作用域问题已在 Windows 源码修正，仍待重新同步后验证一键启动；当前会话手动打开 RViz 已成功。

## 保存当前栅格地图

在键盘控制终端按 k 停车，保持 Gazebo 和建图节点运行。另开终端执行：

```bash
sudo apt install -y ros-humble-nav2-map-server
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/maps
ros2 run nav2_map_server map_saver_cli -t /map -f ~/ros2_ws/maps/room_01 --fmt pgm --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0
```

命令应报告 Map saved successfully，再检查：

```bash
ls -lh ~/ros2_ws/maps/room_01.pgm ~/ros2_ws/maps/room_01.yaml
cat ~/ros2_ws/maps/room_01.yaml
```

PGM 为地图像素，YAML 记录图像路径、分辨率、原点和阈值；两个文件一起保留。
当前预期分辨率为 0.05 m/像素；原点以保存结果为准。重复保存同名前缀会覆盖同名地图文件。
这里保存的是后续定位/导航使用的栅格地图，不包含 SLAM Toolbox 的位姿图状态。
参考：[Nav2 Humble map_saver_cli](https://github.com/ros-navigation/navigation2/blob/humble/nav2_map_server/src/map_saver/main_cli.cpp)。
