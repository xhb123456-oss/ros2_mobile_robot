# 重新加载地图与 AMCL 定位

状态：Ubuntu 中已确认 map_server 与 amcl 为 active，保存地图能重新加载；重新勾选 RViz Map 后地图显示正常。显式 reliable + transient_local 查询已读到 AMCL 位姿。运动后截图确认位置变为约 (0.661,-0.004) m，偏航约 45.489°，扫描与墙面大致对齐；未量化误差。

## 本次检查的含义

地图已保存到 Ubuntu 的 `/home/xxx/ros2_ws/maps/room_01.yaml` 和同目录 PGM。
重新启动相同房间，map_server 读取文件发布静态 `/map`；AMCL 根据 `/scan`
与里程计 TF 估计地图中的位姿，并发布 `/amcl_pose` 和 `map -> odom`。
本启动文件不启动 SLAM Toolbox，也不启动导航控制器。

机器人重启后在 `(0, 0, yaw=0)`，因此给 AMCL 同样的初始估计。这不是从未知位置全局定位。
里程计仍来自 Gazebo 世界真实状态；不能据此宣称真实机器人定位精度。
AMCL 使用官方实现；启动配置与参数适配由助手协助完成。
参考：[Nav2 Humble 定位启动](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/launch/localization_launch.py)、[AMCL 参数实现](https://github.com/ros-navigation/navigation2/blob/humble/nav2_amcl/src/amcl_node.cpp)。

## 操作

先在键盘终端按 k 停车，再 Ctrl+C 退出；停止旧 mapping.launch.py 及单独打开的 RViz。
旧 SLAM Toolbox 必须退出，避免与 AMCL 同时发布 map -> odom。

```bash
sudo apt install -y ros-humble-nav2-amcl ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager
cp -r /mnt/hgfs/robot_project/. ~/ros2_ws/src/ros2_mobile_robot_sim/
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select mobile_robot_sim --symlink-install
```

看到编译成功后执行（地图参数使用绝对路径）：

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch mobile_robot_sim localization.launch.py map:=$HOME/ros2_ws/maps/room_01.yaml
```

Gazebo 与 RViz 应一起打开。先保持机器人静止，确认房间地图、模型及扫描显示。
初始扫描可能有小幅错位；此阶段先确认地图加载和 AMCL 发布，不据单次截图判定最终精度。
新开终端：

```bash
source /opt/ros/humble/setup.bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
timeout 10s ros2 topic echo /amcl_pose geometry_msgs/msg/PoseWithCovarianceStamped --once --qos-reliability reliable --qos-durability transient_local --field pose.pose
```

两个生命周期节点都应为 active；位姿应在初始原点附近。
AMCL 停车后不一定持续发布新位姿，查询显式使用 reliable + transient_local 以接收保留的最近一条。
记录生命周期与位姿输出，并拍摄 RViz 画面。移动时跟踪、定位误差、导航目标另行检查。

如果地图未加载或定位未输出，保留启动终端报错；先不要重复启动第二套进程。

本次排查中 `/map` 已能通过命令接收，但 RViz 显示 No map received；取消并重新勾选 Map 后恢复。
这与 RViz 重置显示相符，时钟切换作为触发原因尚未通过完整日志证实。
读取到的初始位姿为位置全零、单位四元数，与配置初值一致；不能单凭此项确认运动跟踪成功。
