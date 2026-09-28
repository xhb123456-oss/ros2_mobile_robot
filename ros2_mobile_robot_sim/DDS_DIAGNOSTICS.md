# 位姿时间戳停更：Cyclone DDS 对照

更新日期：2026-09-20。Cyclone DDS 对照已运行，未消除局部 footprint 时间戳停更现象。

## 最新证据

`roundtrip_20260920_163736_980691` 中，B 点动作成功，停车后位置误差约 0.1218 m、角度误差约 0.689 度；但随后局部 footprint 时间戳一直为 175.267，而外部 TF 到达 178.267，局部消息接收间隔仍不足 0.1 s。全局 footprint 此次仍更新。测试在返程前停止，不能计为完整往返通过。

用户提供的 `/proc/PID/maps` 已确认 controller_server 与 planner_server 实际加载 `librmw_cyclonedds_cpp.so`。因此不能把问题归结为没有切换成功，也不能认为切换 DDS 已修复。

用户进一步确认 tf2、tf2_ros、tf2_py 安装版本均为 0.25.23；刷新软件源后候选版本仍为 0.25.23。上游 0.25.24 包含 TF2 ABBA 死锁修复（https://github.com/ros2/geometry2/pull/990）。这是待验证的根因线索，尚无进程堆栈证明本机故障就是该死锁。下一步独立编译 0.25.24 的 tf2 和 tf2_ros，运行上游测试，再验证导航进程实际加载路径并复测。系统软件包、导航参数和验收阈值保持原样。

以下保留原对照操作步骤。

独立监听器持续收到当前 TF/odom，而全局和局部 costmap 持续发布带旧位姿时间戳的 footprint。
完整 launch 重启曾恢复，但随后复发；生命周期 RESET 停在全局 costmap 清理。
这些证据不能单独证明 DDS 故障。本对照只改变 RMW 实现，不修改地图、模型、导航参数或验收阈值。
当前终端返回 rmw_fastrtps_cpp，尚未从旧导航进程库映射验证其实现。

1. 在旧 navigation.launch.py 终端 Ctrl+C，等待退出。确认下列检查无匹配结果后再启动另一套：

```bash
pgrep -af 'nav2_|lifecycle_manager|gzserver|gzclient|navigation.launch.py'
```

2. 安装 Cyclone 的 Humble 二进制包；安装失败时不继续启动：

```bash
sudo apt update
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

3. 启动终端设置环境（不写入 .bashrc），停止旧 CLI daemon 后启动全套仿真：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 daemon stop
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 -c "from rclpy.utilities import get_rmw_implementation_identifier; print(get_rmw_implementation_identifier())"
ros2 launch mobile_robot_sim navigation.launch.py map:=$HOME/ros2_ws/maps/room_01.yaml
```

4. 每个用于检查、目标发送及测试的新终端均先执行：

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

确认 /clock 和两个 published_footprint 时间戳接近当前仿真时间后，再发送 A=(1,-1)、yaw=0。
A 成功后运行 `python3 -u ~/ros2_ws/src/ros2_mobile_robot_sim/scripts/repeat_navigation.py --rounds 1 --finish-diagnostics`。
记录所有 FINISH 与事件文件，保留先前失败数据。单轮成功后仍需三轮及覆盖既往停更时间范围的较长观测，
不能仅凭一次重启后的成功就把差异归因于 RMW。

恢复原实现时先完整退出本套节点，再在所有相关终端设置 `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`，
停止旧 daemon 并重新启动。无需卸载 Cyclone 包。不要只切换诊断客户端而保留旧导航节点作为对照。

来源：
- https://github.com/ros2/ros2_documentation/blob/humble/source/Installation/RMW-Implementations/DDS-Implementations/Working-with-Eclipse-CycloneDDS.rst
- https://github.com/ros2/ros2_documentation/blob/humble/source/How-To-Guides/Working-with-multiple-RMW-implementations.rst
