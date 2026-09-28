# Nav2 导航与静态绕障验证

最新进展（2026-09-23）：A* + AMCL 更新阈值 0.02 m / 0.02 rad + 既有 TF2/到点时间修复组合，短测 2/2、长测 20/20、进程重启复验 2/2，累计 24/24 按原标准通过。三组停车后最大位置误差 9.5085 cm、最大角度误差 5.4228°；汇总未记录到所筛选的告警，反馈恢复次数均为零。重启复验 560 次 DWB 评估无全候选无效周期；本组合长测尚未提供完整 DWB 解码结果，不能将此结论扩展到全部长测评估。
固定启动：`bash /mnt/hgfs/robot_project/scripts/start_amcl_update002_trial.sh`。测试必须显式指定 `config/replanning_02hz_trial.xml`。阶段证据与使用范围见 [AMCL_UPDATE_TRIAL.md](AMCL_UPDATE_TRIAL.md)。以下为历史记录；旧组合成功结果不覆盖其后出现的到点失败。

最新进展（2026-09-22）：到点检查时间修复工作区、每次重新检查位置、障碍评分 0.05、AMCL 0.2 s 与每目标 0.2 Hz 行为树组合，已完成短测 2/2、长测 20/20、进程重启复验 2/2，累计 24/24 段按原标准通过。
长测和重启复验的停车后采样均合格；各段已记录评估窗口未发现无有效轨迹周期。长测仍有一次全局规划失败及恢复，不能称为无故障版本。固定使用方式见 [GOAL_CHECK_VALIDATED.md](GOAL_CHECK_VALIDATED.md)。默认配置未覆盖，此组合须显式启用。
配置、完整启动命令、实测结果与结论边界以 [ARRIVAL_MARGIN_TRIAL.md](ARRIVAL_MARGIN_TRIAL.md) 为准。下文保留历史诊断，旧的“待执行”描述不代表当前状态；默认配置与显式启用的试验配置须区分。

先前结果（2026-09-19）：完整 launch 重启后，B→A 返程 action 返回 SUCCEEDED；停车 map TF
为 (0.899,-0.906) m、yaw=-8.064°，相对 A=(1,-1)、yaw=0 的位置偏差约 13.80 cm、朝向偏差 8.064°，
均在当前容差内。之前的旧位姿故障已通过重启恢复本次运行，根本触发原因和长期稳定性仍未确认。
以下保留分阶段诊断过程；较早的待验证描述以后续证据为准。

## 自动往返测试（2026-09-20，完整三轮尚未完成）

`scripts/repeat_navigation.py` 会实际驱动仿真机器人。保持完整导航 launch 运行，机器人应已完成上一目标并停在 A=(1,-1) 附近（距 A 不超过 0.3 m）。退出键盘控制，测试期间不再手动发送导航或速度命令。

```bash
cp /mnt/hgfs/robot_project/scripts/repeat_navigation.py ~/ros2_ws/src/ros2_mobile_robot_sim/scripts/
source /opt/ros/humble/setup.bash
python3 -u ~/ros2_ws/src/ros2_mobile_robot_sim/scripts/repeat_navigation.py --rounds 3
```

直接运行脚本无需重新编译。依次发送 B=(1.5,2)、A=(1,-1)，共 3 轮、6 个目标，目标朝向均为 yaw=0。每次先检查实时 TF、全局 footprint 的时间戳和位置，再用自动起点请求规划；路径首点距机器人超过 0.5 m 时不会发送本次导航目标。检查门限不改变 Nav2 参数。

导航期间记录位姿与反馈；持续观测异常、单目标超过 120 秒墙钟时间、导航失败或停车偏差超过 0.15 m / 0.15 rad 时停止后续目标。Ctrl+C 会请求取消本脚本持有的 action 并等待终态；若出现 `STOP NOT CONFIRMED`，在运行 navigation.launch.py 的终端按 Ctrl+C。取消请求本身不等于机器人已经停止。

每次运行生成独立目录 `~/ros2_ws/test_results/roundtrip_日期时间/`：

- `results.csv`：每次尝试的目标、action 状态、是否通过、导航耗时、停车偏差与失败原因；未到达阶段的指标留空。
- `events.jsonl`：规划点、观测时间戳、每秒位姿、goal ID 与取消确认。中断中的 CSV 状态可能仍为 REQUESTED，最终取消状态以事件记录为准。
- `summary.json`：计划数、尝试数、action 成功数及综合通过数。

完整通过时终端显示 `SUMMARY: planned=6, attempted=6, action_succeeded=6, passed=6`。位置与朝向误差基于 map TF 定位估计，不是真值定位误差；本测试没有碰撞接触检测，6 次通过也不能证明长时间不复发。

Windows 已完成计算、失败停止、旧路径拦截等离线测试；ROS 通信、取消及实际运动仍需 Ubuntu 实测。

停车朝向差异诊断（脚本版本 2）：用户已实测首个 B 通过，返回 A 的 action 成功但 +0.5 s map TF 朝向误差为 10.597°，超过 0.15 rad；运行中的 SimpleGoalChecker 容差确认为 0.15 rad。
新增 `--finish-diagnostics`，建议先运行 `--rounds 1 --finish-diagnostics`。
它以约 10 Hz 记录外部 map/base、odom/base、map/odom TF 及各自时间戳，并订阅 /odom、/cmd_vel、/cmd_vel_nav；
在收到结果时及约 +0.5、+1、+2、+3 秒打印 FINISH，记录 finish_sample。
CSV 和通过判定仍使用 +0.5 秒采样，不用后续更有利的数值替代。额外观测不发送速度或修正目标。
这些来自独立监听器、异步到达的观测，不等于控制器内部判定瞬间的同步位姿；速度必须结合其消息年龄判读。
版本 2 同时增加局部 footprint 与 odom TF 的新鲜度及中心位置检查，避免仅全局观测正常就继续运动。
本次修改未调整导航参数或验收容差，虚拟机诊断实测待完成。

状态：2026-09-17 用户截图确认 Nav2 激活，随后发送 map 坐标系目标 (1,-1) m、yaw=0，action 返回 SUCCEEDED。后续实时 map -> base_footprint 连续输出 (0.945,-0.922) m、yaw=-8.175°，位置偏差约 9.54 cm，位置与朝向均在配置容差内。第二个目标 (1.5,2.0) m 也返回 SUCCEEDED，截图显示绕过方块的规划路径及机器人到达另一侧，停车 TF 位置偏差约 4.37 cm、朝向偏差 8.308°，均在配置容差内。重复性尚未验证。AMCL 最近消息与 odom 的异步采样不能直接作为定位误差；详细数值和限制见 VALIDATION.md。

## 配置与边界

最新状态补充（画面日期 2026-09-17）：第二目标完成后返回 (1,-1) m 的第三次请求被接受但返回 ABORTED。
后续节点日志确认 DWB 反复报告 Resulting plan has 0 poses in it，随后 Received plan with zero length 和 Controller patience exceeded。
旋转、等待、后退恢复均报告完成，但导航仍失败；具体导致路径裁剪为空的输入条件尚未确定。
当前不能宣称双向导航可靠，暂不修改控制参数或重置仿真。

只请求规划的诊断脚本（在上一导航 action 已结束后运行）：

```bash
source /opt/ros/humble/setup.bash
python3 ~/ros2_ws/src/ros2_mobile_robot_sim/scripts/check_return_plan.py
```

脚本读取实际裁剪参数和 map/odom TF，向 ComputePathToPose 请求从当前位置到 (1,-1) 的路径，
打印点数、首末点、离机器人最近的点及按 Nav2 1.1.20 DWB 算法估计的保留点数。
它不会调用 NavigateToPose/FollowPath 或发布速度，但可能更新 RViz 的全局规划路径显示。
这是一条新规划，不是失败时路径的回放；只能帮助缩小原因。Windows 上仅完成 Python 语法检查，Ubuntu 运行待用户反馈。

首次 Ubuntu 诊断已复现：当前 map 位姿约 (1.4809,1.6972)，自动起点规划却只有 3 点、总长 0.1083 m，
首点约 (0.95,-0.92)，所有点均远离当前车体，裁剪保留 0 点。接下来运行显式起点对照：

```bash
python3 ~/ros2_ws/src/ros2_mobile_robot_sim/scripts/check_return_plan.py --explicit-start
```

此模式通过当前 map TF 填充 ComputePathToPose.start 并设置 use_start=true；同时打印规划器与全局代价地图的帧和时钟参数。
若显式起点恢复正常，优先检查规划器自动获取起点的 TF/帧配置；若仍异常，再调查规划插件。对照尚待运行，未修改导航配置。

用户首次运行显式起点模式时在首个控制器参数请求超时，尚未发送规划请求。
脚本已增加逐步进度与具体超时位置；可使用 `--explicit-start --skip-parameters` 跳过参数服务，
仅检查 TF 与显式起点规划，此模式不计算 DWB 裁剪点数。参数服务超时原因仍未确定。

2026-09-19 显式起点对照已成功：当前 map 位姿 (1.4385,1.6884)，路径 160 点、长 4.0262 m、
首点距机器人 0.0142 m，末点为目标 (1,-1)。这是只规划结果，不代表导航修复。
排查重点转向自动起点读取；需在当前现场复查自动模式和全局代价地图的坐标帧/车体轮廓，尚不调整运动参数。

后续证据：自动模式仍返回旧位置附近的 3 点路径；CLI daemon 故障已通过 stop/start 处理。
无 daemon 参数查询返回 Node not found，但全局 published_footprint 消息的时间戳仍为 1198.285 s，
轮廓中心约 (0.94484,-0.92109)，与自动规划旧起点一致，远落后于当前 TF 的时间和位置。
下一步正常停止完整 launch，确认旧进程退出后按本文件原命令重新启动，复测 A→B→A。
这是运行状态恢复；造成该发布者使用旧位姿的底层原因和是否存在重复实例仍未最终确定，不以重启成功代替可靠性验证。

重启后只规划检查已通过：TF 位于原点，自动起点路径 55 点、长 1.4020 m，首点距车体 0.0200 m。
下一步保持当前进程连续运行，依次完成 A=(1,-1)、B=(1.5,2)，在 B 停车后运行自动起点诊断，
确认返回路径首点仍贴近当前机器人，再测试 B→A。每次必须等前一目标明确 SUCCEEDED；失败时停止序列保留日志。

B 附近的自动返回规划已检查：仿真时间 207.820 s，机器人 map 位置 (1.3677,1.9894)，
自动规划返回 150 点、长 3.7658 m、首点距车体 0.0201 m，末点为 A=(1,-1)。
当前起点更新正常，下一步执行返回导航并记录 action 结果与停车 TF；尚未证明长期运行不再出现旧位姿。

返程停车 TF 已收到：连续五组 (0.899,-0.906) m、yaw=-8.064°，相对 A 点位置偏差约 13.80 cm，
朝向偏差 8.064°，均在配置容差内。随后补充截图确认返程 action 为 SUCCEEDED，目标 ID 为 `9950936c3c7c46b8b7008d7f0053fc0e`。

源码依据：
- https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_core/src/dwb_local_planner.cpp
- https://github.com/ros-navigation/navigation2/blob/humble/nav2_controller/src/controller_server.cpp

navigation.launch.py 组合已有房间、map_server/AMCL，以及官方 nav2_bringup 的 navigation_launch.py。
启动时读取已安装 Nav2 的默认参数，递归叠加 config/navigation_overrides.yaml，写入临时参数文件，正常退出时清理该文件。
沿用官方行为树与插件列表，规划器为 NavFn，控制器为 DWB；本项目没有自行实现规划或控制算法。
参数适配与启动文件由助手协助完成。

- `/scan` 用于障碍物层，`/odom` 与 TF 用于位姿；静态地图来自保存的 room_01.yaml。
- 机器人圆形避障半径 0.36 m，覆盖模型及四轮的平面外形；膨胀半径 0.65 m。
- 最高前进速度 0.2 m/s，最高角速度 0.6 rad/s；线加速度 0.5 m/s²、角加速度 1.0 rad/s²。
- 到达判定位置容差 0.15 m、角度容差 0.15 rad。配置值不等于实测精度。
- 官方启动将控制器输出送到 `/cmd_vel_nav`，经 velocity_smoother 输出 `/cmd_vel`。
- 里程计仍来自 Gazebo 世界状态；当前测试不能代表真实机器人或含漂移里程计下的导航性能。

参考：[Nav2 Humble 官方启动](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/launch/navigation_launch.py)、[默认参数](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/params/nav2_params.yaml)。

## 操作

退出键盘控制，关闭上一套 localization.launch.py 及单独启动的 RViz。
导航时不要同时运行 teleop 或其他 /cmd_vel 发布程序。

```bash
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup
cp -r /mnt/hgfs/robot_project/. ~/ros2_ws/src/ros2_mobile_robot_sim/
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select mobile_robot_sim --symlink-install
```

编译成功后：

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch mobile_robot_sim navigation.launch.py map:=$HOME/ros2_ws/maps/room_01.yaml
```

场景重启，机器人回到原点。RViz 应有 Map、RobotModel、LaserScan、Global Path 和 Local Path。
如果 Map 再次出现 No map received，取消再勾选 Map，确认地图恢复。路径会在发送目标后出现。

新开终端检查：

```bash
source /opt/ros/humble/setup.bash
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /controller_server
```

两个都为 active 后，发送第一个已选定的近距离目标：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: -1.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

预期机器人自行规划、行驶并在目标附近停止，命令最终报告 SUCCEEDED。
目标 (1,-1) 在场景空地。对世界几何采样检查，起点至目标的直线到障碍物最小中心净距约 1.061 m；
实际路径由 Nav2 和加载的地图决定，尚未验证实际栅格可达性。

到达后读取并记录位置：

```bash
timeout 10s ros2 topic echo /amcl_pose geometry_msgs/msg/PoseWithCovarianceStamped --once --qos-reliability reliable --qos-durability transient_local --field pose.pose
```

验收材料：action 结果、最终 AMCL 位姿、RViz 路径/目标附近停车画面。
单个无遮挡目标成功只代表基本链路工作，绕障、重复性、失败恢复与多点巡航需分别测试。

需要中止时，在启动 navigation.launch.py 的终端按 Ctrl+C，停止整套仿真。
仅关闭发送目标的命令窗口不能作为已取消导航的证明。

## 第二个目标：静态障碍绕行（action 已成功）

保持第一目标完成后的仿真运行，从约 (0.945,-0.922) m 发送目标 (1.5,2.0) m、yaw=0：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 1.5, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

基于 mapping_room.world 几何，起终点连线穿过 square_obstacle 的 x=[1.1,1.9]、y=[0.4,1.2] 范围。
目标点距最近障碍表面 0.8 m，大于配置的机器人半径 0.36 m；实际规划仍以加载地图和代价地图为准。
记录行驶中的 RViz 路径与 Gazebo 绕行画面、最终 action 状态及停车 TF。
2026-09-17 用户提供行驶中与完成后的两张截图：RViz 绿色规划路径绕过方块，Gazebo 与 RViz 中机器人从障碍一侧移动到另一侧；同一 goal ID 的终端结果为 SUCCEEDED。
本次静态绕障规划及目标完成已有截图支持，停车 TF 位置偏差约 4.37 cm、朝向偏差 8.308°，均在配置容差内。截图不是完整执行轨迹或碰撞检测记录，不据此量化最小安全距离、全程无接触或重复成功率。
