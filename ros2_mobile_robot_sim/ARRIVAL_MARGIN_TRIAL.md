# 到达余量试验（2026-09-20）

最新结论（2026-09-21，10 轮完成）：独立到达余量 + AMCL 0.2 s 方案的连续测试 20/20 原验收通过；+0.5、+1、+2、+3 s 各 20 个采样完整且无超限，观测告警 0。该方案累计原验收 30/30（含运行时试验及重启复验）。仍有进展失败、规划失败、无有效轨迹及恢复行为；不是无故障导航或长期故障彻底消除的证明。详细数据与日志边界见文末。

状态（2026-09-21 更新）：到达余量配置配合 AMCL 1.0 s 的原验收为 14/14，但曾有停车后角度超限。随后 AMCL 0.2 s 配合相同到达余量配置累计原验收 10/10（运行时设置 8 段、完整重启从文件加载后 2 段），已提供的 +1～3 s 采样均合格。重启加载的六项参数及控制器、规划器的修复 TF2/Cyclone DDS 库已核对。途中停滞与恢复仍存在，更长时间验证尚未执行；不能认定长期稳定性或持续精度已获保证。下方按时间保留历史方案，最新结果见文末。

基线证据：TF2 0.25.24 已从独立工作区加载到控制器和规划器；修复后一次往返及三轮六段导航完成，测试期间未触发 footprint 时间戳过期。三轮第一段曾触发 4 次恢复并耗时 94.1 s；第二轮 A 的 +0.5 s 角度通过，但 +1 至 +3 s 超出原角度阈值。不能据此认定长期稳定性或停车后持续精度完全通过。

运行时最低速度试验已确认 min_speed_xy=0.03、min_speed_theta=0.12。结果目录 roundtrip_20260920_203244_377450：仅 B 动作完成，29.46 s，反馈未显示恢复；+0.5 s 地图位置误差 0.150261 m，超过原 0.15 m 阈值，返程未发送。

到达后的里程计位置：+0 为 (1.304297,2.066392)，+0.5 为 (1.312493,2.066020)，+1 为 (1.312493,2.066020)。因此最初约 8.2 mm 是实际停车运动（本模型的 odom 来自 Gazebo world）。+0.5 至 +1 地图位姿变化约 20.1 mm，里程计仅变化约 0.00000029 m，支持 TF/定位估计变化。独立查询的 map→odom 是不同时间戳，不能直接当成同一时刻的组合证据。

独立配置 config/arrival_margin_trial.yaml 将 GoalChecker 的 XY/yaw 到达条件收紧为 0.10 m / 0.10 rad，并将 DWB 的 XY 转向窗口同步改为 0.10 m。保留最低速度试验值。目标是在原验收上限内留余量，并不保证定位修正总小于该余量。验收脚本及阈值保持原样。

navigation.launch.py 新增可选 nav_overrides 参数，在官方默认参数、项目默认参数之后递归合并。省略参数仍使用原配置。RotateToGoal 的窗口在初始化读取，不能仅靠运行中设置参数确认生效；使用完整重启，不重复已出现过挂起的生命周期 RESET。

在已同步并编译项目的 Ubuntu 启动终端，依次加载 /opt/ros/humble/setup.bash、~/ros2_ws/install/setup.bash、~/tf2_fix_ws/install/local_setup.bash，设置 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp，启动：

```bash
ros2 launch mobile_robot_sim navigation.launch.py map:=$HOME/ros2_ws/maps/room_01.yaml nav_overrides:=$HOME/ros2_ws/src/ros2_mobile_robot_sim/config/arrival_margin_trial.yaml
```

确认控制器/规划器仍加载修复版 TF2，核对 GoalChecker XY/yaw 和 FollowPath XY 均为 0.10，最低速度为 0.03/0.12。重启后先到 A=(1,-1),yaw=0，再运行一轮 --finish-diagnostics；检查 +0.5 s 原验收及 +1～3 s 后续漂移。未通过则保留原始日志，不选择最优样本改写判定。

撤回：退出这套 launch，省略 nav_overrides 参数重新启动；最低速度也回到项目原默认值。若要与本次最低速度试验直接对照，运行时恢复 min_speed_xy=0.03、min_speed_theta=0.12。

依据：https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_critics/src/rotate_to_goal.cpp

## 2026-09-21 用户实测反馈

证据来源：用户提供的 Ubuntu 终端文字及截图；助手未直接运行虚拟机。共享目录写入失败，原始结果仍在 Ubuntu，Windows 尚未取得完整结果文件。以下三轮数据来自用户打印的 summary.json、results.csv 和 events.jsonl 停车采样。

单轮 roundtrip_20260921_140602_382770：2/2 通过。B：87.2 s，最后打印恢复次数 3，+0.5 s 为 9.93 cm / 0.447°；A：33.5 s，最后打印恢复次数 0，+0.5 s 为 5.86 cm / 4.846°。两段 +1、+2、+3 s 采样均在原阈值内。去 B 途中仍有停滞，最低速度设置未消除此现象。

三轮 roundtrip_20260921_140919_705193：planned=6、attempted=6、action_succeeded=6、passed=6、exit_code=0。

| 段 | 目标 | wall_s | +0.5 s 位置误差 cm | +0.5 s 角度误差 ° |
|---|---|---:|---:|---:|
| 1 | B | 26.762 | 5.675 | 5.322 |
| 2 | A | 32.762 | 7.322 | 5.012 |
| 3 | B | 30.972 | 8.262 | 4.271 |
| 4 | A | 32.882 | 7.539 | 8.105 |
| 5 | B | 25.962 | 9.350 | 5.978 |
| 6 | A | 32.364 | 6.329 | 4.619 |

第 4 段是关键限制：+0 s 为 7.620 cm / 8.598°；+0.5 s 为 7.539 cm / 8.105°，按原规则通过；+1、+2、+3 s 均为 11.002 cm / 10.595°，角度超过 0.15 rad（约 8.594°）。其余五段在 +1、+2、+3 s 的采样均在原阈值内。这里只评价离散采样，不能证明采样间连续满足阈值。不得用后续超限回写原 PASS，也不能用原 PASS 宣称停车后持续精度全部合格。

用户筛选输出未包含 observation_warning 或 bad_observation。三轮各段耗时约 26～33 s，但恢复次数未写入当前脚本的结果文件，不能从耗时推断恢复次数为零。第 6 段截图可见恢复反馈为 0，其余各段需原终端记录。

下一步：保持配置不变，提取第 4 段 finish_sample 中的 map/base、odom/base、map/odom 位姿、各自时间戳及 motion，区分实际停车运动与定位估计变化。各 TF 为独立查询，不按同一时刻直接组合；本模型 odom 来自 Gazebo world。分析前不进一步收紧参数，不把三轮通过视为长期稳定性结论。

### 第 4 段停车记录补充

用户随后提供了五条完整 finish_sample。+0 至 +0.5 s，odom 位移约 1.223 mm，yaw 增加约 0.493°，存在实际停车运动；+0 时 /cmd_vel_nav 已为零，但 /cmd_vel.wz=0.107895、odom.wz=0.124242 rad/s，说明动作完成时尚未完全停稳。

+0.5 至 +1 s，map/base 位移约 34.817 mm、yaw 变化约 -2.490°；odom/base 位移仅约 2.60e-7 m、yaw 变化约 -9.30e-8°。两端 odom 角速度接近零、/cmd_vel 为零。因此这段超限有力支持定位/TF 估计变化，不能解释为车体又转了约 2.49°。后续 +2、+3 s 的 odom 同样基本不动。

时间戳细节：+0.5 s 的 map/base 与 odom/base 时间戳均为 588.076，而独立查询的最新 map/odom 时间戳为 589.027；+1 s 对应为 588.576 与 589.527。最新 map/odom 五次值相同，不代表相应历史时刻的校正相同。项目 localization.yaml 中 AMCL transform_tolerance=1.0，Nav2 1.1.20 AMCL 会将发布 TF 的时间戳向未来偏移；这与约 0.951 s 的时间差相符，但运行时参数及校正生效过程仍需核对。不得直接把不同时间戳的最新变换相加证明根因。

全局 footprint 在 +0.5 s 仍为 587.196，+1、+2、+3 s 分别推进至 588.196、589.176、590.176，且位置差随后收敛至约 0.625 mm；这组样本不支持旧的长期停更现象复发。下一步先核对运行时 AMCL transform_tolerance，并视需要记录带时间戳的 /tf、/amcl_pose、/scan 等数据，暂不调整参数。

## 带录制的第二次三轮

用户已通过 ros2 param get 确认运行时 AMCL transform_tolerance=1.0，未调整参数。roundtrip_20260921_142219_599090 原验收 6/6 通过，各段 +1、+2、+3 s 采样亦全部在原阈值内；不能覆盖之前的超限证据。

| 段 | 目标 | wall_s | +0.5 s 位置 cm | +0.5 s 角度 ° | 终端打印的最大恢复计数 |
|---|---|---:|---:|---:|---:|
| 1 | B | 52.863 | 8.639 | 4.492 | 0 |
| 2 | A | 33.762 | 4.859 | 7.197 | 0 |
| 3 | B | 26.872 | 9.301 | 2.726 | 0 |
| 4 | A | 32.462 | 7.229 | 4.752 | 0 |
| 5 | B | 26.361 | 9.101 | 2.885 | 0 |
| 6 | A | 31.482 | 6.059 | 5.301 | 1 |

最大 +0.5 s 位置误差 9.301 cm，角度误差 7.197°；+1～3 s 最大位置误差 11.326 cm、角度误差 5.340°。首段距离反馈数次停留约 3.33 m，尽管打印恢复计数为 0，仍有耗时问题。第 6 段记录到恢复计数 1，不能因耗时 31.5 s 就称为零恢复。以上计数是终端周期打印观察值，不是完整恢复事件审计。

第 2 段 +0.5 至 +1 s，map/base yaw 从 -7.197° 变为 -3.990°，odom/base 显示保持 -3.734°；本次变化改善了角度误差，但仍需录制文件核对时间关系。未打印观测警告。

录制目录 arrival_tf_20260921_142201：metadata 显示 SQLite3、256.398 s、47418 条消息，/scan 2547、/odom 12736、/cmd_vel_nav 2047、/cmd_vel 4197、/amcl_pose 408、/clock 2548、/tf 22934、/tf_static 1。录制器正常停止；元数据只能确认话题有数据和总时长，不能单独证明每段完整覆盖或无丢消息。终端日志 arrival_console_20260921_142219.log 已由用户提供。

新增 scripts/inspect_arrival_bag.py：只读单文件 SQLite bag，不创建 ROS 节点、不发布消息，输出话题收录范围、时钟范围、前两段停车样本及附近 AMCL/校正变化。接收时刻对应的最近 /clock 只是近似时刻，输出不把它当消息时间戳。脚本已在 Windows 通过 Python 语法检查，尚待用户在 Ubuntu 执行；原始 bag 仍在 Ubuntu。

### 用户完成离线 bag 分析后的证据

inspect_arrival_bag.py 已由用户在 Ubuntu 成功执行。bag 的 /clock 范围 1209.3～1464.0，停车样本 TF 范围 1283.756～1454.296，录制时间范围覆盖全部停车样本；这不是无丢消息证明。

第 2 段：AMCL 位姿时间戳 1320.532，对应新 map/odom 校正时间戳 1321.532，正好偏移 1.0 s；接收时最近 /clock 约 1320.5，早于该段完成样本 1320.856。+0.5 s（1321.356）map/base yaw=-7.196656°，odom/base=-3.733617°，差值 -3.463039°，等于旧校正；+1 s（1321.856）map/base=-3.990430°，odom/base=-3.733617°，差值约 -0.256813°，等于新校正 -0.256812°。校正变化 +3.206227° 与地图角度变化对应，odom 角度在输出精度内不变。此处 map/base 和 odom/base 各采样的时间戳相同，使用它们的平面 yaw 差与历史校正比对；不是把独立查询的最新 map/odom 与旧时刻直接相加。

第 1 段同样存在 1283.832 的 AMCL 位姿与 1284.832 的新 TF 校正；车停稳后地图角度逐步转至新校正对应值。当前输出只打印 TF 值的变化，未打印全部重复广播，不能仅用两个变化点断言具体插值区间。

因此这次 bag 支持：已经收到的 AMCL 校正因 TF 时间标记向未来偏移，稍后才体现在相应时间的 map/base 查询中。不是 AMCL 必须等车停后一秒才计算，也不是车体继续转动。之前未录 bag 的超限段与此机制一致，但不能将本次证据替代它的完整 TF 历史，更不能据此认定 AMCL 精度或全部恢复问题已解决。

下一项候选试验（尚未执行）：只把运行时 AMCL transform_tolerance 从 1.0 改为 0.2 s，保留到达余量配置、速度和原 +0.5 s 验收。Nav2 1.1.20 的动态参数回调更新 transform_tolerance_ 并重新初始化激光相关组件，区别于 RotateToGoal 初始化缓存的 XY 窗口。修改后需确认参数成功、保持仿真运行并让旧的未来 TF 时间段过去，再进行带 bag 的单轮对照，检查是否产生外推错误及停车后变化。0.2 s 仅是待验证候选，不保证足以覆盖所有延迟。失败时恢复 1.0；运行时修改未固化，完整重启也恢复原 localization.yaml 的 1.0。

## AMCL 0.2 s 运行时单轮结果（已执行）

用户已成功设置并读回 transform_tolerance=0.2。结果 roundtrip_20260921_143344_361840，日志 arrival_console_tol02_20260921_143344.log，录制 arrival_tf_tol02_20260921_143334。与前面的 1.0 s 数据分开统计：本配置目前 2/2 原验收通过；此前到达余量配置配合 1.0 s 时为 14/14 原验收通过，且曾有停车后角度超限。

| 目标 | 耗时 s | +0.5 s 位置 cm | +0.5 s 角度 ° | 终端打印的最大恢复计数 |
|---|---:|---:|---:|---:|
| B | 26.2 | 7.46 | 0.946 | 0 |
| A | 37.3 | 5.58 | 5.078 | 0 |

两段 +1、+2、+3 s 的误差均在阈值内，且与 +0.5 s 基本一致（以输出精度为限）。没有在提供的测试日志中看到观测警告；尚未检查完整导航启动日志，不能据此声称全程没有 TF 外推错误。动作完成不等于瞬时静止：回 A 的 +0 s 仍有 odom.wz=0.1234 rad/s，至 +0.5 s 已接近零。

bag 的 /clock 范围 1900.3～1986.3，停车样本 TF 范围 1939.216～1982.796，八个指定话题均有消息。范围检查通过，不代表不存在丢消息。

时间偏移已有直接录制证据：B 的 AMCL 位姿 stamp=1939.038，对应 map/odom 新校正 stamp=1939.238，差值 0.200 s；A 的对应值为 1979.339 与 1979.539，同样为 0.200 s。B 动作完成采样 stamp=1939.216，新校正稍后到达该查询时间，+0.5 s 已对应校正 yaw=0.508720°（map/base 0.945622° 减 odom/base 0.436902°）；回 A 时完成采样之前已有校正 yaw=0.489915°，后续角度变化主要与停车转动一致。这支持参数实际改变了发布 TF 的时间偏移，并在本次将相关校正体现于 +0.5 s 验收之前；不证明后续所有定位修正必然如此，也不证明精度提升具有统计稳定性。

下一步：保持同一运行时配置，带录制和终端日志做三轮验证。0.2 s 尚未固化到配置文件；后续需要完整重启复验及更长时间稳定性验证。

## AMCL 0.2 s 三轮结果及启动配置准备

结果 roundtrip_20260921_143743_389677；终端日志 arrival_console_tol02_20260921_143743.log；用户报告录制目录 arrival_tf_tol02_20260921_143731。本轮原验收 6/6 通过，六段 +1、+2、+3 s 的打印误差均与各自 +0.5 s 一致。0.2 s 运行时配置累计 8/8 原验收通过，所有已提供后续采样也在阈值内；只限这些离散采样及当前运行实例。

| 段 | 目标 | 耗时 s | +0.5 s 位置 cm | +0.5 s 角度 ° | 打印的最大恢复计数 |
|---|---|---:|---:|---:|---:|
| 1 | B | 81.0 | 6.41 | 4.617 | 2 |
| 2 | A | 33.2 | 5.90 | 4.535 | 0 |
| 3 | B | 35.2 | 8.98 | 0.791 | 0 |
| 4 | A | 31.8 | 11.81 | 0.266 | 0 |
| 5 | B | 28.6 | 10.53 | 3.951 | 0 |
| 6 | A | 32.0 | 6.21 | 4.900 | 0 |

首段在剩余距离约 3.29 m 处停留多次，恢复计数达到 2；途中停滞仍未解决，不能将本次停车采样稳定解释为导航效率问题已修复。本轮最大 +0.5 s 误差为 11.81 cm / 4.900°。控制条件 0.10 m 不等于动作结束后独立采样必然小于 0.10 m；原验收仍为 0.15 m / 0.15 rad。未从测试输出看到观测警告，但尚未审阅完整导航日志，不能断言没有 TF 外推错误。

Windows 已准备 config/amcl_tf_trial.yaml，仅含 AMCL transform_tolerance=0.2；localization.launch.py 新增可选 amcl_overrides，将指定参数文件置于原 localization.yaml 之后，navigation.launch.py 透传同名参数。省略参数时仍加载原 AMCL 1.0。两个 launch 通过 Python AST 语法检查，YAML 内容及默认值保持检查通过；尚未同步到 Ubuntu，也未验证从此文件启动。

下一步先停止旧的小车完整 launch 并确认进程退出，再同步 launch/navigation.launch.py、launch/localization.launch.py、config/amcl_tf_trial.yaml，编译并按既定顺序加载 TF2 环境。启动候选方案：

```bash
ros2 launch mobile_robot_sim navigation.launch.py \
  map:=$HOME/ros2_ws/maps/room_01.yaml \
  nav_overrides:=$HOME/ros2_ws/src/ros2_mobile_robot_sim/config/arrival_margin_trial.yaml \
  amcl_overrides:=$HOME/ros2_ws/src/ros2_mobile_robot_sim/config/amcl_tf_trial.yaml
```

必须重新核对参数、修复库加载，先到 A，再进行带日志的重启复验。只省略 amcl_overrides 即撤回到 AMCL 1.0 并保留到达余量试验；两个 overrides 都省略则恢复原项目配置。此处是待执行步骤，不代表已完成重启验证。

## 完整重启后的单轮复验（已完成）

用户已同步两个 launch 和 amcl_tf_trial.yaml，编译成功；完整重启时显式传入 nav_overrides 和 amcl_overrides，没有通过运行时 param set 补值。读回六项参数依次为 AMCL 0.2、GoalChecker XY/yaw 0.1/0.1、FollowPath XY 0.1、最低速度 0.03/0.12。controller_server PID=4872、planner_server PID=4876 的 /proc/PID/maps 均显示加载 tf2_fix_ws 的 libtf2.so、libtf2_ros.so，以及系统 Cyclone DDS 库。先到 A 的动作 SUCCEEDED、最后反馈恢复计数 0。

正式结果 roundtrip_20260921_145005_555014，日志 arrival_console_restart_20260921_145005.log，录制目录 arrival_tf_restart_20260921_144959。原验收 planned=2、attempted=2、action_succeeded=2、passed=2。

| 目标 | 耗时 s | +0.5 s 位置 cm | +0.5 s 角度 ° | 打印的最大恢复计数 |
|---|---:|---:|---:|---:|
| B | 25.5 | 9.67 | 1.941 | 0 |
| A | 31.1 | 7.01 | 4.654 | 0 |

两段 +1、+2、+3 s 的打印误差与 +0.5 s 一致，均在原阈值内；测试输出未见观测告警，但尚未独立审阅整个导航启动日志。录制已正常停止，尚未解析本次 bag。证据来自用户终端反馈，不是助手直接在虚拟机执行。

当前 AMCL 0.2 s 方案累计原验收 10/10，覆盖一次完整启动复验；不抹去之前首段 81 s、恢复计数 2 的事实。下一步保持当前配置，从 A 开始做 10 轮（20 段）连续测试并保留 finish-diagnostics、终端日志与 bag，新增录制 /rosout 以辅助核对恢复及 TF 错误。该连续测试尚未执行，仍按原 +0.5 s 阈值判定并单独报告后续采样。

## 10 轮连续测试（已完成）

用户执行 summarize_endurance.py 提供完整汇总。结果 roundtrip_20260921_145404_601930，日志 arrival_console_endurance_20260921_145404.log，bag arrival_tf_endurance_20260921_145355。planned=20、attempted=20、action_succeeded=20、passed=20、exit_code=0。

- +0.5、+1、+2、+3 s 各有 20 个样本，无缺失或重复段号，无超限；每组最大位置误差均为 11.448 cm、最大角度误差均为 5.9866°。这些最大值相同不等于所有样本完全相同，也不能证明采样间连续满足阈值。
- observation_warning / bad_observation 共 0。本轮支持未复现被脚本监测到的 TF/footprint 停更，不能推导永久修复。
- 各段耗时 25.26～62.98 s，均值约 34.70 s、中位数约 33.02 s；导航耗时合计约 694.04 s（由四舍五入的逐段耗时计算，不含所有测试间隔）。
- 第 5、7、17 段（均到 B）打印的最大恢复计数分别为 1、3、2，耗时分别为 62.98、26.75、35.88 s；其余段打印为 0。恢复计数是反馈观测，不是每种恢复行为的独立事件审计。
- bag 时钟 366.8～1142.2，约 775.4 s（12 分 55 秒），单调且覆盖全部停车采样；九个请求话题均有数据。范围覆盖不证明无丢消息。
- /rosout 共 794 条，筛选告警及相关错误 32 条、11 种消息。包含 1 次 Failed to make progress；4 次 NavFn potential 存在但路径提取失败及相伴的规划失败/中止；4 次 No valid trajectories out of 405；2 次 follow_path halt 时获取结果失败；8 次规划循环频率未达期望。不同消息可能描述同一次故障链，不能相加当成独立事故数。
- 该筛选输出没有 TF 外推错误。结论限于已录制的 /rosout 与当前筛选规则，不覆盖未发布到 /rosout 或丢失的日志。

停车精度阶段可记录为本次验收通过；下一阶段重点是导航过程可靠性和效率。先分析现有 bag 中的故障时间线及附近速度/位姿，区分进展检查、规划路径提取和局部轨迹不可用，保持所有配置不变，不立即追加测试。summarize_endurance.py 新增 --timeline，使用 rosbag 接收时间对应的最近 /clock 近似关联到段号和事件采样；不把日志的 Unix 时间直接当成仿真时间，也不宣称关联样本代表节点内部同步状态。此扩展仅通过 Windows 语法检查，待 Ubuntu 执行。

### 故障时间线（用户已执行 --timeline）

- 第 5 段，sim≈547.0：Failed to make progress，附近 map 位姿约 (0.7555,-0.1557,3.128 rad)。取到的是提前约 0.1 s 的样本：nav wz=-0.15789、cmd wz=-0.07895、odom wz=+0.06933 rad/s，线速度指令为零。单个异步样本中角速度符号相反不能证明驱动方向错误、摩擦问题或持续反向运动；需检查失败前整个时间窗。
- 第 7 段 sim≈633.1/633.3 与第 17 段 sim≈1005.2/1005.3：NavFn 路径提取失败，都在 map 约 (0.62～0.66,-0.08～-0.05) 的邻近区域；附近实际前进速度约 0.20 m/s，并非当时完全不动。两段随后均有 follow_path halt 获取结果失败日志。时间相邻支持把它作为同一流程的后续现象排查，尚未证明直接因果。记录未含当时完整 costmap、potential 或失败路径，不能据此确定 NavFn 内部失败原因。
- 第 14 段 sim≈885.1～885.3 和第 20 段 sim≈1108.4：DWB 无有效轨迹，位置约 (1.13～1.19,1.54～1.57)。第 14 段 cmd vx 从 0.20 降至 0.15、0.10 m/s，odom vx 从 0.212 降至 0.162、0.108 m/s，而 nav 输出已为零，支持减速过程，不等于指令失控。记录没有各 critic 的拒绝理由或局部代价图，不能断言是碰撞或某项权重导致。

先调查最慢第 5 段的进展失败：SimpleProgressChecker 1.1.20 按 XY 位移判断进展，原地旋转不重置其位移进展计时。项目门限仍为 0.1 m / 20 s；不通过延长超时掩盖停滞。新增 --motion-window 522 547，用已保存的 pose_sample 输出每秒一条位姿/速度并统计相对窗口首样本的位移；首样本不是控制器内部基准点。该扩展仅通过 Windows 语法检查，待用户在 Ubuntu 读取现有结果，无需运动或重录。

### 第 5 段 522～547 s 运动窗（用户已提供）

共 248 个 pose_sample，逐秒展示表明 522～527 s 仍在向前绕行，约 530 s 后前进很少、转向指令反复换符号。以展示的 530～546 s 为例，odom 从 (0.7213,-0.1446) 变为 (0.6951,-0.1470)，端点位移约 2.63 cm；地图端点位移约 1.42 cm。控制器转向输出多次达到约 0.16～0.60 rad/s，不是全程仅发过小角速度。实际朝向和角速度也发生往复变化，支持局部控制振荡；不能仅归因于摩擦或驱动反向。

543～546 s 附近 yaw 跨越 ±180°，数值跳变不是实际转了约 360°。报告末尾相对 522 s 的约 0.72 m 位移包含了前段正常行驶，不能证明后面的 20 s 进展检查错误。控制器与平滑后命令短时异号可能与换向过程和异步采样相关，不能据此认定 velocity_smoother 本身就是根因。

下一步只读核对 FollowPath.critics 和 Oscillation 参数。Nav2 1.1.20 OscillationCritic 在初始化读取距离、角度、时间重置门限；需区分已加载、被重置、轨迹选择等可能机制，不能仅看到 critic 在列表中就认定它应该彻底消除振荡。暂不提高最低速度、不延长进展超时，不调整既有 AMCL/到达参数。

### 振荡参数与专项诊断录制

用户核对 Oscillation 在 critics 列表中，scale=1.0、reset_dist=0.05、reset_angle=0.2、reset_time=-1.0、x_only_threshold=0.05；publish_evaluation 与 publish_global_plan 均为 True，诊断话题存在。Nav2 1.1.20 DWB setPlan 会 reset 所有 critics；角度/距离门限也是 Oscillation 自身的重置途径，实际触发原因尚未确定。

保持运动参数不变，用户完成一轮专项诊断：roundtrip_20260921_153249_190212；日志 dwb_console_20260921_153249.log；bag dwb_diagnostic_20260921_153241。B：29.1 s、+0.5 s 9.40 cm/4.586°；A：39.4 s、0.50 cm/5.119°。两段原验收通过，+1～3 s 打印误差与各自 +0.5 s 一致，打印恢复计数均为零，未复现进展失败。当前方案累计原验收 32/32，但不证明振荡消失。到 A 的 0.50 cm 是地图定位估计下误差，不是真值定位精度。

bag 元数据约 90.423 s、19514 条消息，包含 /evaluation 654 条、/received_global_plan 754 条、/transformed_global_plan 685 条、局部/全局 costmap_raw 150/47 条和 /rosout 97 条。数量只证明数据存在，不保证无丢失或全部同步。收到路径数量较多值得检查，不能直接据此认定全部为不同新路径或推导故障根因。

新增 scripts/inspect_dwb_bag.py：只读单文件 SQLite，按已保存 raw_plan/finish_sample 时间窗汇总每段路径接收间隔、忽略头时间戳的重复几何、候选轨迹首个负分 critic、远离目标时低前进速度下的转向换符号，以及获选轨迹各项加权评分。非负候选分数可能已被 short-circuit 提前截断，不能直接用它们对全部候选完整排序。轨迹样本与最近路径接收时刻只作离线关联，不当成内部状态插桩。已通过 Windows Python 语法检查，尚待用户在 Ubuntu 执行。

### 首次评分分析及路径话题解释修正

用户已成功运行 inspect_dwb_bag.py。第 1 段评分 272 条、路径消息 321 条，相邻几何相同 294 次，唯一 header stamp 29 个；第 2 段评分 382 条、路径消息 433 条，相邻几何相同 406 次，唯一 header stamp 39 个。路径消息接收间隔中位数均为 0.1 s。

必须澄清：Nav2 1.1.20 dwb_local_planner.cpp 的 transformGlobalPlan 在 prune_plan 分支也调用 publishGlobalPlan(global_plan_)，因此 /received_global_plan 不只代表 setPlan。0.1 s 发布频率及评分前约 3 ms 的该话题消息，不是 critic 状态重置频率或重置时间的证据；不能据此认定每个控制周期都 reset Oscillation。原先把此话题简称“控制器收到的新路径”的说法不够准确。setPlan 确实 reset critics，但需结合 controller_server 的 Passing new path to controller 日志或内部插桩区分触发点。

第 2 段记录到远离目标、低 vx 条件下 14 次转向换符号，约 2724～2731 s 多次 vx=0、wz 在 ±0.1579 等值间往复，目标距离保持约 2.997 m。这次虽无恢复或进展失败，仍复现了较短时间的振荡。多个展示周期有 357 个前进候选得到非负记录分数、未显示硬拒绝；但因短路评分，不可声称全部候选完成合法性验证。不能将此现象简单解释为所有前进方向均被障碍否决。获选轨迹常由 GoalAlign/GoalDist 贡献较多分值，这不足以直接决定修改哪个权重，需比较竞争前进候选。

脚本已补充控制器路径更新 INFO 日志的接收时序、获选总分及前进候选的最小已记录分数/分项。前进候选记录值仍可能只是提前截断的下界，不是完整最优前进轨迹的证明。此次补充通过 Windows 语法检查，尚待 Ubuntu 执行；不改变仿真配置、不需要重录。

### 控制器更新日志与分数对比（用户已执行补充分析）

第 2 段真实控制器 Passing new path 日志约每 1 s 一次：2725.3、2726.3、2727.2、2728.3 等。2725.3、2726.3、2727.2、2730.3 的负向转动样本在相应日志接收后约 3～5 ms；其间 0.3～0.7 s 后又出现正向转动。结合 setPlan 重置 critics 的源码，可把周期重规划重置振荡状态作为有证据支持的干预假设；仍未直接记录 critic 内部标志，不能排除角度重置和评分地形的作用，不能把所有振荡归于单一原因。

典型周期 2725.3：获选原地转动总分 69.2，前进候选最低已记录分数 71.2，差值来自 GoalAlign +0.6、PathAlign +0.8、GoalDist +0.6。其所列七项 critic 均出现；其它未完整评分候选仍不能被离线精确重排。2724.2 的获选分数 69.4、前进最低记录 72.4，BaseObstacle 都为零。说明这些展示周期的选择并非所有前进轨迹都被障碍硬拒绝，而是当前评分更偏向旋转；也不代表强行提高前进速度是合适修复。单独缩小路径对齐权重不足以保证所有周期前进获选。

下一项拟议对照：在实际使用的行为树副本里，仅把周期重规划 RateController 从 1 Hz 改为 0.2 Hz（约 5 s），保留恢复分支、DWB/AMCL/速度/进展和验收条件。仅用于当前静态场景诊断，较慢更新可能降低路径调整及时性，不作为默认长期方案。先核对 Ubuntu 当前 default_nav_to_pose_bt_xml 和实际 XML，再准备独立副本并通过单目标 behavior_tree 字段指定；避免改动 /opt/ros/humble 或用全局默认替换污染基线。尚未取得实际 XML、未创建副本、未修改重规划频率、未运行对照。

### 0.2 Hz 行为树试验准备（Windows 已完成，Ubuntu 待执行）

用户已确认 default_nav_to_pose_bt_xml 指向系统 navigate_to_pose_w_replanning_and_recovery.xml，并提供完整 XML；其 RateController hz=1.0，恢复分支与官方 Humble 默认树一致。

新增 config/replanning_02hz_trial.xml，依据该 XML 复制，可执行树唯一修改为 hz=0.2。repeat_navigation.py 新增 --behavior-tree：在发送导航前读取并检查 XML 语法，将绝对路径写入每个 NavigateToPose.Goal.behavior_tree；原先独立的规划预检和 +0.5 s 验收不变。test_start 事件记录行为树路径，启动时也打印路径。不传此选项仍使用 Nav2 默认行为树；不覆盖系统文件或默认 BT 参数。XML 语法有效不代表所有 BT 插件运行时均可加载，需实际验证。

Windows 验证：Python AST/XML 检查通过；--help 显示新选项；已有 13 项离线回归测试全部通过。这些是模拟接口测试，不是 Ubuntu 仿真验证。需要同步脚本与 XML，直接运行源脚本无需重新编译或重启导航。下一次以相同诊断话题记录单轮对照，比较真实路径更新间隔、低速换向、耗时、恢复及原停车验收。初次加载、失败恢复时路径更新可不遵循固定 5 s 间隔，不能把 0.2 Hz 理解为所有情形下一律只每 5 s 更新一次。

### 0.2 Hz 单轮对照（用户已执行）

结果 roundtrip_20260921_192737_738477，日志 dwb_console_replan02_20260921_192737.log，bag dwb_replan02_20260921_192732。启动记录明确指定 replanning_02hz_trial.xml；controller_server 路径更新日志通常间隔约 5 s，证明周期修改生效。该方案单独统计原验收 2/2；不要并入默认 1 Hz、AMCL 0.2 s 方案的 32/32。

| 指标 | 前次 1 Hz 诊断单轮 | 本次 0.2 Hz 单轮 |
|---|---:|---:|
| 到 B 耗时 s | 29.1 | 32.2 |
| 回 A 耗时 s | 39.4 | 36.1 |
| 合计耗时 s | 68.5 | 68.3 |
| 到 B / 回 A 低 vx 换向数 | 1 / 14 | 0 / 1 |
| 打印的恢复计数 B / A | 0 / 0 | 0 / 1 |
| 评分中无有效获选轨迹周期 B / A | 0 / 0 | 0 / 4 |

本次 B +0.5 s 为 7.13 cm/4.618°，A 为 5.87 cm/4.485°；两段 +1～3 s 打印值与 +0.5 s 一致。回 A 出现四次 No valid trajectories out of 405、一次 Controller patience exceeded、随后 follow_path 中止与新的控制目标日志，并最终成功。不能将换向减少直接称为整体修复或效率改善；单轮、不同起点估计及规划路径长度，因果结论有限。暂不推广 0.2 Hz 到默认配置。

分析脚本原先在无有效获选轨迹时提前跳过候选拒绝统计，因此此前输出的 critic 累计数不包含这四个失败周期；no_valid_best=4 本身仍有效。现已修正为统计所有周期，并增加 INVALID_EVAL/INVALID_MOTION，专门读取这四帧的拒绝 critic 与附近运动记录。Python 语法检查通过，尚待 Ubuntu 执行。仍只分析已有 bag，不改运动配置、不重录。

### 2026-09-21: 0.2 Hz trial obstacle rejection follow-up

User-provided inspection of `dwb_replan02_20260921_192732` shows four all-invalid evaluations at simulation times 16779.1–16779.4. Each has 405 candidates rejected by BaseObstacle (357 forward candidates). Recorded odometry vx decreases from 0.16777 to 0.01320 m/s across these samples; this establishes braking during the rejected interval, not physical collision or the exact offending grid cell. Trial 2 subsequently succeeds after recovery. Do not promote this trial to the default based on fewer angular sign changes alone.

Added read-only `scripts/inspect_obstacle_rejections.py` to compare all-invalid trajectory samples against the recorded local raw costmap immediately before and after each evaluation by bag receipt time. It reports frame compatibility, snapshot times, first stored pose costs and first forbidden trajectory indices. Costs 253/254/255 and off-grid positions are rejected according to Nav2 1.1.20 BaseObstacle. Snapshot comparisons cannot reproduce the exact internal controller costmap or prove physical contact. Local syntax and synthetic cell-boundary checks passed; actual Ubuntu bag analysis remains pending.

### 2026-09-22: Recorded costmaps reproduce rejection during predicted motion

User ran inspect_obstacle_rejections.py against dwb_replan02_20260921_192732: 142 local raw costmap snapshots and four all-negative evaluations. For every evaluation at 16779.1–16779.4, the preceding snapshot places all 405 trajectory start poses at cost 225, and every candidate first reaches cost 253 at an index greater than zero. For the first two evaluations, the following snapshot reproduces the same complete rejection. No off-grid, unknown or lethal-cell first rejection appears in these comparisons. This supports predicted entry into the inscribed inflated zone, rather than a forbidden start pose, for this recorded episode.

The following 16779.8 snapshot instead leaves 99/405 and 115/405 trajectories without a BaseObstacle rejection for evaluations 16779.3 and 16779.4 respectively. These counts do not establish validity under the other critics. They also cannot be backdated to the evaluation time. metadata.update_time is zero in the supplied snapshots, so use displayed header and receipt times without treating zero as a real update timestamp.

Zero target vx candidates still contain predicted translational motion. The Humble 1.1.20 StandardTrajectoryGenerator projects the starting velocity toward the candidate target using acceleration/deceleration limits; verify the live generator and odometry smoothing duration before attributing the translational remainder to that mechanism. No tuning or collision-radius reduction applied. Next read-only check: live controller generator, odometry duration, trajectory horizon/deceleration and obstacle/path scoring parameters.

### 2026-09-22: Prepare obstacle-score-only trial (not run)

Live readback: StandardTrajectoryGenerator, sim_time 1.7, decel_lim_x -0.5, BaseObstacle.scale 0.02, PathAlign/PathDist.scale 32, GoalDist.scale 24. odom_duration is unset. Nav2 1.1.20 controller uses nav_2d_utils::OdomSubscriber, whose callback stores the latest odometry twist directly; do not infer a rolling average or add odom_duration to this controller. The preceding query was not applicable to this version's controller implementation.

Prepared config/obstacle_margin_trial.yaml with the existing arrival trial's five parameters preserved and only BaseObstacle.scale=0.05 added. This exploratory value raises the preference for lower obstacle costs among legal trajectories; it cannot make all-rejected trajectories legal and is not an established fix. No radius, inflation, acceleration, AMCL, acceptance or default configuration change. YAML parse and exact comparison against arrival_margin_trial.yaml passed locally. Runtime experiment is pending. Restart to initialize critic scale, retain AMCL 0.2 and the same per-goal 0.2 Hz BT for comparison. Compare success, recoveries/all-invalid evaluations, transit times and post-finish error; do not promote on a single successful round. Roll back using arrival_margin_trial.yaml at launch.

### 2026-09-22: First obstacle scale 0.05 run completed

User supplied full console and inspector output for roundtrip_20260922_144117_495838, bag dwb_obstacle005_20260922_144101, console dwb_console_obstacle005_20260922_144117.log. Per-goal BT remains replanning_02hz_trial.xml; live scale previously verified 0.05 and AMCL transform tolerance 0.2.

Both goals passed. B: 26.5 s, 9.37 cm / 4.655 deg at +0.5 s, one printed recovery. A: 33.9 s, 8.20 cm / 5.222 deg, zero printed recoveries. Both remain unchanged at +1, +2, +3 s. Total navigation wall time is 60.4 s (prior 0.02/0.2 Hz trial 68.3 s); separate starts and stochastic localization mean this is not a controlled proof of speed improvement.

Recorded evaluation counts 259 and 333; no_valid_best=0 for both and the inspector's low-vx sign-flip metric is zero for both. A has 218 BaseObstacle candidate rejections; candidate rejections alone are expected filtering and not controller failure. Recorded warning/error summary has one NavFn potential-to-path extraction failure, accompanying GridBased/action warnings, and one missed planner rate warning. No controller warning/error was listed. This is consistent with the B recovery being associated with the planner, but the aggregate warning report is not an exact timestamp association.

Conclusion: the previously observed four all-invalid DWB cycles did not recur in this one round. Planning failure remains and this is not a fault-free run. Keep scale 0.05 as an opt-in trial, do not promote default or change additional parameters. Next bounded validation: three more B/A rounds with same AMCL, scale and per-goal BT, recording evaluation/plans/costmaps/rosout. No new A positioning goal is needed if robot remains where this successful run ended.

### 2026-09-22: Three-round obstacle scale 0.05 validation completed

Results roundtrip_20260922_144703_123403; bag dwb_obstacle005_20260922_144653; console dwb_console_obstacle005_20260922_144703.log. Same per-goal 0.2 Hz BT and AMCL transform tolerance 0.2. All six goals passed; printed recovery feedback is zero for all six. Recorded ROSOUT warning/error dictionary is empty. All six inspector windows have no_valid_best=0 and low_vx_turn_sign_flips_away_from_goal=0. Evaluation counts: 313,322,295,329,292,331 (1882 total). These statements concern recorded messages and the inspector's defined windows/metric, not guaranteed lossless coverage or absence of every physical oscillation.

Per-trial +0.5 s xy cm / yaw deg / wall s:
1 B: 6.07 / 4.539 / 32.5
2 A: 5.58 / 4.325 / 33.8
3 B: 9.19 / 5.604 / 30.7
4 A: 6.90 / 5.777 / 35.1
5 B: 6.66 / 5.148 / 31.9
6 A: 0.93 / 4.361 / 35.3
Total rounded wall time 199.3 s; mean 33.22 s. +0.5 through +3 s samples stay within original 0.15 m / 0.15 rad acceptance; max displayed xy 9.19 cm and yaw 5.777 deg. Last trial xy changes only from displayed 0.93 to 0.92 cm at +3 s. Return-leg BaseObstacle candidate rejections 177,243,540 are individual filtering, not all-invalid cycles.

Combined with preceding two-goal trial: 8/8 accepted, zero recorded all-invalid DWB cycles across 2474 evaluations, one earlier planner extraction failure/recovery; do not describe all eight as recovery-free. Keep obstacle_margin_trial.yaml + per-goal replanning_02hz_trial.xml as the current candidate combination, without promoting defaults or claiming isolated causal proof from unequal runs. Three-round validation is complete; further promotion should use a longer fixed-configuration run and a cold-start check, with the unresolved intermittent NavFn failure tracked separately.

### 2026-09-22: Endurance trial 11 failed arrival acceptance

User evidence: roundtrip_20260922_145648_052093, bag dwb_obstacle005_endurance_20260922_145640. Planned 20, attempted 11, action_succeeded 11, passed 10. Failure at B: +0.0 s error 0.1503643 m, +0.5 s 0.1513634 m, then stable to +3 s. Thus already outside 0.15 m on receipt of success; stopping adds only about 1 mm to radial error. Odom TF position displacement from +0 to +0.5 is about 1.72 mm. Sampled map/odom correction values remain identical across all five finish observations. No basis to attribute the main error to a post-result localization jump or large coast.

Global footprint center at stamp 1448.795 was about 9.28 cm from B, whereas map/base at 1449.695 is 15.04 cm from B. This is a 0.9 s asynchronous comparison and only motivates inspection of preceding TF/localization/motion; it is not proof of a localization jump. Trial 8 also has +0.5 yaw 8.40553 deg, close to the unchanged 8.594 deg acceptance limit.

Added --trials and --before options to inspect_arrival_bag.py (legacy defaults 1 2 / 2 s preserved) to inspect trials 8 and 11 over a 6 s pre-result interval. Syntax compilation passed. Actual bag run pending on Ubuntu. SimpleGoalChecker 1.1.20 can latch XY satisfaction with stateful=true; runtime plugin/stateful and TF chronology must be checked before attributing the success/error discrepancy to it. No change to acceptance, trial config or defaults.

### 2026-09-22: Arrival correction timing and outstanding goal-transform hypothesis

Live goal checker confirmed nav2_controller::SimpleGoalChecker, stateful=true. Trial 11 AMCL samples move from (1.408038,1.993514) at 1448.452 (~9.22 cm from B) to (1.387814,1.904748) at 1449.352 (~14.72 cm); these are different times and do not alone isolate physical movement. map/odom yaw correction changes 0.926747 to 3.982958 deg, with final changed TF stamped 1449.552, before finish 1449.695. Trial 8 correction changes 0.346193 to -2.934441 deg at TF stamp 1343.85, before finish 1344.255; final map yaw -8.651774 deg while odom yaw -5.717333 deg. XY latching alone cannot explain the yaw discrepancy relative to internal 0.1 rad tolerance.

Nav2 1.1.20 controller setPlannerPath copies the last stamped path pose to end_pose_ and isGoalReached transforms it to the local costmap frame. Whether a historical goal stamp contributes must be checked against recorded path data, without claiming internal callback-state reconstruction. RotateToGoal also latches its in-window/rotating states; simply setting goal-checker stateful=false is not a demonstrated complete fix. Extended inspect_arrival_bag.py with timestamped received/transformed path endpoint reports, throttled to ~0.5 s or a new header stamp. Reports explicitly warn that published conversion can rewrite stamps and transformed paths may be cropped. Syntax compilation passed; runtime evidence pending. No parameters changed.

### 2026-09-22: DWB endpoint updates confirmed; prepare goal-check repair trial

Latest supplied PLAN_END data disproves an unchanged DWB endpoint: trial 8 transformed end yaw becomes +2.934441 deg at receipt clock 1343.7, while received map path retains stamp 1341.2. Trial 11 transformed goal becomes (1.511643,2.129607,-3.982958 deg) at 1449.3, while received map path retains stamp 1445.8. Published endpoint stamps can be rewritten, so these are not direct observations of controller goal-checker inputs. Together with upstream 1.1.20 source they support separate goal-check timestamp handling as a repair target; stateful XY checking is also confirmed.

Prepared scripts/patch_goal_check_time.py to patch an isolated Nav2 1.1.20 checkout: copy end_pose_, set its timestamp to current robot pose timestamp before transformation, and return false on failed transform. Never modifies /opt/ros. Prepared config/goal_check_trial.yaml identical to obstacle trial except stateful=false. This is a coherent arrival-check correctness trial, not a single-scalar tuning comparison. RotateToGoal's rotation state still resets on new path; possible delay/stall pending replanning remains a runtime concern. No runtime change made, no guarantee the endurance failure is resolved. Local patch syntax, match against exact upstream source and YAML single-delta check passed; C++ compile and ROS tests remain pending on Ubuntu.

### 2026-09-22: Goal-time repair first runtime round passed

User confirmed build and upstream test summary: 70 tests, 0 errors, 0 failures, 16 skipped. Actual controller process maps show nav2_goal_fix_ws/build/nav2_controller/libcontroller_server_core.so plus tf2_fix_ws tf2/tf2_ros. Live stateful=False, BaseObstacle.scale=0.05, AMCL transform_tolerance=0.2.

First runtime result roundtrip_20260922_152900_348061, bag dwb_goalfix_20260922_152852, same per-goal 0.2 Hz BT. B: 25.6 s, 9.35 cm, 1.985 deg. A: 33.6 s, 5.45 cm, 4.289 deg. Both pass with printed recoveries zero, and +0.5/1/2/3 s finish errors stable at displayed precision. Recorded evaluations 250+322=572, all have valid best candidates; low-vx sign-flip metric zero; recorded ROSOUT warnings/errors empty. 177 BaseObstacle rejected candidates on return do not constitute controller failure. No timeout in either goal. This first round does not demonstrate behavior during a repeat of the earlier localization correction; do not claim endurance issue fixed yet.

Next validation: fixed-configuration 10-round/20-goal endurance run, recording /plan in addition to prior DWB and TF topics. Retain original external 0.15 m / 0.15 rad and +0.5 s criterion. Cold-start validation still pending after endurance. No default configuration promotion.

### 2026-09-22: Goal-fix endurance acceptance summary confirmed

Result roundtrip_20260922_153415_582412; bag dwb_goalfix_endurance_20260922_153402; matching console dwb_console_goalfix_endurance_20260922_153415.log. 20 planned/attempted/action-succeeded/passed, all_passed=true. Each +0.5/+1/+2/+3 delay has all 20 samples without missing/duplicate trials. Max xy 9.4111 cm, max yaw 5.7754 deg, no observation warnings. Original 15 cm / 0.15 rad criterion retained. Clock 439.5–1177.6 is monotonic and covers finish samples; not a no-loss claim.

Printed recovery count is 1 on trial 7 (B), zero elsewhere. ROSOUT includes one NavFn potential-to-path extraction failure with its associated abort/warning messages and one planner-rate warning. Thus successful with recovery, not fault-free. User's pasted output ends at summarize_endurance; DWB inspector summary is not supplied yet. Bag contains 6236 evaluation messages, but do not infer zero invalid evaluations from message count or successful goals.

Candidate goal-time patch + stateful=false has 22/22 accepted across first short run and this endurance run. Default config remains unmodified, intermittent global planner issue remains tracked separately. Prepared scripts/start_goal_check_trial.sh with explicit environment sourcing, required-file checks, controller prefix check, and trial YAML paths. Uses LF bytes, verified locally; Ubuntu bash syntax/runtime check pending. It intentionally leaves default BT unchanged; repeated navigation still specifies replanning_02hz_trial.xml per goal. Next: finish missing DWB summary, then restart navigation/simulator and one recorded B/A round to verify startup reproducibility.

### 2026-09-22: Endurance DWB analysis completed

User supplied completed inspector output: 20420 selected messages parsed in 374.8 s. All 20 trial windows report no_valid_best=0. Low-vx turn-sign metric counts one change each in trials 2 and 10, zero in other trials; do not call this zero oscillation or a controller failure. ROSOUT warning/error summary contains only the previously recorded NavFn failure chain and planner-rate warning. Together with prior acceptance summary, candidate passes the 20-goal endurance acceptance with one observed recovery. No additional tuning warranted from these results. Next step is navigation/simulator process restart and one recorded B/A round using the explicit launcher; this is process-restart validation, not an OS/VM reboot test.
