# 到点修复组合：使用与验收记录

截至 2026-09-22，本组合在当前四轮小车仿真场景累计 24/24 段通过原验收。用于后续工作的已验证组合，不代表任意地图、实车或无限时长可靠性保证。机械臂项目不在本记录范围。

## 固定启动

先结束上一套仿真与导航进程，再在 Ubuntu 执行：

```bash
bash /mnt/hgfs/robot_project/scripts/start_goal_check_trial.sh
```

脚本显式加载 Humble、ros2_ws、tf2_fix_ws、nav2_goal_fix_ws，使用 room_01 地图、goal_check_trial.yaml 与 amcl_tf_trial.yaml。依赖 VMware 共享目录已挂载。不会修改 /opt/ros、系统默认参数或默认行为树。

新测试终端需要加载环境：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/tf2_fix_ws/install/local_setup.bash
source ~/nav2_goal_fix_ws/install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

确认机器人停在 A=(1,-1) 附近且无其他运动命令后，运行：

```bash
python3 -u ~/ros2_ws/src/ros2_mobile_robot_sim/scripts/repeat_navigation.py \
  --rounds 1 --finish-diagnostics \
  --behavior-tree "$HOME/ros2_ws/src/ros2_mobile_robot_sim/config/replanning_02hz_trial.xml"
```

每目标行为树参数不可省略：启动脚本本身不将默认重规划频率改为 0.2 Hz。RViz 或未指定该 XML 的其他导航目标，不自动属于同一测试组合。

## 修复和参数

- TF2 使用独立 tf2_fix_ws 修复库；DDS 为 Cyclone。
- Nav2 1.1.20 控制器独立源码覆盖层：目标按当前机器人位姿时间进行坐标变换，变换失败不宣布到点。
- general_goal_checker.stateful=false；内部位置/角度容差为 0.10 m / 0.10 rad。
- FollowPath.xy_goal_tolerance=0.10，min_speed_xy=0.03，min_speed_theta=0.12，BaseObstacle.scale=0.05。
- AMCL transform_tolerance=0.2；每目标 replanning_02hz_trial.xml。
- 外部验收不变：导航成功且 +0.5 s 位置误差不超过 0.15 m、角度误差不超过 0.15 rad；另观察至 +3 s。

## 证据

| 测试 | 结果目录尾部 | 通过 | 最大 +0.5 s 位置/角度误差 |
|---|---|---|---|
| 首轮短测 | 20260922_152900_348061 | 2/2 | 9.35 cm / 4.289°（打印精度） |
| 十轮连续 | 20260922_153415_582412 | 20/20 | 9.4111 cm / 5.7754° |
| 进程重启复验 | 20260922_200711_777372 | 2/2 | 8.7539 cm / 4.4864° |

完整结果位于 ~/ros2_ws/test_results/roundtrip_<目录尾部>。重启复验对应 bag 为 dwb_goalfix_restart_20260922_200706，console 为 dwb_console_goalfix_restart_20260922_200711.log。正确尾号是 777372，早先截图误读的 773772 不存在。

长测与重启复验在 +0.5、1、2、3 s 采样完整、均在原标准内，观测警告为零。长测各段 no_valid_best=0；第 2、10 段各一次低速角速度符号变化。重启复验两段 no_valid_best=0，第二段一次该指标变化；打印恢复为零，录制 ROSOUT 警告/错误为空。该指标不等于物理抖动次数；记录覆盖不保证零丢包。重启包有 578 条 evaluation，分段统计为 248+329=577，故不能将“窗口内零失败”扩大到逐条无遗漏审计。

编译测试用户报告：70 tests，0 errors，0 failures，16 skipped。进程 maps 已验证加载修复控制器及 TF2 库。本次重启是仿真与导航进程重启，不是虚拟机/系统冷启动。

## 剩余问题与回退

长测第 7 段打印一次恢复；记录有一次 NavFn 势场路径提取失败及关联警告，最终导航成功。作为独立规划问题保留，不继续通过到点容差放宽掩盖。未证明任意定位突变均能恢复，RotateToGoal 内部状态和后续路径更新仍可能影响修正过程。

阶段验收完成，停止无依据的重复调参。保留 Ubuntu 内原始 bag、日志、结果与两个修复工作区。

若需对照旧版本：停止进程，在全新终端仅加载 Humble、ros2_ws、tf2_fix_ws，省略 nav2_goal_fix_ws，并用 obstacle_margin_trial.yaml 启动。不要在已经加载修复覆盖层的终端尝试仅再次 source 基础环境来回退。旧组合有已知停车超限记录，不作为本次已验证组合。
