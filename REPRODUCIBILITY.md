# 复现条件与已知缺口

本更新公开导航配置、试验脚本、分析工具和已记录的结果，不把源码下载等同于完整环境复现。

## 本仓库已有

- ROS 包：ros2_mobile_robot_sim，CMake 包名 mobile_robot_sim。
- 基础模型、世界、雷达、建图、定位和导航 launch。
- 最终组合：config/navfn_astar_trial.yaml、config/amcl_update002_trial.yaml、config/replanning_02hz_trial.xml。
- Nav2 1.1.20 控制器补丁工具：scripts/patch_goal_check_time.py；只针对匹配的上游源码，不修改系统二进制。
- 往返验收脚本、离线分析脚本、阶段结果索引、演示视频。

## 仍依赖已配置的 Ubuntu 环境

- ~/tf2_fix_ws：实际使用的 TF2 修复工作区，其源码不在本仓库中。
- ~/nav2_goal_fix_ws：使用 Nav2 1.1.20 应用目标时间戳补丁后构建的控制器。
- ~/ros2_ws/maps/room_01.yaml 及对应地图图像：实际验收地图不在本仓库中；按 MAPPING.md 重新建图得到的是新地图，不能等同于原验收环境。
- CycloneDDS 运行库，以及 ROS 2 Humble、Gazebo Classic 11、Nav2 和建图依赖。
- 原始 rosbag 与结果日志保留在用户虚拟机，本仓库中的汇总不是原始数据的替代品。

用户已在 Ubuntu 保存 robot_sources_20260923_160045.tar.gz，内容检查确认上述源码目录和地图包含实际文件，未发现软链接；但该备份未导入本仓库，也未做全新环境重建测试。

## 路径适配

历史说明中的 /mnt/hgfs/robot_project 是开发时的 VMware 共享目录。
Git 克隆到 ~/ros2_ws/src 后，包目录为：

```bash
project_dir="$HOME/ros2_ws/src/ros2_mobile_robot/ros2_mobile_robot_sim"
```

基础构建成功不代表两个修复工作区存在。已有修复工作区和地图的环境可使用：

```bash
bash "$project_dir/scripts/start_amcl_update002_trial.sh"
```

启动脚本根据自身位置寻找 config，默认地图为 ~/ros2_ws/maps/room_01.yaml。
它检查固定的三个工作区环境；自定义目录需先审查脚本。它不会更改默认行为树。
新终端需要先加载 ROS 和三个工作区并设置 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp。
往返测试必须显式传入：

```bash
python3 "$project_dir/scripts/repeat_navigation.py" --rounds 1 --finish-diagnostics \
  --behavior-tree "$project_dir/config/replanning_02hz_trial.xml"
```

测试会移动机器人，要求上一任务已结束、没有并发遥控或导航，且机器人在 A 点 0.3 m 内。
这些是适配仓库目录的命令说明，尚未在全新克隆环境执行验证。现有仿真验证证据见 AMCL_UPDATE_TRIAL.md。
