# AMCL motion update trial: 24/24 stage validation (2026-09-23)

The A* short and endurance runs passed 22/22. A* process-restart run
roundtrip_20260923_144445_101130 failed the first B goal after action success.
Bag: dwb_astar_restart_20260923_144440.

At finish stamp 169.259, map error was 7.23 cm. At +0.5 s it was 16.026 cm.
Between those samples, map displacement was 8.961 cm and odom displacement
was 0.164 cm. Applying the final recorded map-to-odom correction to the
finish odom pose gives approximately 15.890 cm goal error. AMCL stamp
169.223 generated correction stamp 169.423; receipt-clock associations are
approximate and do not establish the controller's internal receipt order.

Confirmed runtime: SimpleGoalChecker, stateful false, XY/yaw tolerance 0.1,
A* true, AMCL transform_tolerance 0.2, patched controller and TF2 libraries.
AMCL update_min_d and update_min_a were both 0.1, resample_interval 1;
controller_frequency was 10.0 Hz.

Independent candidate: config/amcl_update002_trial.yaml and
scripts/start_amcl_update002_trial.sh. Reduce motion update thresholds to
0.02 m / 0.02 rad while retaining transform_tolerance 0.2, resampling 1,
config/navfn_astar_trial.yaml, and existing source overlays. Continue the
same per-goal replanning_02hz_trial.xml and external 0.15 m / 0.15 rad
acceptance. Hypothesis: more frequent motion-triggered updates may reduce
late correction size. This does not guarantee correction size or implement
pre-success stability confirmation. Runtime evidence is recorded below; Python
YAML structure and LF launcher encoding were checked on Windows only.

Start with one recorded round trip after returning to A. Compare correction
jumps, finish error, planning/control failures and timing warnings before
considering endurance testing. Roll back by restarting with
scripts/start_navfn_astar_trial.sh. Earlier 22/22 passes do not make the
failed A* restart test a pass, nor count as validation of this new trial.

## Completed validation

All runs used config/navfn_astar_trial.yaml, config/amcl_update002_trial.yaml,
the goal-time controller and TF2 overlays, CycloneDDS, and explicit per-goal
config/replanning_02hz_trial.xml. External acceptance was unchanged:
action SUCCEEDED plus <=0.15 m and <=0.15 rad at +0.5 s. Additional
+1, +2, +3 s samples also passed for every trial.

| Run | Results directory | Bag directory | Passed | Max post-finish XY cm | Max yaw deg |
|---|---|---|---|---|---|
| Short | roundtrip_20260923_150954_147598 | dwb_amcl002_20260923_150947 | 2/2 | 9.1497 | 4.7466 |
| Endurance | roundtrip_20260923_151955_783257 | dwb_amcl002_endurance_20260923_151948 | 20/20 | 9.3917 | 5.4228 |
| Process restart | roundtrip_20260923_153816_625773 | dwb_amcl002_restart_20260923_153810 | 2/2 | 9.5085 | 4.5223 |

All paths above are under ~/ros2_ws/test_results in the Ubuntu VM.
Console prefixes match dwb_console_amcl002_, dwb_console_amcl002_endurance_,
and dwb_console_amcl002_restart_, respectively, with each results timestamp.
All sampled recovery counts were zero; summary log filters found no selected
warnings/errors in these runs. No missing/duplicate post-finish samples or
observation warnings were reported. Endurance's largest printed XY error
increase from +0 to +0.5 s was 1.60 cm (trial 13: 7.77 to 9.37 cm), not a
measurement of physical displacement. Restart changes were 9.32 to 9.51 cm
at B and 6.57 to 6.67 cm at A.

Restart DWB analysis covered 251 + 309 = 560 recorded evaluations:
no_valid_best=0 in both windows, low-speed turn sign flip counts=0 in both,
and ROSOUT warnings/errors={}. Full DWB evaluation decoding has NOT been
provided for the new AMCL short or endurance runs; do not extend the restart
zero-invalid result to all 24 goals. Older A* DWB results used different AMCL
thresholds and are separate evidence.

This establishes a room A/B simulation baseline, not a guarantee of future
success, other maps, hardware accuracy or physical stopped-at-success status.
Restart means navigation/simulation processes, not a VM reboot. Clock range
coverage and sampled feedback do not prove zero message loss. The earlier
A* restart failure remains a failure and is excluded from this configuration's
24-goal denominator. The correction timing mechanism remains possible.

## Fixed startup and repeat test

Stop the prior launch and wait for exit, then run:

```bash
bash /mnt/hgfs/robot_project/scripts/start_amcl_update002_trial.sh
```

This launcher sources ~/ros2_ws, ~/tf2_fix_ws and ~/nav2_goal_fix_ws and
checks the controller package prefix. It uses room_01.yaml by default;
an explicit map path may be supplied as its first argument. Default package
configuration files remain unchanged.

In a separate terminal, after the prior goal has finished and the robot is
within 0.3 m of A=(1,-1), with no concurrent teleop/navigation commands:

```bash
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
source "$HOME/tf2_fix_ws/install/local_setup.bash"
source "$HOME/nav2_goal_fix_ws/install/local_setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 -u "$HOME/ros2_ws/src/ros2_mobile_robot_sim/scripts/repeat_navigation.py" \
  --rounds 1 --finish-diagnostics \
  --behavior-tree /mnt/hgfs/robot_project/config/replanning_02hz_trial.xml
```

This moves the simulated robot. The launcher does not change Nav2's default
behavior tree: omit the explicit --behavior-tree and the tested combination
is no longer reproduced. Preserve bag/results/logs and the source overlays.
No additional repeated navigation is required to close this stage.
