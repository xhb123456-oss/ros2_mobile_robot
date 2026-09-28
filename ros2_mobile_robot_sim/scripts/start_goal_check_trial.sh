#!/usr/bin/env bash
# Explicit candidate launcher. Does not change installed/default configs.
# Navigation tests must still pass replanning_02hz_trial.xml per goal.
set -e
trial_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
trial_map="${1:-$HOME/ros2_ws/maps/room_01.yaml}"
for trial_file in \
  /opt/ros/humble/setup.bash \
  "$HOME/ros2_ws/install/setup.bash" \
  "$HOME/tf2_fix_ws/install/local_setup.bash" \
  "$HOME/nav2_goal_fix_ws/install/local_setup.bash" \
  "$trial_root/config/goal_check_trial.yaml" \
  "$trial_root/config/amcl_tf_trial.yaml" \
  "$trial_map"
do
  if [[ ! -f "$trial_file" ]]; then
    printf 'Missing required file: %s\n' "$trial_file" >&2
    exit 1
  fi
done
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
source "$HOME/tf2_fix_ws/install/local_setup.bash"
source "$HOME/nav2_goal_fix_ws/install/local_setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
trial_prefix="$(ros2 pkg prefix nav2_controller)"
if [[ "$(readlink -f "$trial_prefix")" != "$(readlink -f "$HOME/nav2_goal_fix_ws/install/nav2_controller")" ]]; then
  printf 'Unexpected nav2_controller prefix: %s\n' "$trial_prefix" >&2
  exit 1
fi
printf 'Controller: %s\nMap: %s\n' "$trial_prefix" "$trial_map"
printf 'Candidate launch; repeated tests still require --behavior-tree replanning_02hz_trial.xml\n'
exec ros2 launch mobile_robot_sim navigation.launch.py \
  "map:=$trial_map" \
  "nav_overrides:=$trial_root/config/goal_check_trial.yaml" \
  "amcl_overrides:=$trial_root/config/amcl_tf_trial.yaml"
