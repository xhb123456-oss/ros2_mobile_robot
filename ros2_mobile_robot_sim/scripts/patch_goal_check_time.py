"""Patch a separate Nav2 1.1.20 source checkout, never the installed binary."""
import argparse
from pathlib import Path

OLD = '''  nav_2d_utils::transformPose(
    costmap_ros_->getTfBuffer(), costmap_ros_->getGlobalFrameID(),
    end_pose_, transformed_end_pose, tolerance);
'''
NEW = '''  // Compare the fixed map goal and robot pose at the same current time.
  // The path creation stamp must not freeze the map-to-odom correction.
  auto goal_at_robot_time = end_pose_;
  goal_at_robot_time.header.stamp = pose.header.stamp;
  if (!nav_2d_utils::transformPose(
      costmap_ros_->getTfBuffer(), costmap_ros_->getGlobalFrameID(),
      goal_at_robot_time, transformed_end_pose, tolerance))
  {
    return false;
  }
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('checkout', type=Path)
    args = parser.parse_args()
    source = args.checkout / 'nav2_controller/src/controller_server.cpp'
    text = source.read_text(encoding='utf-8')
    if NEW in text:
        print('Patch already present:', source)
        return
    if text.count(OLD) != 1:
        raise SystemExit('Source does not match expected Nav2 1.1.20 block; no changes made.')
    backup = source.with_suffix('.cpp.before_goal_time_fix')
    if backup.exists():
        raise SystemExit('Backup already exists; inspect checkout before retrying.')
    backup.write_text(text, encoding='utf-8')
    source.write_text(text.replace(OLD, NEW), encoding='utf-8')
    print('Patched:', source)
    print('Backup:', backup)


if __name__ == '__main__':
    main()
