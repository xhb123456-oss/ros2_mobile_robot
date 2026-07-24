#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class TrajectoryCommander : public rclcpp::Node {
 public:
  TrajectoryCommander() : Node("trajectory_commander"), start_time_(this->now()) {
    mode_ = this->declare_parameter<std::string>("mode", "figure_eight");
    linear_speed_ = this->declare_parameter<double>("linear_speed", 0.35);
    angular_speed_ = this->declare_parameter<double>("angular_speed", 0.65);
    filter_alpha_ = this->declare_parameter<double>("filter_alpha", 0.18);
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&TrajectoryCommander::publish_command, this));
    RCLCPP_INFO(this->get_logger(), "Trajectory mode: %s", mode_.c_str());
  }

 private:
  void publish_command() {
    geometry_msgs::msg::Twist target;
    const double elapsed = (this->now() - start_time_).seconds();

    if (mode_ == "straight") {
      target.linear.x = linear_speed_;
    } else if (mode_ == "rotate") {
      target.angular.z = angular_speed_;
    } else if (mode_ == "figure_eight") {
      target.linear.x = linear_speed_;
      // Alternating curvature generates a continuous figure-eight-like trajectory.
      target.angular.z = angular_speed_ * std::sin(0.45 * elapsed);
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "Unknown mode '%s'; publishing zero velocity", mode_.c_str());
    }
    // First-order low-pass filter prevents abrupt velocity changes in simulation.
    filtered_linear_ += filter_alpha_ * (target.linear.x - filtered_linear_);
    filtered_angular_ += filter_alpha_ * (target.angular.z - filtered_angular_);
    geometry_msgs::msg::Twist command;
    command.linear.x = filtered_linear_;
    command.angular.z = filtered_angular_;
    publisher_->publish(command);
  }

  std::string mode_;
  double linear_speed_{};
  double angular_speed_{};
  double filter_alpha_{};
  double filtered_linear_{};
  double filtered_angular_{};
  rclcpp::Time start_time_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryCommander>());
  rclcpp::shutdown();
  return 0;
}
