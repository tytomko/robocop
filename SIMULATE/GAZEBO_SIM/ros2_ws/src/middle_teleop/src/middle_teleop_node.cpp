#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include <algorithm>
#include <cmath>

class MiddleTeleop : public rclcpp::Node
{
public:
  MiddleTeleop()
  : Node("middle_teleop_node"),
    linear_vel_(0.0),
    angular_vel_(0.0)
  {
    this->declare_parameter<std::string>("robot_name", "not_defined");
    this->declare_parameter<int>("robot_number", -1);

    robot_name_ = this->get_parameter("robot_name").as_string();
    robot_num_ = this->get_parameter("robot_number").as_int();

    std::string key_topic = "/" + robot_name_ + "/key_publisher";
    std::string cmd_vel_topic = "/" + robot_name_ + "/cmd_vel";

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      key_topic,
      10,
      std::bind(&MiddleTeleop::keyCallback, this, std::placeholders::_1)
    );

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&MiddleTeleop::controlLoop, this)
    );

    RCLCPP_INFO(this->get_logger(), "MiddleTeleop node has started.");
  }

private:
  void keyCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    auto now = std::chrono::steady_clock::now();

    // 특정 키 입력이 감지될 경우, 해당 키의 타임스탬프만 업데이트
    if (msg->data == "UP") {
      up_last_time_ = now;
    } else if (msg->data == "DOWN") {
      down_last_time_ = now;
    } else if (msg->data == "LEFT") {
      left_last_time_ = now;
    } else if (msg->data == "RIGHT") {
      right_last_time_ = now;
    } else if (msg->data == "SPACE") {
      space_last_time_ = now;
    } else if (msg->data == "UP_LEFT") {
      up_last_time_ = now;
      left_last_time_ = now;
    } else if (msg->data == "UP_RIGHT") {
      up_last_time_ = now;
      right_last_time_ = now;
    } else if (msg->data == "DOWN_LEFT") {
      down_last_time_ = now;
      left_last_time_ = now;
    } else if (msg->data == "DOWN_RIGHT") {
      down_last_time_ = now;
      right_last_time_ = now;
    }

    RCLCPP_INFO(this->get_logger(), "Received key input: %s", msg->data.c_str());
  }

  void controlLoop()
  {
    auto now = std::chrono::steady_clock::now();

    auto is_key_pressed_recently = [&](const auto &last_time) {
      return std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count() < 200;
    };

    bool up = is_key_pressed_recently(up_last_time_);
    bool down = is_key_pressed_recently(down_last_time_);
    bool left = is_key_pressed_recently(left_last_time_);
    bool right = is_key_pressed_recently(right_last_time_);
    bool space = is_key_pressed_recently(space_last_time_);

    if (space) {
      linear_vel_ *= BRAKE_FACTOR_LINEAR;
      angular_vel_ *= BRAKE_FACTOR_ANGULAR;
      if (std::fabs(linear_vel_) < 0.001) linear_vel_ = 0.0;
      if (std::fabs(angular_vel_) < 0.001) angular_vel_ = 0.0;
    } else {
      double forward_sign = 0.0;
      double turn_sign = 0.0;

      if (up) forward_sign += 1.0;
      if (down) forward_sign -= 1.0;
      if (left) turn_sign += 1.0;
      if (right) turn_sign -= 1.0;

      if (forward_sign != 0.0) {
        linear_vel_ += forward_sign * ACCEL_STEP;
        linear_vel_ = std::clamp(linear_vel_, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
      } else {
        linear_vel_ *= FRICTION_FACTOR_LINEAR;
        if (std::fabs(linear_vel_) < 0.001) linear_vel_ = 0.0;
      }

      if (turn_sign != 0.0) {
        angular_vel_ += turn_sign * ANGLE_STEP;
        angular_vel_ = std::clamp(angular_vel_, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
      } else {
        angular_vel_ *= FRICTION_FACTOR_ANGULAR;
        if (std::fabs(angular_vel_) < 0.001) angular_vel_ = 0.0;
      }
    }

    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = linear_vel_;
    cmd_vel_msg.angular.z = angular_vel_;
    cmd_vel_publisher_->publish(cmd_vel_msg);

    RCLCPP_INFO(this->get_logger(), "Velocity updated - Linear: %.2f, Angular: %.2f", linear_vel_, angular_vel_);
  }

  std::string robot_name_;
  int robot_num_;
  double linear_vel_;
  double angular_vel_;
  std::chrono::steady_clock::time_point up_last_time_, down_last_time_, left_last_time_, right_last_time_, space_last_time_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  static constexpr double ACCEL_STEP = 0.04;
  static constexpr double ANGLE_STEP = 0.03;
  static constexpr double BRAKE_FACTOR_LINEAR = 0.005;
  static constexpr double BRAKE_FACTOR_ANGULAR = 0.001;
  static constexpr double FRICTION_FACTOR_LINEAR = 0.98;
  static constexpr double FRICTION_FACTOR_ANGULAR = 0.95;
  static constexpr double MAX_LINEAR_SPEED = 1.5;
  static constexpr double MAX_ANGULAR_SPEED = 1.5;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MiddleTeleop>());
  rclcpp::shutdown();
  return 0;
}
