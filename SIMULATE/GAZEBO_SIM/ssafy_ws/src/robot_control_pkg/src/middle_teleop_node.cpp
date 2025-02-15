/*
250207 수정
- 키보드 토픽 /key_pulisher -> /key_input
- 기존 /ssafy/key_input -> /robot_1/key_input
- 로봇 status 토픽 구독
- 로봇 status에서 mode가 manual 일때만 동작하도록 수정
*/
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include "robot_custom_interfaces/msg/status.hpp"
#include <algorithm>  // std::clamp
#include <cmath>      // std::fabs, std::sqrt, std::pow
// test
class MiddleTeleop : public rclcpp::Node
{
public:
  MiddleTeleop()
  : Node("middle_teleop_node"),
    linear_vel_(0.0),
    angular_vel_(0.0)
  {
    // 파라미터 선언 및 초기화
    this->declare_parameter<std::string>("robot_name", "not_defined");
    this->declare_parameter<int>("robot_number", -1);

    robot_name_ = this->get_parameter("robot_name").as_string();
    robot_num_ = this->get_parameter("robot_number").as_int();

    // 구독 및 발행 토픽 설정
    std::string key_topic = "/robot_" + std::to_string(robot_num_) + "/key_input";
    std::string cmd_vel_topic = "/" + robot_name_ + "/cmd_vel";
    //std::string cmd_vel_topic = "/cmd_vel";
    std::string robot_status_topic = "/robot_" + std::to_string(robot_num_) + "/status";
    
    //status 토픽 구독
    status_subscription_ = this->create_subscription<robot_custom_interfaces::msg::Status>(
      robot_status_topic,
      10,
      [this](const robot_custom_interfaces::msg::Status::SharedPtr msg) {
        if (msg->mode == "manual") {
          if(!is_manual_mode){
            RCLCPP_INFO(this->get_logger(), "Manual mode");
            is_manual_mode = true;
          }
        } 
      }
    );

    // key_input 토픽 구독
    key_subscription_ = this->create_subscription<std_msgs::msg::String>(
      key_topic,
      10,
      std::bind(&MiddleTeleop::keyCallback, this, std::placeholders::_1)
    );

    // cmd_vel 토픽 발행
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);


    // 일정 주기로 속도 갱신(가속/감속)하는 타이머
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50), // 20Hz
      std::bind(&MiddleTeleop::controlLoop, this)
    );

    RCLCPP_INFO(this->get_logger(), "MiddleTeleop node has started.");
  }

private:
  void keyCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    auto now = std::chrono::steady_clock::now(); // 시스템 시간 사용
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
    }
  }

  void controlLoop()
  {
    auto now = std::chrono::steady_clock::now(); // 시스템 시간 사용

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

      if (std::fabs(linear_vel_) < 0.001) {
        linear_vel_ = 0.0;
      }
      if (std::fabs(angular_vel_) < 0.001) {
        angular_vel_ = 0.0;
      }
    } else {
      double forward_sign = 0.0;
      if (up) forward_sign += 1.0;
      if (down) forward_sign -= 1.0;

      double turn_sign = 0.0;
      if (left) turn_sign += 1.0;
      if (right) turn_sign -= 1.0;

      if (forward_sign != 0.0) {
        linear_vel_ += forward_sign * ACCEL_STEP;
        linear_vel_ = std::clamp(linear_vel_, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
      } else {
        linear_vel_ *= FRICTION_FACTOR_LINEAR;
        if (std::fabs(linear_vel_) < 0.001) {
          linear_vel_ = 0.0;
        }
      }

      if (turn_sign != 0.0) {
        angular_vel_ += turn_sign * ANGLE_STEP;
        angular_vel_ = std::clamp(angular_vel_, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
      } else {
        angular_vel_ *= FRICTION_FACTOR_ANGULAR;
        if (std::fabs(angular_vel_) < 0.001) {
          angular_vel_ = 0.0;
        }
      }
    }

    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = linear_vel_;
    cmd_vel_msg.angular.z = angular_vel_;
    //manual mode일때만 publish
    if(is_manual_mode){    
        cmd_vel_publisher_->publish(cmd_vel_msg);
    }
    else{
        static auto last_error_log_time = std::chrono::steady_clock::now();
        auto current_time = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(current_time - last_error_log_time).count() >= 1) {
            //RCLCPP_INFO(this->get_logger(), "Current Mode is not Manual mode");
            last_error_log_time = current_time;
        }
    }
  }
  bool is_manual_mode = false;
  std::string robot_name_;
  int robot_num_;
  double linear_vel_;
  double angular_vel_;
  std::chrono::steady_clock::time_point up_last_time_, down_last_time_, left_last_time_, right_last_time_, space_last_time_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr key_subscription_;
  rclcpp::Subscription<robot_custom_interfaces::msg::Status>::SharedPtr status_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  static constexpr double ACCEL_STEP = 0.17;
  static constexpr double ANGLE_STEP = 0.10;
  static constexpr double BRAKE_FACTOR_LINEAR = 0.01;
  static constexpr double BRAKE_FACTOR_ANGULAR = 0.01;
  static constexpr double FRICTION_FACTOR_LINEAR = 0.90;
  static constexpr double FRICTION_FACTOR_ANGULAR = 0.90;
  static constexpr double MAX_LINEAR_SPEED = 2.0;
  static constexpr double MAX_ANGULAR_SPEED = 2.0;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MiddleTeleop>());
  rclcpp::shutdown();
  return 0;
}
