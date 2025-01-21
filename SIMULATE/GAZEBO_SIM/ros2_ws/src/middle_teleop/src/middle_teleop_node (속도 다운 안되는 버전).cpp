#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>  // For std::clamp
#include <cmath>      // For std::fabs

class MiddleTeleop : public rclcpp::Node
{
public:
  MiddleTeleop()
  : Node("middle_teleop_node"),
    linear_vel_(0.0),
    angular_vel_(0.0)
  {
    // key_publisher 토픽 구독
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/key_publisher",
      10,
      std::bind(&MiddleTeleop::keyCallback, this, std::placeholders::_1)
    );

    // cmd_vel 토픽 발행
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "MiddleTeleop node has started.");
  }

private:
  void keyCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    // 키 입력 로그 출력
    RCLCPP_INFO(this->get_logger(), "Received key input: %s", msg->data.c_str());

    // 키에 따라 속도 업데이트
    if (msg->data == "UP") {
      // 전진 가속
      linear_vel_ += ACCEL_STEP;
      // 최대 속도 제한
      linear_vel_ = std::clamp(linear_vel_, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);

    } else if (msg->data == "DOWN") {
      // 후진 가속
      linear_vel_ -= ACCEL_STEP;
      // 최대 속도 제한
      linear_vel_ = std::clamp(linear_vel_, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);

    } else if (msg->data == "LEFT") {
      // 좌회전 (각속도 증가)
      angular_vel_ += ANGLE_STEP;
      // 최대 각속도 제한
      angular_vel_ = std::clamp(angular_vel_, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

    } else if (msg->data == "RIGHT") {
      // 우회전 (각속도 감소)
      angular_vel_ -= ANGLE_STEP;
      // 최대 각속도 제한
      angular_vel_ = std::clamp(angular_vel_, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

    } else if (msg->data == "SPACE") {
      // 브레이크: 선속도와 각속도를 모두 급격히 줄임
      linear_vel_  *= BRAKE_FACTOR;  // 예: 0.7배로 감속
      angular_vel_ *= BRAKE_FACTOR;

      // 아주 작아지면 0으로 보정
      if (std::fabs(linear_vel_) < 0.01) {
        linear_vel_ = 0.0;
      }
      if (std::fabs(angular_vel_) < 0.01) {
        angular_vel_ = 0.0;
      }
    }

    // 업데이트된 선속도, 각속도를 메시지에 반영
    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = linear_vel_;
    cmd_vel_msg.angular.z = angular_vel_;

    // 토픽 발행
    cmd_vel_publisher_->publish(cmd_vel_msg);
  }

  // 구독자, 퍼블리셔
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

  // 현재 선속도, 각속도 (계속 누적)
  double linear_vel_;
  double angular_vel_;

  // 상수들 (필요에 따라 조절)
  static constexpr double ACCEL_STEP       = 0.1;  // 전/후진 시 한번 누를 때 가속량
  static constexpr double ANGLE_STEP       = 0.1;  // 좌/우 회전 시 한번 누를 때 변화량
  static constexpr double BRAKE_FACTOR     = 0.4;  // SPACE 누를 때 속도 곱해줄 감속 비율
  static constexpr double MAX_LINEAR_SPEED = 2.0;  // 최대 선속도
  static constexpr double MAX_ANGULAR_SPEED= 2.0;  // 최대 각속도
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MiddleTeleop>());
  rclcpp::shutdown();
  return 0;
}
