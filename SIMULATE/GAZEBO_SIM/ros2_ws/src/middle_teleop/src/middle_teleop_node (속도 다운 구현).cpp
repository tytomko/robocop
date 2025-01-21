#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>  // std::clamp
#include <cmath>      // std::fabs

class MiddleTeleop : public rclcpp::Node
{
public:
  MiddleTeleop()
  : Node("middle_teleop_node"),
    linear_vel_(0.0),
    angular_vel_(0.0),
    up_last_time_(0, 0, get_clock()->get_clock_type()),
    down_last_time_(0, 0, get_clock()->get_clock_type()),
    left_last_time_(0, 0, get_clock()->get_clock_type()),
    right_last_time_(0, 0, get_clock()->get_clock_type()),
    space_last_time_(0, 0, get_clock()->get_clock_type())
  {
    // key_publisher 토픽 구독
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/key_publisher",
      10,
      std::bind(&MiddleTeleop::keyCallback, this, std::placeholders::_1)
    );

    // cmd_vel 토픽 발행
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // 일정 주기로 속도 갱신(가속/감속)하는 타이머
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50), // 20Hz
      std::bind(&MiddleTeleop::controlLoop, this)
    );

    RCLCPP_INFO(this->get_logger(), "MiddleTeleop node has started.");
  }

private:
  // 키 입력 콜백: 키가 들어올 때마다 해당 키의 '마지막 입력 시각' 갱신
  void keyCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received key input: %s", msg->data.c_str());
    auto now = this->now();

    if (msg->data == "UP") {
      up_last_time_ = now;
    } 
    else if (msg->data == "DOWN") {
      down_last_time_ = now;
    }
    else if (msg->data == "LEFT") {
      left_last_time_ = now;
    }
    else if (msg->data == "RIGHT") {
      right_last_time_ = now;
    }
    else if (msg->data == "SPACE") {
      space_last_time_ = now;
    }
  }

  // 주기적으로 속도를 업데이트하고 cmd_vel 발행
  void controlLoop()
  {
    auto now = this->now();

    // 0.2초 안에 해당 키가 다시 입력됐으면 "누르고 있다"고 판단
    bool up    = (now - up_last_time_).seconds()    < 0.2;
    bool down  = (now - down_last_time_).seconds()  < 0.2;
    bool left  = (now - left_last_time_).seconds()  < 0.2;
    bool right = (now - right_last_time_).seconds() < 0.2;
    bool space = (now - space_last_time_).seconds() < 0.2;

    // 먼저 브레이크(스페이스) 처리: 점진적으로 감속
    if (space) {
      linear_vel_  *= BRAKE_FACTOR;  
      angular_vel_ *= BRAKE_FACTOR;

      // 충분히 작아지면 0으로
      if (std::fabs(linear_vel_) < 0.001) {
        linear_vel_ = 0.0;
      }
      if (std::fabs(angular_vel_) < 0.001) {
        angular_vel_ = 0.0;
      }
    }

    // 전/후진 방향 체크
    double forward_sign = 0.0;
    if (up)    forward_sign += 1.0;
    if (down)  forward_sign -= 1.0;

    // 좌/우 회전 방향 체크
    double turn_sign = 0.0;
    if (left)  turn_sign += 1.0;
    if (right) turn_sign -= 1.0;

    // 전진/후진 가속 처리
    if (forward_sign != 0.0) {
      // 가속
      linear_vel_ += forward_sign * ACCEL_STEP;
      // 최대 선속도 제한
      linear_vel_ = std::clamp(linear_vel_, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
    } else {
      // 키를 떼면 서서히 감속(마찰)
      linear_vel_ *= FRICTION_FACTOR;
      if (std::fabs(linear_vel_) < 0.001) {
        linear_vel_ = 0.0;
      }
    }

    // 좌/우 회전 가속 처리
    if (turn_sign != 0.0) {
      angular_vel_ += turn_sign * ANGLE_STEP;
      angular_vel_ = std::clamp(angular_vel_, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
    } else {
      angular_vel_ *= FRICTION_FACTOR;
      if (std::fabs(angular_vel_) < 0.001) {
        angular_vel_ = 0.0;
      }
    }

    // 최종 속도 메시지 작성
    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x  = linear_vel_;
    cmd_vel_msg.angular.z = angular_vel_;

    // 발행
    cmd_vel_publisher_->publish(cmd_vel_msg);
  }

  // ---------- 멤버 변수들 ----------

  // 현재 선속도, 각속도
  double linear_vel_;
  double angular_vel_;

  // 마지막 키 입력 시각
  rclcpp::Time up_last_time_, down_last_time_, left_last_time_, right_last_time_, space_last_time_;

  // 구독자/퍼블리셔/타이머
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 상수들 (필요에 따라 조절하세요)
  static constexpr double ACCEL_STEP       = 0.07;  // 전/후진 시 가속량
  static constexpr double ANGLE_STEP       = 0.09;   // 좌/우 회전 시 가속량
  static constexpr double BRAKE_FACTOR     = 0.3;   // 브레이크(SPACE) 시 속도 곱해줄 비율
  static constexpr double FRICTION_FACTOR  = 0.95;  // 키 떼었을 때 감속 비율 (마찰)
  static constexpr double MAX_LINEAR_SPEED = 2.0;   // 최대 선속도
  static constexpr double MAX_ANGULAR_SPEED= 2.0;   // 최대 각속도
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MiddleTeleop>());
  rclcpp::shutdown();
  return 0;
}
