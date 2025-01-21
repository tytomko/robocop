#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MiddleTeleop : public rclcpp::Node
{
public:
  MiddleTeleop() : Node("middle_teleop_node")
  {
    // key_publisher 토픽 구독
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/key_publisher", 
      10,
      std::bind(&MiddleTeleop::keyCallback, this, std::placeholders::_1)
    );

    // cmd_vel 토픽 발행 (필요 시)
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  void keyCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    // 로그로 현재 수신한 키를 출력
    RCLCPP_INFO(this->get_logger(), "Received key input: %s", msg->data.c_str());

    // 이후 로봇 제어가 필요하다면, 아래 로직처럼 추가 가능
    auto cmd_vel_msg = geometry_msgs::msg::Twist();
    
    if (msg->data == "UP") {
      // 전진
      cmd_vel_msg.linear.x = 0.5;
      cmd_vel_msg.angular.z = 0.0;
    } else if (msg->data == "DOWN") {
      // 후진
      cmd_vel_msg.linear.x = -0.5;
      cmd_vel_msg.angular.z = 0.0;
    } else if (msg->data == "LEFT") {
      // 좌회전
      cmd_vel_msg.linear.x = 0.0;
      cmd_vel_msg.angular.z = 1.0;
    } else if (msg->data == "RIGHT") {
      // 우회전
      cmd_vel_msg.linear.x = 0.0;
      cmd_vel_msg.angular.z = -1.0;
    } else if (msg->data == "SPACE") {
      // 브레이크
      cmd_vel_msg.linear.x = 0.0;
      cmd_vel_msg.angular.z = 0.0;
    }

    // cmd_vel 토픽으로 메시지 발행
    cmd_vel_publisher_->publish(cmd_vel_msg);
  }

  // 구독자와 퍼블리셔 선언
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MiddleTeleop>());
  rclcpp::shutdown();
  return 0;
}
