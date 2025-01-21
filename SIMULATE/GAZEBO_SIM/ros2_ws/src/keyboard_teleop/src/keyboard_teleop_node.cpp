#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

// Key press handling for keyboard input
int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

class KeyboardTeleopNode : public rclcpp::Node
{
public:
  KeyboardTeleopNode() : Node("keyboard_teleop_node")
  {
    // 'keyboard_input/keyboard_event' 토픽을 구독하여 키보드 입력을 처리 (키보드 이벤트 발행 토픽에 맞추기)
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "keyboard_input/keyboard_event", 10, std::bind(&KeyboardTeleopNode::keyboardCallback, this, std::placeholders::_1));

    // Gazebo 로봇으로 명령을 보내는 publisher 생성
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

private:
  void keyboardCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    geometry_msgs::msg::Twist twist;

    // 수신된 키보드 입력을 콘솔에 출력
    RCLCPP_INFO(this->get_logger(), "Received key: '%s'", msg->data.c_str());

    // 'keyboard_input/keyboard_event' 토픽에서 받은 값에 따라 로봇 명령을 설정
    if (msg->data == "UP")
    {
      twist.linear.x = 1.0;
    }
    else if (msg->data == "DOWN")
    {
      twist.linear.x = -1.0;
    }
    else if (msg->data == "LEFT")
    {
      twist.angular.z = 1.0;
    }
    else if (msg->data == "RIGHT")
    {
      twist.angular.z = -1.0;
    }
    else if (msg->data == "STOP")
    {
      twist.linear.x = 0.0;
      twist.angular.z = 0.0;
    }

    // /cmd_vel 토픽에 명령을 발행
    publisher_->publish(twist);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardTeleopNode>());
  rclcpp::shutdown();
  return 0;
}
