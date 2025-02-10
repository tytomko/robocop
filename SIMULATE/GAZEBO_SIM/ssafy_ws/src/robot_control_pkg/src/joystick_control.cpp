#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "robot_custom_interfaces/msg/status.hpp"
#include <algorithm>  // std::clamp
#include <cmath>      // std::fabs, std::sqrt, std::pow

class MiddleTeleop : public rclcpp::Node
{
public:
  MiddleTeleop()
  : Node("middle_teleop_node"),
    linear_vel_(0.0),
    angular_vel_(0.0)
  {
    // 파라미터 선언 및 초기화
    this->declare_parameter<std::string>("robot_name", "ssafy");
    this->declare_parameter<int>("robot_number", 1);

    robot_name_ = this->get_parameter("robot_name").as_string();
    robot_num_ = this->get_parameter("robot_number").as_int();

    // 구독 및 발행 토픽 설정
    std::string cmd_vel_topic = "/" + robot_name_ + "/cmd_vel";
    std::string robot_status_topic = "/robot_" + std::to_string(robot_num_) + "/status";
    std::string joy_topic = "/joy";  // joystick 데이터 토픽

    // status 토픽 구독
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

    // joy 토픽 구독 (블루투스 joystick)
    joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      joy_topic,
      10,
      std::bind(&MiddleTeleop::joyCallback, this, std::placeholders::_1)
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
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // joystick 입력을 바탕으로 linear_velocity와 angular_velocity를 설정
    // msg->axes[1] = 앞뒤 이동 (좌/우 조작은 axes[0], 회전은 axes[3] 등)

    linear_vel_ = msg->axes[1] * MAX_LINEAR_SPEED;  // 앞뒤
    angular_vel_ = msg->axes[0] * MAX_ANGULAR_SPEED; // 좌우 회전
  }

  void controlLoop()
  {
    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = linear_vel_;
    cmd_vel_msg.angular.z = angular_vel_;

    // manual mode일때만 publish
    if(is_manual_mode){    
        cmd_vel_publisher_->publish(cmd_vel_msg);
    }
    else{
        static auto last_error_log_time = std::chrono::steady_clock::now();
        auto current_time = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(current_time - last_error_log_time).count() >= 1) {
            RCLCPP_INFO(this->get_logger(), "Current Mode is not Manual mode");
            last_error_log_time = current_time;
        }
    }
  }

  bool is_manual_mode = false;
  std::string robot_name_;
  int robot_num_;
  double linear_vel_;
  double angular_vel_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Subscription<robot_custom_interfaces::msg::Status>::SharedPtr status_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

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
