#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

class CalVelocityNode : public rclcpp::Node
{
public:
  CalVelocityNode()
  : Node("cal_velocity_node"),
    linear_velocity_(0.0),
    angular_velocity_(0.0)
  {
    // cmd_vel 구독
    velocity_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&CalVelocityNode::velocityCallback, this, std::placeholders::_1)
    );

    // 속력 (m/s) 발행
    speed_publisher_mps_ = this->create_publisher<std_msgs::msg::Float64>("calculated_speed_mps", 10);

    // 속력 (km/h) 발행
    speed_publisher_kmh_ = this->create_publisher<std_msgs::msg::Float64>("calculated_speed_kmh", 10);

    // 1초 주기로 속력 발행
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&CalVelocityNode::publishSpeed, this)
    );

    RCLCPP_INFO(this->get_logger(), "CalVelocityNode (cal_velocity_node) has started.");
  }

private:
  void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // 선속도와 각속도 업데이트
    linear_velocity_ = msg->linear.x;     // m/s
    angular_velocity_ = msg->angular.z;  // rad/s
  }

  void publishSpeed()
  {
    // 회전 반경 (TurtleBot3 기준, m)
    const double turning_radius = 0.08;

    // 속력 계산 (m/s)
    double speed_mps = std::sqrt(std::pow(linear_velocity_, 2) + std::pow(turning_radius * angular_velocity_, 2));

    // 속력 변환 (km/h)
    double speed_kmh = speed_mps * 3.6;

    // m/s 단위 속력 발행
    std_msgs::msg::Float64 speed_msg_mps;
    speed_msg_mps.data = speed_mps;
    speed_publisher_mps_->publish(speed_msg_mps);

    // km/h 단위 속력 발행
    std_msgs::msg::Float64 speed_msg_kmh;
    speed_msg_kmh.data = speed_kmh;
    speed_publisher_kmh_->publish(speed_msg_kmh);

    // 로그 출력
    RCLCPP_INFO(this->get_logger(),
                "Calculated Speed: %.2f m/s, %.2f km/h",
                speed_mps, speed_kmh);
  }

  double linear_velocity_;  // 선속도 (m/s)
  double angular_velocity_; // 각속도 (rad/s)

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscription_; // cmd_vel 구독자
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_publisher_mps_;        // m/s 퍼블리셔
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_publisher_kmh_;        // km/h 퍼블리셔
  rclcpp::TimerBase::SharedPtr timer_;                                             // 타이머
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CalVelocityNode>());
  rclcpp::shutdown();
  return 0;
}
