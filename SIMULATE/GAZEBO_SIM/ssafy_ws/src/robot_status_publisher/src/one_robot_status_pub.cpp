#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "robot_custom_interfaces/msg/status.hpp"

// 새로 추가된 헤더들
#include <sensor_msgs/msg/imu.hpp>       
#include <tf2/LinearMath/Quaternion.h>   
#include <tf2/LinearMath/Matrix3x3.h>    
#include <std_msgs/msg/float32.hpp>      

using namespace std::chrono_literals;

class RobotStatusPublisher : public rclcpp::Node
{
public:
    RobotStatusPublisher()
    : Node("robot_status_pub_node")
    {
        // --------------------
        // 1) 파라미터 선언
        // --------------------
        this->declare_parameter<std::string>("namespace_prefix", "");

        // 파라미터 값 가져오기
        auto prefix = this->get_parameter("namespace_prefix").as_string();

        // prefix가 "ssafy"라면 토픽 이름에 "/ssafy"를 붙이도록 설정
        if (prefix == "ssafy") {
            topic_prefix_ = "/ssafy";
        } else {
            topic_prefix_ = "";
        }

        // --------------------
        // 2) IMU 구독
        // --------------------
        subscription_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            topic_prefix_ + "/parameter/imu",  // 예: /ssafy/parameter/imu
            10,
            std::bind(&RobotStatusPublisher::imu_callback, this, std::placeholders::_1)
        );

        // --------------------
        // 3) Heading 발행
        // --------------------
        publisher_heading_ = this->create_publisher<std_msgs::msg::Float32>(
            topic_prefix_ + "/parameter/heading", // 예: /ssafy/parameter/heading
            10
        );

        // --------------------
        // 기존 로봇 상태 퍼블리셔
        // --------------------
        publisher_status_ = this->create_publisher<robot_custom_interfaces::msg::Status>(
            "robot_status", // 여기서도 토픽 이름을 바꾸고 싶다면 앞에 prefix_를 붙이면 됨
            10
        );

        // 기존 타이머(500ms)
        timer_ = this->create_wall_timer(
            500ms,
            std::bind(&RobotStatusPublisher::publish_status, this)
        );
    }

private:
    // --------------------
    // IMU 콜백 함수
    // --------------------
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // 쿼터니언에서 RPY 계산
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );
        tf2::Matrix3x3 m(q);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);  // radian 단위

        // yaw(heading)만 발행
        auto heading_msg = std_msgs::msg::Float32();
        heading_msg.data = static_cast<float>(yaw);

        RCLCPP_INFO(
            this->get_logger(),
            "IMU received. Yaw (heading): %.2f (rad)",
            yaw
        );

        publisher_heading_->publish(heading_msg);
    }

    // --------------------
    // 기존 상태 퍼블리시 함수
    // --------------------
    void publish_status()
    {
        auto message = robot_custom_interfaces::msg::Status();
        message.id = 1;
        message.status = "operational";
        message.temperature = 36.5f; 
        message.is_active = true;

        RCLCPP_INFO(
            this->get_logger(),
            "Publishing: ID=%d, Status=%s, Temperature=%.2f, Active=%s",
            message.id,
            message.status.c_str(),
            message.temperature,
            message.is_active ? "true" : "false"
        );

        publisher_status_->publish(message);
    }

    // --------------------
    // 멤버 변수
    // --------------------
    // prefix를 저장할 변수
    std::string topic_prefix_;

    // IMU 구독용
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu_;

    // Heading 발행용
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_heading_;

    // 기존 로봇 상태 발행용
    rclcpp::Publisher<robot_custom_interfaces::msg::Status>::SharedPtr publisher_status_;

    // 타이머
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotStatusPublisher>());
    rclcpp::shutdown();
    return 0;
}
