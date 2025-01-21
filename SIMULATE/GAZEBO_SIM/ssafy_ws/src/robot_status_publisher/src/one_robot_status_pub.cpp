#include <chrono>
#include <string> // std::string, std::to_string 사용 시 필요한 경우
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
        // 1) 파라미터 선언
        this->declare_parameter<std::string>("namespace_prefix", "");
        this->declare_parameter<int>("robot_number", 1);

        auto prefix = this->get_parameter("namespace_prefix").as_string();
        auto robot_num = this->get_parameter("robot_number").as_int();

        // 2) 구독 토픽 결정
        std::string imu_topic = namespace_prefix + "/imu";

        // 3) 발행 토픽 결정
        //    예) robot_num = 3 --> /robot_3/heading
        std::string heading_topic = "/robot_" + std::to_string(robot_num) + "/heading";
        std::string status_topic = "/robot_" + std::to_string(robot_num) + "/status";
        // 디버그용 로그
        RCLCPP_INFO(
            this->get_logger(),
            "Node initialized: namespace_prefix='%s', robot_number=%ld, imu_topic='%s', heading_topic='%s'",
            prefix.c_str(), robot_num, imu_topic.c_str(), heading_topic.c_str()
        );

        // IMU 구독
        subscription_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic,
            10,
            std::bind(&RobotStatusPublisher::imu_callback, this, std::placeholders::_1)
        );

        // Heading 발행
        publisher_heading_ = this->create_publisher<std_msgs::msg::Float32>(
            heading_topic,
            10
        );

        // 기존 로봇 상태 퍼블리셔
        publisher_status_ = this->create_publisher<robot_custom_interfaces::msg::Status>(
            status_topic, 
            10
        );

        // 기존 타이머(500ms)
        timer_ = this->create_wall_timer(
            500ms,
            std::bind(&RobotStatusPublisher::publish_status, this)
        );
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );
        tf2::Matrix3x3 m(q);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

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

    // 멤버 변수
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_heading_;
    rclcpp::Publisher<robot_custom_interfaces::msg::Status>::SharedPtr publisher_status_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotStatusPublisher>());
    rclcpp::shutdown();
    return 0;
}
