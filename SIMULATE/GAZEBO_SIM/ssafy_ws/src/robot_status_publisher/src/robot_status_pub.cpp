#include <chrono>  // 변경 사항: 타이머에 필요한 헤더
#include "rclcpp/rclcpp.hpp"
#include "robot_custom_interfaces/msg/status.hpp"

// 변경 사항: chrono_literals 사용을 위해 추가
using namespace std::chrono_literals;  

class RobotStatusPublisher : public rclcpp::Node
{
public:
    // 변경 사항: 노드 이름을 "robot_status_pub_node"로 변경 (원하시는 이름으로 쓰면 됨)
    RobotStatusPublisher()
    : Node("robot_status_pub_node")
    {
        // 변경 사항 없음: 퍼블리셔 생성
        publisher_ = this->create_publisher<robot_custom_interfaces::msg::Status>("robot_status", 10);

        // 변경 사항: 1초 → 500ms 주기로 변경 (원하시면 다시 1s로 바꾸세요)
        timer_ = this->create_wall_timer(
            500ms,  // 1s를 쓰고 싶다면 500ms 대신 1000ms 또는 1s
            std::bind(&RobotStatusPublisher::publish_status, this)
        );
    }

private:
    void publish_status()
    {
        auto message = robot_custom_interfaces::msg::Status();
        message.id = 1;
        message.status = "operational";
        message.temperature = 36.5f; // 변경 사항: 소수점 표현을 위해 뒤에 f 추가
        message.is_active = true;

        RCLCPP_INFO(
            this->get_logger(),
            "Publishing: ID=%d, Status=%s, Temperature=%.2f, Active=%s",
            message.id,
            message.status.c_str(),
            message.temperature,
            message.is_active ? "true" : "false"
        );

        publisher_->publish(message);
    }

    rclcpp::Publisher<robot_custom_interfaces::msg::Status>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotStatusPublisher>());
    rclcpp::shutdown();
    return 0;
}
