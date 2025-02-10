/**********************************
    Created on : 10th Jan  2024
    Ported to ROS2 by: Shin Hyeon-hak 
    Github : Carepediem324
**********************************/

#include "rclcpp/rclcpp.hpp"
#include "localization_sensor_fusion_pkg/ukf.hpp"  // ukf.hpp의 경로에 맞게 조정

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    // "ukf_node"라는 이름의 노드를 생성
    auto node = rclcpp::Node::make_shared("ukf_node");

    // UKF 클래스의 인스턴스를 생성 (node를 전달)
    auto ukf = std::make_shared<UKF>(node);

    rclcpp::Rate loop_rate(30); // 30 Hz 주기로 실행

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        ukf->estimateVehicleState(); // UKF 추정 실행
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
