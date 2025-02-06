// 파일: src/lidar_subscriber_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class LidarSubscriber : public rclcpp::Node
{
public:
  LidarSubscriber()
  : Node("lidar_subscriber")
  {
    // robot_name 파라미터 선언 (기본값 "not_defined")
    this->declare_parameter<std::string>("robot_name", "not_defined");
    // 파라미터에서 robot_name 값을 가져옴
    std::string robot_name = this->get_parameter("robot_name").as_string();

    // 동적으로 토픽 이름 생성: "/{robot_name}/velodyne_points"
    std::string velodyne_topic_name = "/" + robot_name + "/velodyne_points";
    RCLCPP_INFO(this->get_logger(), "Subscribing to topic: %s", velodyne_topic_name.c_str());

    // 토픽 구독자 생성
    velodyne_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      velodyne_topic_name, 10,
      std::bind(&LidarSubscriber::lidar_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Lidar Subscriber Node has been started.");
  }

private:
  // 구독 콜백 함수: 메시지를 수신할 때마다 호출
  void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // 예시로 메시지의 width와 height 값을 로그로 출력합니다.
    RCLCPP_INFO(this->get_logger(), "Received LiDAR data - width: %d, height: %d", msg->width, msg->height);
  }

  // 구독자 객체
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_subscription;
};

int main(int argc, char **argv)
{
  // ROS2 초기화
  rclcpp::init(argc, argv);
  // 노드 객체 생성 후 스핀 (콜백 실행)
  rclcpp::spin(std::make_shared<LidarSubscriber>());
  // ROS2 종료
  rclcpp::shutdown();
  return 0;
}
