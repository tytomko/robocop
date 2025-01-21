#include <chrono>
#include <string> // std::string, std::to_string 사용 시 필요한 경우
#include "rclcpp/rclcpp.hpp"
#include "robot_custom_interfaces/msg/status.hpp"

// 새로 추가된 헤더들
#include <GeographicLib/UTMUPS.hpp>
#include <sensor_msgs/msg/imu.hpp>       
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>   
#include <tf2/LinearMath/Matrix3x3.h>    
#include <std_msgs/msg/float32.hpp>      

using namespace std::chrono_literals;

class RobotStatusPublisher : public rclcpp::Node
{

public:
    //전역 변수 선언
    std::string robot_name;
    int robot_num;

    RobotStatusPublisher()
    : Node("robot_status_pub_node")
    {
        // 1) 파라미터 선언
        this->declare_parameter<std::string>("robot_name", "not_defined");
        this->declare_parameter<int>("robot_number", -1);

        robot_name = this->get_parameter("robot_name").as_string();
        robot_num = this->get_parameter("robot_number").as_int();

        // 2) 구독 토픽 결정
        std::string imu_topic = "/" + robot_name + "/imu";
        std::string gps_topic = "/" + robot_name + "/gps";

        // 3) 발행 토픽 결정
        //    예) robot_num = 3 --> /robot_3/heading
        std::string heading_topic = "/robot_" + std::to_string(robot_num) + "/heading";
        std::string status_topic = "/robot_" + std::to_string(robot_num) + "/status";
        // 디버그용 로그
        RCLCPP_INFO(
            this->get_logger(),
            "Node initialized: robot_name='%s', robot_number=%d, imu_topic='%s', heading_topic='%s'",
            robot_name.c_str(), robot_num, imu_topic.c_str(), heading_topic.c_str()
        );

        // IMU 구독
        subscription_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic,
            10,
            std::bind(&RobotStatusPublisher::imu_callback, this, std::placeholders::_1)
        );

        // GPS 구독
        subscription_gps_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            gps_topic,
            10,
            std::bind(&RobotStatusPublisher::gps_callback, this, std::placeholders::_1)
        );

        // UTM 좌표 퍼블리셔
        publisher_utm_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/robot_" + std::to_string(robot_num) + "/utm_pose",
            10
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

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        double lat = msg->latitude;
        double lon = msg->longitude;
        double alt = msg->altitude;

        // GeographicLib에서 필요한 변수
        int zone;      // 1~60
        // zone = 52;   // 예: 서울 경도에 해당하는 UTM zone
        bool northp;   // 북반구(true) / 남반구(false)
        // 서울은 북반구
        double x, y;   // Easting, Northing (단위: 미터)

        // (1) 위경도 -> UTM 자동 변환
        // zone 자동 계산
        GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, x, y);

        // (2) PoseStamped 메시지 생성
        geometry_msgs::msg::PoseStamped utm_msg;
        utm_msg.header.stamp = msg->header.stamp;
        utm_msg.header.frame_id = "map";  // 예: 지도 좌표계 프레임

        // UTM 좌표 대입
        utm_msg.pose.position.x = x;
        utm_msg.pose.position.y = y;
        utm_msg.pose.position.z = alt;

        // (3) 로깅
        // Zone + (N/S) 문자열 만들기
        std::string utm_zone_str = std::to_string(zone) + (northp ? "N" : "S");

        RCLCPP_INFO(
            this->get_logger(),
            "GPS received. Latitude: %.6f, Longitude: %.6f, Altitude: %.2f",
            lat, lon, alt
        );
        RCLCPP_INFO(
            this->get_logger(),
            "Converted to UTM: X=%.2f, Y=%.2f, Zone=%s",
            x, y, utm_zone_str.c_str()
        );

        // (4) 퍼블리시
        publisher_utm_->publish(utm_msg);
    }

    void publish_status()
    {
        auto message = robot_custom_interfaces::msg::Status();
        message.id = static_cast<int32_t>(robot_num); 
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
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_gps_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_heading_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_utm_;
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
