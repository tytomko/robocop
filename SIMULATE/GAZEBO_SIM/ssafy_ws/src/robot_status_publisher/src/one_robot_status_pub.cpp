// robot_status_publisher 패키지의 노드 구현
// - IMU, GPS 데이터 수신 및 처리
// - UTM 좌표 변환
// - Heading 발행
// - Status 메시지 발행
// - 추가: 호밍, 네비게이트, 패트롤 서비스 처리

#include <chrono>
#include <string>
#include <iomanip>
#include <sstream>
#include <random>
#include "rclcpp/rclcpp.hpp"

// Custom message, service
#include "robot_custom_interfaces/msg/status.hpp"
#include "robot_custom_interfaces/srv/estop.hpp"

// 추가된 서비스 헤더 파일들
#include "robot_custom_interfaces/srv/homing.hpp"
#include "robot_custom_interfaces/srv/navigate.hpp"
#include "robot_custom_interfaces/srv/patrol.hpp"

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
    RobotStatusPublisher()
        : Node("robot_status_pub_node")
    {
        // 1) 파라미터 선언
        this->declare_parameter<std::string>("robot_name", "not_defined");
        this->declare_parameter<int>("robot_number", -1);

        robot_name = this->get_parameter("robot_name").as_string();
        robot_num = this->get_parameter("robot_number").as_int();

        // 시작 시간 설정 (문자열 변환)
        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);
        std::ostringstream oss;
        oss << std::put_time(std::localtime(&time_t_now), "%Y-%m-%d %H:%M:%S");
        status_message.starttime = oss.str(); // 시작 시간 문자열로 저장

        status_message.name = robot_name;
        status_message.battery = 75.0f;  // 초기 배터리 값
        status_message.temperatures = {55.0f};  // 초기 온도 값
        status_message.network = 100.0f;  // 초기 네트워크 상태 값
        status_message.mode = "operational";
        status_message.is_active = true;

        // 토픽 설정
        std::string imu_topic = "/" + robot_name + "/imu";
        std::string gps_topic = "/" + robot_name + "/gps";
        std::string heading_topic = "/robot_" + std::to_string(robot_num) + "/heading";
        std::string status_topic = "/robot_" + std::to_string(robot_num) + "/status";
        std::string stop_service = "/robot_" + std::to_string(robot_num) + "/stop";
        std::string resume_service = "/robot_" + std::to_string(robot_num) + "/resume";
        // 추가된 서비스 토픽
        std::string homing_service = "/robot_" + std::to_string(robot_num) + "/homing";
        std::string navigate_service = "/robot_" + std::to_string(robot_num) + "/navigate";
        std::string patrol_service = "/robot_" + std::to_string(robot_num) + "/patrol";

        RCLCPP_INFO(this->get_logger(),
            "🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨\n Node initialized: robot_name='%s', robot_number=%d, imu_topic='%s', heading_topic='%s', status_topic='%s', gps_topic='%s' \n 🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨\n",
            robot_name.c_str(), robot_num, imu_topic.c_str(), heading_topic.c_str(), status_topic.c_str(), gps_topic.c_str());

        // 구독 및 발행 설정
        subscription_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 10, std::bind(&RobotStatusPublisher::imu_callback, this, std::placeholders::_1));

        subscription_gps_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            gps_topic, 10, std::bind(&RobotStatusPublisher::gps_callback, this, std::placeholders::_1));

        publisher_utm_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/robot_" + std::to_string(robot_num) + "/utm_pose", 10);

        publisher_heading_ = this->create_publisher<std_msgs::msg::Float32>(
            heading_topic, 10);

        publisher_status_ = this->create_publisher<robot_custom_interfaces::msg::Status>(
            status_topic, 10);

        // 기존 서비스: Estop 관련
        stop_srv = this->create_service<robot_custom_interfaces::srv::Estop>(
            stop_service, std::bind(&RobotStatusPublisher::stop_service_callback, this, std::placeholders::_1, std::placeholders::_2));

        resume_srv = this->create_service<robot_custom_interfaces::srv::Estop>(
            resume_service, std::bind(&RobotStatusPublisher::resume_service_callback, this, std::placeholders::_1, std::placeholders::_2));

        // 추가된 서비스: Homing, Navigate, Patrol
        homing_srv = this->create_service<robot_custom_interfaces::srv::Homing>(
            homing_service, std::bind(&RobotStatusPublisher::homing_service_callback, this, std::placeholders::_1, std::placeholders::_2));

        navigate_srv = this->create_service<robot_custom_interfaces::srv::Navigate>(
            navigate_service, std::bind(&RobotStatusPublisher::navigate_service_callback, this, std::placeholders::_1, std::placeholders::_2));

        patrol_srv = this->create_service<robot_custom_interfaces::srv::Patrol>(
            patrol_service, std::bind(&RobotStatusPublisher::patrol_service_callback, this, std::placeholders::_1, std::placeholders::_2));

        status_timer_ = this->create_wall_timer(
            100ms, std::bind(&RobotStatusPublisher::publish_status, this));
    }

private:
    std::string robot_name;
    int robot_num;
    robot_custom_interfaces::msg::Status status_message;

    std::chrono::steady_clock::time_point last_imu_log_time_;
    std::chrono::steady_clock::time_point last_gps_log_time_;
    std::chrono::steady_clock::time_point last_status_log_time_;
    std::chrono::steady_clock::time_point last_battery_update_time_;

    std::random_device rd;
    std::mt19937 gen{rd()};
    std::uniform_real_distribution<float> temp_dist{50.0f, 80.0f};  // 온도 랜덤
    std::uniform_int_distribution<int> network_dist{0, 100};  // 네트워크 상태 랜덤

    // 구독자, 발행자, 서비스 서버, 타이머 멤버 변수
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_gps_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_heading_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_utm_;
    rclcpp::Publisher<robot_custom_interfaces::msg::Status>::SharedPtr publisher_status_;
    rclcpp::Service<robot_custom_interfaces::srv::Estop>::SharedPtr stop_srv;
    rclcpp::Service<robot_custom_interfaces::srv::Estop>::SharedPtr resume_srv;
    // 추가된 서비스 서버 멤버 변수
    rclcpp::Service<robot_custom_interfaces::srv::Homing>::SharedPtr homing_srv;
    rclcpp::Service<robot_custom_interfaces::srv::Navigate>::SharedPtr navigate_srv;
    rclcpp::Service<robot_custom_interfaces::srv::Patrol>::SharedPtr patrol_srv;
    rclcpp::TimerBase::SharedPtr status_timer_;

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // IMU에서 받은 쿼터니언 데이터를 TF2 객체로 변환
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf2::Matrix3x3 m(q);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // Yaw 값을 -π ~ π 범위로 제한 (누적 오차 방지)
        yaw = std::fmod(yaw + M_PI, 2 * M_PI) - M_PI;

        // 로우패스 필터 적용 (0.95의 계수를 적용하여 부드러운 yaw 값 유지)
        static float filtered_yaw = yaw;  // 초기값 설정
        float alpha = 0.95;  // 필터 계수 (0~1, 1에 가까울수록 반응 속도 감소)

        filtered_yaw = alpha * filtered_yaw + (1 - alpha) * yaw;

        auto heading_msg = std_msgs::msg::Float32();
        heading_msg.data = filtered_yaw;

        // 일정 주기마다 로그 출력
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_imu_log_time_).count() >= 2) {
            RCLCPP_INFO(this->get_logger(), "IMU received. Filtered Yaw (heading): %.2f (rad)", filtered_yaw);
            last_imu_log_time_ = now;
        }

        publisher_heading_->publish(heading_msg);
    }

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        double lat = msg->latitude, lon = msg->longitude, alt = msg->altitude;
        int zone;
        bool northp;
        double x, y;

        GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, x, y);

        geometry_msgs::msg::PoseStamped utm_msg;
        utm_msg.header.stamp = msg->header.stamp;
        utm_msg.header.frame_id = "map";
        utm_msg.pose.position.x = x;
        utm_msg.pose.position.y = y;
        utm_msg.pose.position.z = alt;

        publisher_utm_->publish(utm_msg);
    }

    void publish_status()
    {
        auto now = std::chrono::steady_clock::now();

        // 배터리 감소
        if (std::chrono::duration_cast<std::chrono::minutes>(now - last_battery_update_time_).count() >= 1) {
            if (status_message.battery > 0.0f) {
                status_message.battery -= 1.0f;
            }
            last_battery_update_time_ = now;
        }

        // 랜덤 온도 및 네트워크 상태 업데이트
        status_message.temperatures = {temp_dist(gen)};
        status_message.network = static_cast<float>(network_dist(gen));
        
        publisher_status_->publish(status_message);
    }

    void stop_service_callback(const std::shared_ptr<robot_custom_interfaces::srv::Estop::Request> request,
                               std::shared_ptr<robot_custom_interfaces::srv::Estop::Response> response)
    {
        status_message.mode = "emergency stop";
        status_message.is_active = false;
        publisher_status_->publish(status_message);
        
        response->success = true;
    }

    void resume_service_callback(const std::shared_ptr<robot_custom_interfaces::srv::Estop::Request> request,
                                 std::shared_ptr<robot_custom_interfaces::srv::Estop::Response> response)
    {
        if (status_message.mode == "emergency stop") {
            RCLCPP_INFO(this->get_logger(), "[RESUME] Returning to operational mode.");
            
            status_message.mode = "operational";
            status_message.is_active = true;
            publisher_status_->publish(status_message);
            
            response->success = true;
        } else {
            RCLCPP_WARN(this->get_logger(), "[RESUME] Robot is already operational.");
            response->success = false;
        }
    }

    // 추가된 서비스 콜백 함수들

    void homing_service_callback(const std::shared_ptr<robot_custom_interfaces::srv::Homing::Request> request,
                                 std::shared_ptr<robot_custom_interfaces::srv::Homing::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "[HOMING] Switching to homing mode.");
        status_message.mode = "homing";
        publisher_status_->publish(status_message);
        response->success = true;
    }

    void navigate_service_callback(const std::shared_ptr<robot_custom_interfaces::srv::Navigate::Request> request,
                                   std::shared_ptr<robot_custom_interfaces::srv::Navigate::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "[NAVIGATE] Switching to navigating mode. Goal: x=%.2f, y=%.2f, theta=%.2f", 
                    request->goal.x, request->goal.y, request->goal.theta);
        status_message.mode = "navigating";
        publisher_status_->publish(status_message);
        response->success = true;
    }

    void patrol_service_callback(const std::shared_ptr<robot_custom_interfaces::srv::Patrol::Request> request,
                                 std::shared_ptr<robot_custom_interfaces::srv::Patrol::Response> response)
    {
        std::ostringstream oss;
        oss << "[PATROL] Switching to patrolling mode. Goals: ";
        for (const auto & goal : request->goals) {
            oss << "(" << goal.x << ", " << goal.y << ", " << goal.theta << ") ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
        status_message.mode = "patrolling";
        publisher_status_->publish(status_message);
        response->success = true;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotStatusPublisher>());
    rclcpp::shutdown();
    return 0;
}
