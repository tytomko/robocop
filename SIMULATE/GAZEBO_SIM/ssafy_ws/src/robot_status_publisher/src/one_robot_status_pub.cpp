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
#include <cmath>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"

// Custom message, service
#include "robot_custom_interfaces/msg/status.hpp"
#include "robot_custom_interfaces/srv/estop.hpp"

// 추가된 서비스 헤더 파일들
#include "robot_custom_interfaces/srv/homing.hpp"
#include "robot_custom_interfaces/srv/navigate.hpp"
#include "robot_custom_interfaces/srv/patrol.hpp"
#include "robot_custom_interfaces/srv/waiting.hpp"

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
        std::string waiting_service = "/robot_" + std::to_string(robot_num) + "/waiting";

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

        waiting_srv = this->create_service<robot_custom_interfaces::srv::Waiting>(
            waiting_service, std::bind(&RobotStatusPublisher::waiting_service_callback, this, std::placeholders::_1, std::placeholders::_2));

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
    rclcpp::Service<robot_custom_interfaces::srv::Waiting>::SharedPtr waiting_srv;
    rclcpp::TimerBase::SharedPtr status_timer_;

    /// -π ~ π 범위로 정규화하는 함수
    static double normalizeAngle(double angle) {
        while (angle >  M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf2::Matrix3x3 m(q);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // 1) yaw를 우선 -π ~ π로 정규화
        yaw = normalizeAngle(yaw);

        // 2) UTM 좌표계 기준으로 맞추기 위해 오프셋 적용 (여기서는 -π 예시)
        constexpr double OFFSET = -M_PI;
        yaw = normalizeAngle(yaw + OFFSET);

        // 3) 로우패스 필터 (각도 차이 방식으로 필터링하는 것을 권장)
        //    - 간단히 기존 방식 유지한다면:
        // gazebo imu의 yaw값은 서쪽이 0rad 동쪽이 +-3.14rad, 북쪽이 -1.57rad, 남쪽이 1.57rad
        static double filtered_yaw = yaw;  // 초기화
        float alpha = 0.95;
        // (단순 방식) filtered_yaw = alpha * filtered_yaw + (1 - alpha) * yaw;

        // (권장 방식) 각도 차이를 -π~π로 정규화하여 필터링
        double diff = normalizeAngle(yaw - filtered_yaw);
        double adjusted = (1.0 - alpha) * diff;
        filtered_yaw = normalizeAngle(filtered_yaw + adjusted);

        // 4) 메시지에 담아 퍼블리시
        auto heading_msg = std_msgs::msg::Float32();
        heading_msg.data = filtered_yaw;
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

        // 배터리 감소 로직 (1분마다 1% 감소)
        if (std::chrono::duration_cast<std::chrono::minutes>(now - last_battery_update_time_).count() >= 1) {
            if (status_message.battery > 0.0f) {
                status_message.battery -= 1.0f;
            }
            last_battery_update_time_ = now;
        }

        // 랜덤 엔진 생성 (static으로 유지하여 계속 같은 시드 사용)
        static std::random_device rd;
        static std::mt19937 gen(rd());

        // 온도 및 네트워크 상태 초기값
        static float temperature = 55.0f;
        static float network = 100.0f;
        static float network_trend = 0.0f;

        // 온도 변화: 작은 변동 추가 (±0.5°C)
        std::normal_distribution<float> temp_noise(0.0f, 0.3f); // 표준 편차 0.3
        temperature += temp_noise(gen);
        temperature = std::clamp(temperature, 50.0f, 70.0f); // 현실적인 온도 범위 유지
        temperature = std::round(temperature * 1000.0f) / 1000.0f; // 소수점 3번째 자리까지 반올림

        // 네트워크 상태 변화: 랜덤 변동 추가
        network_trend += 0.1f;
        std::uniform_real_distribution<float> network_noise(-5.0f, 5.0f); // ±5% 변동
        network = 50.0f + 30.0f * std::sin(network_trend) + network_noise(gen);
        network = std::clamp(network, 60.0f, 100.0f); // 네트워크 신호가 60~100% 사이에서 유지되도록 제한
        network = std::round(network * 1000.0f) / 1000.0f; // 소수점 3번째 자리까지 반올림

        // 상태 메시지 업데이트
        status_message.temperatures = {temperature};
        status_message.network = network;

        // 상태 메시지 발행
        publisher_status_->publish(status_message);
    }

    void stop_service_callback(const std::shared_ptr<robot_custom_interfaces::srv::Estop::Request> request,
                               std::shared_ptr<robot_custom_interfaces::srv::Estop::Response> response)
    {
        status_message.mode = "emergency stop";
        status_message.is_active = false;
        publisher_status_->publish(status_message);
        
        response->success = true;
        response->message = "Robot stopped.";
    }

    void resume_service_callback(const std::shared_ptr<robot_custom_interfaces::srv::Estop::Request> request,
                                 std::shared_ptr<robot_custom_interfaces::srv::Estop::Response> response)
    {
        if (status_message.mode == "emergency stop") {
            RCLCPP_INFO(this->get_logger(), "[RESUME] Returning to operational mode.");
            
            status_message.mode = "waiting";
            status_message.is_active = true;
            publisher_status_->publish(status_message);
            
            response->success = true;
            response->message = "Robot is waiting.";
        } else {
            RCLCPP_WARN(this->get_logger(), "[RESUME] Robot is already operational.");
            response->success = false;
            response->message = "Robot is already operational.";
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

    void waiting_service_callback(const std::shared_ptr<robot_custom_interfaces::srv::Waiting::Request> request,
                                  std::shared_ptr<robot_custom_interfaces::srv::Waiting::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "[WAITING] Switching to waiting mode.");
        status_message.mode = "waiting";
        publisher_status_->publish(status_message);
        response->success = true;
        response->message = "Robot is waiting.";
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotStatusPublisher>());
    rclcpp::shutdown();
    return 0;
}
