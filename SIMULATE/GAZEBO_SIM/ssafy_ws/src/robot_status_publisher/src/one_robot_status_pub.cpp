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
#include "robot_custom_interfaces/srv/manual.hpp"

#include <GeographicLib/UTMUPS.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
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
        status_message.id = robot_num;
        status_message.name = robot_name;
        status_message.battery = 75.0f;  // 초기 배터리 값
        status_message.temperatures = {55.0f};  // 초기 온도 값
        status_message.network = 100.0f;  // 초기 네트워크 상태 값
        status_message.mode = "waiting"; // 초기 모드: 대기
        status_message.is_active = true;

        // 토픽 설정
        std::string imu_topic = "/" + robot_name + "/imu";
        std::string gps_topic = "/" + robot_name + "/gps";
        std::string heading_topic = "/robot_" + std::to_string(robot_num) + "/heading";
        std::string utm_topic = "/robot_" + std::to_string(robot_num) + "/utm_pose";
        std::string status_topic = "/robot_" + std::to_string(robot_num) + "/status";
        std::string stop_service = "/robot_" + std::to_string(robot_num) + "/stop";
        // 일시 정지, 재개 서비스 토픽
        std::string temp_stop_service = "/robot_" + std::to_string(robot_num) + "/temp_stop";
        std::string resume_service = "/robot_" + std::to_string(robot_num) + "/resume";
        // 추가된 서비스 토픽
        std::string homing_service = "/robot_" + std::to_string(robot_num) + "/homing";
        std::string navigate_service = "/robot_" + std::to_string(robot_num) + "/navigate";
        std::string patrol_service = "/robot_" + std::to_string(robot_num) + "/patrol";
        std::string waiting_service = "/robot_" + std::to_string(robot_num) + "/waiting";
        std::string manual_service = "/robot_" + std::to_string(robot_num) + "/manual";

        homing_plan_service = "/robot_" + std::to_string(robot_num) + "/homing_plan";
        navigate_plan_service = "/robot_" + std::to_string(robot_num) + "/navigate_plan";
        patrol_plan_service = "/robot_" + std::to_string(robot_num) + "/patrol_plan";
        RCLCPP_INFO(this->get_logger(),
            "🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨\n Node initialized: robot_name='%s', robot_number=%d, imu_topic='%s', heading_topic='%s', status_topic='%s', gps_topic='%s' \n 🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨\n",
            robot_name.c_str(), robot_num, imu_topic.c_str(), heading_topic.c_str(), status_topic.c_str(), gps_topic.c_str());

        // 구독 및 발행 설정
        subscription_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 10, std::bind(&RobotStatusPublisher::imu_callback, this, std::placeholders::_1));

        subscription_gps_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            gps_topic, 10, std::bind(&RobotStatusPublisher::gps_callback, this, std::placeholders::_1));

        publisher_utm_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            utm_topic, 10);

        publisher_heading_ = this->create_publisher<std_msgs::msg::Float32>(
            heading_topic, 10);

        publisher_status_ = this->create_publisher<robot_custom_interfaces::msg::Status>(
            status_topic, 10);

        // 기존 서비스: Estop 관련
        stop_srv = this->create_service<robot_custom_interfaces::srv::Estop>(
            stop_service, std::bind(&RobotStatusPublisher::stop_service_callback, this, std::placeholders::_1, std::placeholders::_2));

        temp_stop_srv = this->create_service<robot_custom_interfaces::srv::Estop>(
            temp_stop_service, std::bind(&RobotStatusPublisher::temp_stop_service_callback, this, std::placeholders::_1, std::placeholders::_2));

        resume_srv = this->create_service<robot_custom_interfaces::srv::Estop>(
            resume_service, std::bind(&RobotStatusPublisher::resume_service_callback, this, std::placeholders::_1, std::placeholders::_2));

        // 추가된 서비스: Homing, Navigate, Patrol, Waiting, Manual
        homing_srv = this->create_service<robot_custom_interfaces::srv::Homing>(
            homing_service, std::bind(&RobotStatusPublisher::homing_service_callback, this, std::placeholders::_1, std::placeholders::_2));

        navigate_srv = this->create_service<robot_custom_interfaces::srv::Navigate>(
            navigate_service, std::bind(&RobotStatusPublisher::navigate_service_callback, this, std::placeholders::_1, std::placeholders::_2));

        patrol_srv = this->create_service<robot_custom_interfaces::srv::Patrol>(
            patrol_service, std::bind(&RobotStatusPublisher::patrol_service_callback, this, std::placeholders::_1, std::placeholders::_2));

        waiting_srv = this->create_service<robot_custom_interfaces::srv::Waiting>(
            waiting_service, std::bind(&RobotStatusPublisher::waiting_service_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        manual_srv = this->create_service<robot_custom_interfaces::srv::Manual>(
            manual_service, std::bind(&RobotStatusPublisher::manual_service_callback, this, std::placeholders::_1, std::placeholders::_2));

        status_timer_ = this->create_wall_timer(
            200ms, std::bind(&RobotStatusPublisher::publish_status, this));
    }

private:
    std::string robot_name;
    int robot_num;
    robot_custom_interfaces::msg::Status status_message;
    std::string before_mode = "";
    std::chrono::steady_clock::time_point last_imu_log_time_;
    std::chrono::steady_clock::time_point last_gps_log_time_;
    std::chrono::steady_clock::time_point last_status_log_time_;
    std::chrono::steady_clock::time_point last_battery_update_time_;
    std::string homing_plan_service;
    std::string navigate_plan_service;
    std::string patrol_plan_service;
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
    rclcpp::Service<robot_custom_interfaces::srv::Estop>::SharedPtr temp_stop_srv;

    // 추가된 서비스 서버 멤버 변수
    rclcpp::Service<robot_custom_interfaces::srv::Homing>::SharedPtr homing_srv;
    rclcpp::Service<robot_custom_interfaces::srv::Navigate>::SharedPtr navigate_srv;
    rclcpp::Service<robot_custom_interfaces::srv::Patrol>::SharedPtr patrol_srv;
    rclcpp::Service<robot_custom_interfaces::srv::Waiting>::SharedPtr waiting_srv;
    rclcpp::Service<robot_custom_interfaces::srv::Manual>::SharedPtr manual_srv;
    
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

        // 배터리 감소 로직 (1분마다 0.1% 감소)
        if (std::chrono::duration_cast<std::chrono::minutes>(now - last_battery_update_time_).count() >= 1) {
            if (status_message.battery > 0.0) {
                // status_message.battery에서 1.0을 빼고, 소수점 한 자리까지 반올림
                status_message.battery = std::round(int((status_message.battery * 10.0 - 1))) / 10.0;
            }
            last_battery_update_time_ = now;
        }

        // 랜덤 엔진 생성 (static으로 유지하여 계속 같은 시드 사용)
        static std::random_device rd;
        static std::mt19937 gen(rd());

        // 온도 및 네트워크 상태 초기값 (이전 상태를 유지하기 위해 static 사용)
        static float temperature = 55.0f;
        static float network = 100.0f;
        static float network_trend = 0.0f;

        // 온도 변화: 작은 변동 추가 (±0.5°C 정도)
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
        status_message.temperatures = { temperature };
        status_message.network = network;

        // 상태 메시지 발행
        publisher_status_->publish(status_message);
    }

    void stop_service_callback(const std::shared_ptr<robot_custom_interfaces::srv::Estop::Request> request,
                            std::shared_ptr<robot_custom_interfaces::srv::Estop::Response> response)
    {
        (void) request;
        status_message.mode = "emergency stop";
        status_message.is_active = false;
        publisher_status_->publish(status_message);
        
        response->success = true;
        response->message = "Robot stopped.";
    }

    void temp_stop_service_callback(const std::shared_ptr<robot_custom_interfaces::srv::Estop::Request> request,
                                    std::shared_ptr<robot_custom_interfaces::srv::Estop::Response> response)
    {
        (void) request;
        if (status_message.mode == "temp stop") {
            RCLCPP_WARN(this->get_logger(), "[TEMP STOP] Robot is already stopped.");
            response->success = false;
            response->message = "Robot is already stopped.";
        } 
        else if (status_message.mode == "emergency stop") {
            RCLCPP_INFO(this->get_logger(), "[TEMP STOP] Robot is already E-stop.");
            response->success = false;
            response->message = "Robot is already E-stop.";
        }
        else {
            RCLCPP_INFO(this->get_logger(), "[TEMP STOP] Temporarily stopping the robot.");
            // 이전 모드를 저장 후 일시 정지 모드로 변경
            before_mode = status_message.mode;
            status_message.mode = "temp stop";
            status_message.is_active = true;
            publisher_status_->publish(status_message);
            response->success = true;
            response->message = "Robot is temporarily stopped.";
        }
    }

    void resume_service_callback(const std::shared_ptr<robot_custom_interfaces::srv::Estop::Request> request,
                                std::shared_ptr<robot_custom_interfaces::srv::Estop::Response> response)
    {
        (void) request;
        if (status_message.mode == "emergency stop") { // 비상 정지 상태에서는 waiting으로 재개
            RCLCPP_INFO(this->get_logger(), "[RESUME] Returning to operational mode.");
            
            status_message.mode = "waiting";
            status_message.is_active = true;
            publisher_status_->publish(status_message);
            
            response->success = true;
            response->message = "Robot is waiting.";
        } 
        else if (status_message.mode == "temp stop") { // 일시 정지 상태에서는 이전 모드로 복귀
            RCLCPP_INFO(this->get_logger(), "[RESUME] Resuming from temp stop.");
            status_message.mode = before_mode;  // 예: "patrol" 또는 다른 모드
            status_message.is_active = true;
            publisher_status_->publish(status_message);
            
            response->success = true;  // 성공으로 처리 (true)
            response->message = "Robot resumed.";
        }
        else if (status_message.mode == "manual") { // 메뉴얼 모드에서 resume 하면 waiting 모드로 변경
            RCLCPP_WARN(this->get_logger(), "[RESUME] Back to watiing mode from manual mode.");
            status_message.mode = "waiting";
            status_message.is_active = true;
            response->success = true;
            response->message = "resume to waiting mode from manual mode.";
        }
        else {
            RCLCPP_WARN(this->get_logger(), "[RESUME] Robot is already operational.");
            response->success = false;
            response->message = "Robot is already operational.";
        }
    }

    // homing, navigate, patrol 서비스는 현재 로봇이 waiting 상태일 때만 동작하도록 수정

    void homing_service_callback(const std::shared_ptr<robot_custom_interfaces::srv::Homing::Request> request,
                                std::shared_ptr<robot_custom_interfaces::srv::Homing::Response> response)
    {
        (void) request;
        if (status_message.mode != "waiting") {
            RCLCPP_WARN(this->get_logger(), "[HOMING] Cannot switch to homing mode because robot is not in waiting mode.");
            response->success = false;
            response->message = "Homing service is allowed only in waiting mode.";
            return;
        }

        RCLCPP_INFO(this->get_logger(), "[HOMING] Switching to homing mode.");
        status_message.mode = "homing";
        publisher_status_->publish(status_message);
        response->success = true;

        // homing_plan 서비스를 호출
        call_homing_plan_service();
    }

    void navigate_service_callback(const std::shared_ptr<robot_custom_interfaces::srv::Navigate::Request> request,
                                std::shared_ptr<robot_custom_interfaces::srv::Navigate::Response> response)
    {
        if (status_message.mode != "waiting") {
            RCLCPP_WARN(this->get_logger(), "[NAVIGATE] Cannot switch to navigate mode because robot is not in waiting mode.");
            response->success = false;
            response->message = "Navigate service is allowed only in waiting mode.";
            return;
        }

        RCLCPP_INFO(this->get_logger(), "[NAVIGATE] Switching to navigate mode. Goal: x=%.2f, y=%.2f, theta=%.2f", 
                    request->goal.x, request->goal.y, request->goal.theta);
        status_message.mode = "navigate";
        publisher_status_->publish(status_message);
        response->success = true;

        // navigate_plan 서비스를 호출
        call_navigate_plan_service(request->goal);
    }

    void patrol_service_callback(const std::shared_ptr<robot_custom_interfaces::srv::Patrol::Request> request,
                                std::shared_ptr<robot_custom_interfaces::srv::Patrol::Response> response)
    {
        if (status_message.mode != "waiting") {
            RCLCPP_WARN(this->get_logger(), "[PATROL] Cannot switch to patrol mode because robot is not in waiting mode.");
            response->success = false;
            response->message = "Patrol service is allowed only in waiting mode.";
            return;
        }

        std::ostringstream oss;
        oss << "[PATROL] Switching to patrol mode. Goals: ";
        for (const auto & goal : request->goals) {
            oss << "(" << goal.x << ", " << goal.y << ", " << goal.theta << ") ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

        status_message.mode = "patrol";
        publisher_status_->publish(status_message);
        response->success = true;

        // patrol_plan 서비스를 호출
        call_patrol_plan_service(request->goals);
    }

    void waiting_service_callback(const std::shared_ptr<robot_custom_interfaces::srv::Waiting::Request> request,
                                std::shared_ptr<robot_custom_interfaces::srv::Waiting::Response> response)
    {
        (void) request;
        if(status_message.mode == "waiting") {
            RCLCPP_WARN(this->get_logger(), "🚨[WAITING] Robot is already in waiting mode.🚨");
            response->success = false;
            response->message = "Robot is already waiting.";
            return;
        }
        if(status_message.mode == "emergency stop" || status_message.mode == "temp stop") {
            // 비상 정지 또는 일시 정지 상태에서만 대기 모드로 변경 가능
            RCLCPP_INFO(this->get_logger(), "🚨[WAITING] Switching to waiting mode.🚨");
            status_message.mode = "waiting";
            publisher_status_->publish(status_message);
            response->success = true;
            response->message = "Robot is waiting.";
            return;
        }

        RCLCPP_WARN(this->get_logger(), "🚨[WARN] Robot is  %s mode. Only E-stop and temp stop can change waiting mode🚨", status_message.mode.c_str());
        response->success = false;
        response->message = "Robot is already waiting.";
        return;
    }

    void manual_service_callback(const std::shared_ptr<robot_custom_interfaces::srv::Manual::Request> request,
                                std::shared_ptr<robot_custom_interfaces::srv::Manual::Response> response)
    {
        (void) request;
        if (status_message.mode != "waiting") {
            RCLCPP_WARN(this->get_logger(), "[MANUAL] Cannot switch to manual mode because robot is not in waiting mode.");
            response->success = false;
            response->message = "🚨Manual service is allowed only in waiting mode.🚨";
            return;
        }
        RCLCPP_INFO(this->get_logger(), "[MANUAL] Switching to manual mode.");
        status_message.mode = "manual";
        publisher_status_->publish(status_message);
        response->success = true;
    }

    void call_homing_plan_service(){
        auto client = this->create_client<robot_custom_interfaces::srv::Homing>(homing_plan_service);

        // 서비스가 응답할 때까지 반복 시도
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "[HOMING] Waiting for homing plan service...");
            rclcpp::sleep_for(std::chrono::seconds(1));  // 1초 대기 후 재시도
        }
        
        // 요청 객체 생성
        auto request = std::make_shared<robot_custom_interfaces::srv::Homing::Request>();

        // 비동기 요청 전송
        client->async_send_request(request,
            [this](rclcpp::Client<robot_custom_interfaces::srv::Homing>::SharedFuture future) {
                try {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(), "[HOMING] Homing plan executed successfully.");
                    } else {
                        RCLCPP_WARN(this->get_logger(), "[HOMING] Failed to execute homing plan: %s", response->message.c_str());
                    }
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "[HOMING] Exception while calling homing plan service: %s", e.what());
                }
            }
        );
    }

    void call_navigate_plan_service(const geometry_msgs::msg::Pose2D& goal){
        auto client = this->create_client<robot_custom_interfaces::srv::Navigate>(navigate_plan_service);

        // 서비스가 응답할 때까지 반복 시도
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "[NAVIGATE] Waiting for navigate plan service...");
            rclcpp::sleep_for(std::chrono::seconds(1));  // 1초 대기 후 재시도
        }

        // 요청 객체 생성
        auto request = std::make_shared<robot_custom_interfaces::srv::Navigate::Request>();
        request->goal = goal;  // 요청에 목표 좌표 설정

        // 비동기 요청 전송
        client->async_send_request(request,
            [this](rclcpp::Client<robot_custom_interfaces::srv::Navigate>::SharedFuture future) {
                try {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(), "[NAVIGATE] Navigate plan executed successfully.");
                    } else {
                        RCLCPP_WARN(this->get_logger(), "[NAVIGATE] Failed to execute navigate plan: %s", response->message.c_str());
                    }
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "[NAVIGATE] Exception while calling navigate plan service: %s", e.what());
                }
            }
        );
    }

    void call_patrol_plan_service(const std::vector<geometry_msgs::msg::Pose2D>& goals){
        auto client = this->create_client<robot_custom_interfaces::srv::Patrol>(patrol_plan_service);

        // 서비스가 응답할 때까지 반복 시도
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "[PATROL] Waiting for patrol plan service...");
            rclcpp::sleep_for(std::chrono::seconds(1));  // 1초 대기 후 재시도
        }

        // 요청 객체 생성
        auto request = std::make_shared<robot_custom_interfaces::srv::Patrol::Request>();
        request->goals = goals;  // 요청에 목표 좌표 목록 설정

        // 비동기 요청 전송
        client->async_send_request(request,
            [this](rclcpp::Client<robot_custom_interfaces::srv::Patrol>::SharedFuture future) {
                try {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(), "[PATROL] Patrol plan executed successfully.");
                    } else {
                        RCLCPP_WARN(this->get_logger(), "[PATROL] Failed to execute patrol plan: %s", response->message.c_str());
                    }
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "[PATROL] Exception while calling patrol plan service: %s", e.what());
                }
            }
        );
    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotStatusPublisher>());
    rclcpp::shutdown();
    return 0;
}
