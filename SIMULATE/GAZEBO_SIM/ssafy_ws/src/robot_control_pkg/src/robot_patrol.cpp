#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <limits>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sstream>
#include <string>
#include <algorithm>
#include <nav_msgs/msg/path.hpp>
#include "robot_custom_interfaces/msg/status.hpp"
// 목적지 도착 시 상태를 waiting으로 바꾸는 서비스
#include "robot_custom_interfaces/srv/waiting.hpp"
// 목적지 도착 시 정지상태로 변경하는 서비스
#include "robot_custom_interfaces/srv/estop.hpp"

#include <queue>

// ─── 파라미터들 ─────────────────────────────────────────────────────────────
const double LOOKAHEAD_DISTANCE       = 1.0;  // Lookahead 거리 (m)
const double MAX_LINEAR_SPEED         = 3.0;  // 최대 선속도 (m/s)
const double MAX_ANGULAR_SPEED        = 1.3;  // 최대 각속도 (rad/s)
const double ACCEL_STEP               = 0.3;  // 선속도 가속도 계수
const double ANGLE_STEP               = 0.3;  // 각속도 가속도 계수
const double FRICTION_FACTOR_LINEAR   = 0.9;  // 선속도 감쇠 계수
const double FRICTION_FACTOR_ANGULAR  = 0.8;  // 각속도 감쇠 계수
const double POSITION_TOLERANCE       = 0.1;  // 목표점 도달 허용 오차 (m)
const double ANGLE_ERROR_THRESHOLD    = 0.05; // 각 오차 임계값 (rad)
const double heading_threshold_       = M_PI / 3;  // 목표 각도 오차 임계값 (rad)
// (중간 지점 스킵용) 이미 지나간 지점이라고 간주할 거리 기준
const double SKIP_THRESHOLD = 0.2;
// 로그 출력 여부
const bool log_print = false;  // 로그 출력 여부

struct Point {
    double x, y, z;
};

enum class PatrolState { NONE, APPROACH, PATROL_FORWARD, PATROL_REVERSE };

class PurePursuitNode : public rclcpp::Node {
public:
    PurePursuitNode()
      : Node("pure_pursuit_node"),
        current_heading_(0.0),
        linear_vel_(0.0),
        angular_vel_(0.0),
        current_mode_("operational"),
        patrol_state_(PatrolState::NONE),
        before_patrol_state_(PatrolState::NONE)  // 추가: 일시정지 전 patrol 상태 저장 변수
    {
        // ─── 파라미터 선언 ─────────────────────────────────────────────────
        this->declare_parameter<int>("robot_number", 1);
        this->declare_parameter<std::string>("robot_name", "not_defined");
        my_robot_number_ = this->get_parameter("robot_number").as_int();
        my_robot_name_   = this->get_parameter("robot_name").as_string();

        // ─── 토픽 이름 설정 ───────────────────────────────────────────────
        std::string pose_topic          = "/robot_" + std::to_string(my_robot_number_) + "/utm_pose";
        std::string heading_topic       = "/robot_" + std::to_string(my_robot_number_) + "/heading";
        std::string cmd_vel_topic       = "/"       + my_robot_name_ + "/cmd_vel";
        std::string status_topic        = "/robot_" + std::to_string(my_robot_number_) + "/status";
        std::string global_path_topic   = "/robot_" + std::to_string(my_robot_number_) + "/global_path";
        std::string approach_path_topic = "/robot_" + std::to_string(my_robot_number_) + "/approach_path";
        std::string target_point_topic  = "/robot_" + std::to_string(my_robot_number_) + "/target_point";
        std::string target_heading_topic = "/robot_" + std::to_string(my_robot_number_) + "/target_heading";
        waiting_service_name_ = "/robot_" + std::to_string(my_robot_number_) + "/waiting"; // waiting service 토픽 설정
        temp_stop_service_name_ = "/robot_" + std::to_string(my_robot_number_) + "/temp_stop"; // temp stop service 토픽 설정
        // ─── 구독자 생성 ─────────────────────────────────────────────────
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic, 10,
            std::bind(&PurePursuitNode::pose_callback, this, std::placeholders::_1));

        heading_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            heading_topic, 10,
            std::bind(&PurePursuitNode::heading_callback, this, std::placeholders::_1));

        status_sub_ = this->create_subscription<robot_custom_interfaces::msg::Status>(
            status_topic, 10,
            std::bind(&PurePursuitNode::status_callback, this, std::placeholders::_1));

        global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            global_path_topic, 10,
            std::bind(&PurePursuitNode::global_path_callback, this, std::placeholders::_1));

        approach_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            approach_path_topic, 10,
            std::bind(&PurePursuitNode::approach_path_callback, this, std::placeholders::_1));
        // ─── 퍼블리셔 생성 ───────────────────────────────────────────────
        cmd_vel_pub_         = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
        target_point_pub_    = this->create_publisher<geometry_msgs::msg::Point>(target_point_topic, 10);
        target_heading_pub_  = this->create_publisher<std_msgs::msg::Float32>(target_heading_topic, 10);
    }

private:
    // ─── 멤버 변수들 ─────────────────────────────────────────────────────
    // 구독자 & 퍼블리셔
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr heading_sub_;
    rclcpp::Subscription<robot_custom_interfaces::msg::Status>::SharedPtr status_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr approach_path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_point_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_heading_pub_;
    // waiting service는 client로 호출할 예정이므로 별도 생성

    int my_robot_number_;
    std::string my_robot_name_;
    // 다른 함수에서 사용하기에 멤버 변수로 선언
    std::string waiting_service_name_; // waiting service 토픽 이름
    std::string temp_stop_service_name_; // temp stop service 토픽 이름
    // ─────────────
    // ※ 여기서 핵심: "인덱스+벡터" 대신 "큐"로 경로를 관리
    // ─────────────
    std::queue<Point> path_queue_; 
    std::queue<Point> approach_path_queue_;  // 추가: Approach path를 위한 큐
    std::queue<Point> save_path_queue_;  // 추가: Global path 저장용 큐
    // 현재 로봇 상태
    Point current_position_{0.0, 0.0, 0.0};
    double current_heading_;
    double linear_vel_;
    double angular_vel_;
    // 현재 로봇 상태 외에 접근 경로의 종료점을 저장하는 변수 추가
    Point approach_end_point_{0.0, 0.0, 0.0};

    // 현재 모드 및 순찰 상태
    std::string current_mode_;
    PatrolState patrol_state_;
    PatrolState before_patrol_state_;  // 추가: 일시정지 전 patrol 상태 저장

    bool global_path_received_ = false; // 글로벌 경로 수신 여부
    // ─────────────────────────────────────────────────────────────────────
    // 상태, 콜백, 경로 관리, 제어 함수들
    // ─────────────────────────────────────────────────────────────────────

    // -------------------------------
    // 1) status 콜백
    // -------------------------------
    void status_callback(const robot_custom_interfaces::msg::Status::SharedPtr msg) {
        // 만약 이전 모드가 temp stop였다면, resume 후에 이전 patrol 상태 복원
        if (current_mode_ == "temp stop" && msg->mode != "temp stop") {
            // temp stop 전 저장했던 patrol 상태를 복원 (만약 patrol 모드였다면)
            if (msg->mode == "patrol") {
                patrol_state_ = before_patrol_state_;
                RCLCPP_INFO(this->get_logger(), "Temp stop 해제 후 patrol 상태 복원");
            }
            else if (msg->mode == "navigate") {
                patrol_state_ = before_patrol_state_;
                RCLCPP_INFO(this->get_logger(), "Temp stop 해제 후 patrol 상태 복원");
            }
            else if (msg->mode == "homing") {
                patrol_state_ = before_patrol_state_;
                RCLCPP_INFO(this->get_logger(), "Temp stop 해제 후 patrol 상태 복원");
            }
        }
        current_mode_ = msg->mode;
        if (current_mode_ == "patrol") {
            // patrol 모드로 전환되었는데 patrol_state_가 NONE이면 기본 APPROACH로 시작
            if (patrol_state_ == PatrolState::NONE) {
                patrol_state_ = PatrolState::APPROACH;
                RCLCPP_INFO(this->get_logger(), "Patrol 모드: APPROACH 상태 시작.");
            }
        } 
        else if (current_mode_ == "temp stop") {
            // temp stop 진입 전 patrol 상태 저장 (나중에 resume 시 복원)
            before_patrol_state_ = patrol_state_;
            stop_robot();
        }
        else if (current_mode_ == "emergency stop") {
            // 비상정지 모드로 전환되었을 때는 path_queue_를 비움
            path_queue_ = std::queue<Point>();
            approach_path_queue_ = std::queue<Point>();
            save_path_queue_ = std::queue<Point>();
            // 다른 모드로 전환되었을 때는 patrol_state_를 NONE으로 초기화
            patrol_state_ = PatrolState::NONE;
            // 비상정지 모드로 전환되었을 때는 로봇 정지
            stop_robot();
        }
        else {
            // 다른 모드로 전환되었을 때는 patrol_state_를 NONE으로 초기화
            patrol_state_ = PatrolState::NONE;
        }
    }

    // -------------------------------
    // 2) global_path 콜백
    // -------------------------------
    // 글로벌 경로 수신 콜백 함수
    void global_path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        // 기존 저장된 경로 비움
        while (!save_path_queue_.empty()) {
            save_path_queue_.pop();
            global_path_received_ = false;
        }
        while (!path_queue_.empty()) {
            path_queue_.pop();
            global_path_received_ = false;
        }

        for (auto &ps : msg->poses) {
            Point p;
            p.x = ps.pose.position.x;
            p.y = ps.pose.position.y;
            p.z = ps.pose.position.z;
            save_path_queue_.push(p);
            global_path_received_ = true;
        }
        // 최초 순찰 시작 시에는 글로벌 경로 원본을 그대로 사용
        path_queue_ = save_path_queue_;

        RCLCPP_INFO(this->get_logger(),
            "Global path 업데이트: 큐에 %zu개 노드 저장", msg->poses.size());
    }

    // -------------------------------
    // 3) approach_path 콜백 (patrol-APPROACH 전용)
    // -------------------------------
    void approach_path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (current_mode_ == "patrol" && patrol_state_ == PatrolState::APPROACH) {
            // 기존 approach path 큐를 비우고 새 경로 저장
            while (!approach_path_queue_.empty()) {
                approach_path_queue_.pop();
                global_path_received_ = false;
            }

            for (auto & ps : msg->poses) {
                Point p;
                p.x = ps.pose.position.x;
                p.y = ps.pose.position.y;
                p.z = ps.pose.position.z;
                approach_path_queue_.push(p);
                global_path_received_ = true;
            }
            RCLCPP_INFO(this->get_logger(),
                "Approach path 업데이트: 큐에 %zu개 노드 저장", msg->poses.size());
        }
    }

    // -------------------------------
    // 4) pose, heading 콜백
    // -------------------------------
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_position_.x = msg->pose.position.x;
        current_position_.y = msg->pose.position.y;
        current_position_.z = msg->pose.position.z;
        follow_path();  // 위치 갱신될 때마다 경로 추종 시도
    }

    void heading_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        current_heading_ = msg->data;
    }

    // -------------------------------
    // 5) 경로 추종 핵심 로직 (queue 이용)
    // -------------------------------
    bool follow_current_path() {
        // 사용할 큐 선택 (APPROACH 상태이면 approach_path_queue_, 아니면 global_path_queue_)
        std::queue<Point>* current_queue;
        if (current_mode_ == "patrol" && patrol_state_ == PatrolState::APPROACH) {
            current_queue = &approach_path_queue_;
        } else {
            current_queue = &path_queue_;
        }

        // 경로가 비었으면 false
        if (current_queue->empty()) {
            RCLCPP_WARN(this->get_logger(), "현재 경로(선택된 큐)가 비어 있습니다.");
            return false;
        }

        // ── (1) 이미 지나친 노드 스킵 ─────────────
        while (!current_queue->empty()) {
            const Point &front_node = current_queue->front();
            double dist = std::hypot(front_node.x - current_position_.x,
                                    front_node.y - current_position_.y);
            if (dist < SKIP_THRESHOLD) {
                RCLCPP_INFO(this->get_logger(),
                    "Skip node (%.2f, %.2f) dist=%.2f", front_node.x, front_node.y, dist);
                current_queue->pop();
            } else {
                break;
            }
        }

        if (current_queue->empty()) {
            RCLCPP_INFO(this->get_logger(), "선택된 큐의 노드를 모두 스킵 → 경로 끝");
            return true;
        }

        // ── (2) 큐 front 노드를 목표로 Pure Pursuit 제어 ─────────────
        const Point &target = current_queue->front();
        double distance = std::hypot(target.x - current_position_.x,
                                    target.y - current_position_.y);

        double target_angle = 0.0;
        if (distance > 0.01) {
            target_angle = std::atan2(target.y - current_position_.y,
                                    target.x - current_position_.x);
        }
        double angle_error = normalize_angle(target_angle - current_heading_);

        // 각속도 제어
        if (std::fabs(angle_error) < ANGLE_ERROR_THRESHOLD) {
            angular_vel_ = 0.0;
        } else {
            angular_vel_ += ANGLE_STEP * angle_error;
            angular_vel_ = std::clamp(angular_vel_, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
            angular_vel_ *= FRICTION_FACTOR_ANGULAR;
        }

        // 선속도 제어
        double target_speed = std::min(distance, MAX_LINEAR_SPEED);
        linear_vel_ += ACCEL_STEP * (target_speed - linear_vel_);
        linear_vel_ = std::clamp(linear_vel_, 0.0, MAX_LINEAR_SPEED);
        linear_vel_ *= FRICTION_FACTOR_LINEAR;

        if (std::fabs(angle_error) > heading_threshold_) {
            linear_vel_ = 0.0;
        }

        // cmd_vel 퍼블리시
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x  = linear_vel_;
        cmd_vel_msg.angular.z = angular_vel_;
        cmd_vel_pub_->publish(cmd_vel_msg);

        // 디버그용 target point & heading 퍼블리시
        geometry_msgs::msg::Point target_point_msg;
        target_point_msg.x = target.x;
        target_point_msg.y = target.y;
        target_point_msg.z = target.z;
        target_point_pub_->publish(target_point_msg);

        std_msgs::msg::Float32 target_heading_msg;
        target_heading_msg.data = static_cast<float>(target_angle);
        target_heading_pub_->publish(target_heading_msg);

        if(log_print){
            // 로그 출력
            RCLCPP_INFO(this->get_logger(),
                "\n🚨🚨🚨\nTarget: (%.2f, %.2f)\nDistance: %.2f\nTargetAngle: %.2f\nHeading: %.2f"
                "\nAngleError: %.2f\nLinearVel: %.2f\nAngularVel: %.2f\n🚨🚨🚨\n",
                target.x, target.y, distance, target_angle, current_heading_,
                angle_error, linear_vel_, angular_vel_);
        }
        
        // 목표점 도달 시 pop
        if (distance < POSITION_TOLERANCE) {
            RCLCPP_INFO(this->get_logger(),
                "목표점 (%.2f, %.2f)에 도달 → pop", target.x, target.y);
            current_queue->pop();

            if (current_queue->empty()) {
                return true;
            }
        }
        return false;
    }

    // -------------------------------
    // 6) 상태(mode)에 따른 로봇 행동
    // -------------------------------
    void follow_path() {
        // ── Emergency Stop ───────────────────────────────────────────────
        if (current_mode_ == "emergency stop") {
            stop_robot();
            return;
        }

        // ── homing, navigate ───────────────────────────────────────────
        if (current_mode_ == "homing" || current_mode_ == "navigate") {
            // 전역 경로를 한 번이라도 받은 적이 없다면 -> 실제 경로 정보 없음
            if (!global_path_received_) {
                RCLCPP_WARN(this->get_logger(), "Global path을 수신하지 못했습니다.");
                //0215 이부분 때문에 경로를 받기 전에 멈춰버린다. 현재 문제인부분임..
                //일단 정지를 안하게해서 해결해보자.
                
                //callTempStop();  // temp stop service 호출
                //stop_robot();
                //callWaitingService();
                return;
            }

            // 전역 경로가 수신된 상태라면, 경로를 따라가면서 도착 여부를 확인
            bool reached = follow_current_path();
            if (reached) {
                RCLCPP_INFO(this->get_logger(),
                    "경로에 도달했습니다. Waiting service 호출하여 대기모드로 전환.");
                callTempStop();  // temp stop service 호출
                stop_robot();
                callWaitingService();
                return;
            }
            // 도착하지 않은 경우 계속 진행 (여기서 추가 작업이 있을 수 있음)
            return;
        }

        // ── 순찰 모드 ────────────────────────────────────────────────────
        if (current_mode_ == "patrol") {
            if (patrol_state_ == PatrolState::APPROACH) {
                if (approach_path_queue_.empty()) {
                    stop_robot();
                    if (!path_queue_.empty()) {
                        RCLCPP_INFO(this->get_logger(), "Approach path가 비어 있음 → Global path가 존재하므로 PATROL_FORWARD 전환.");
                        approach_end_point_ = current_position_;
                        trimGlobalPathQueueToClosestPoint(approach_end_point_);
                        patrol_state_ = PatrolState::PATROL_FORWARD;
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Approach path와 Global path 모두 비어 있음. 로봇 정지.");
                        stop_robot();
                    }
                } else {
                    bool reached = follow_current_path();
                    if (reached) {
                        RCLCPP_INFO(this->get_logger(), "Approach path 도착. Global path로 순찰 시작.");
                        approach_end_point_ = current_position_;
                        trimGlobalPathQueueToClosestPoint(approach_end_point_);
                        patrol_state_ = PatrolState::PATROL_FORWARD;
                    }
                }
            }
            else if (patrol_state_ == PatrolState::PATROL_FORWARD) {
                if (path_queue_.empty()) {
                    RCLCPP_WARN(this->get_logger(), "Global patrol path가 없습니다(큐 비어있음).");
                    return;
                }
                bool reached = follow_current_path();
                if (reached) {
                    RCLCPP_INFO(this->get_logger(), "순찰 경로 끝 도달. 역방향 순찰 시작.");
                    construct_reverse_path_queue();  // 정방향에서 역방향 전환
                    patrol_state_ = PatrolState::PATROL_REVERSE;
                }
            }
            else if (patrol_state_ == PatrolState::PATROL_REVERSE) {
                if (path_queue_.empty()) {
                    RCLCPP_WARN(this->get_logger(), "역순찰 경로가 없습니다(큐 비어있음).");
                    return;
                }
                bool reached = follow_current_path();
                if (reached) {
                    RCLCPP_INFO(this->get_logger(), "역순찰 경로 끝 도달. 다시 순방향 순찰 시작.");
                    construct_forward_path_queue();  // 역방향에서 정방향 전환 시, 현재 위치 기준으로 재구성
                    patrol_state_ = PatrolState::PATROL_FORWARD;
                }
            }
            return;
        }

        else if (current_mode_ == "temp stop") {
            // 일시 정지일 때는 로봇 정지 (경로 큐와 patrol 상태는 그대로 유지)
            stop_robot();
            return;
        }
        //전혀 다른 모드일 때도 무조건 로봇 정지
        else if (current_mode_ == "manual") {
            // 수동모드일 때는 로봇 정지안함
            //stop_robot();
            return;
        }
        else {
            // 다른 모드일 때는 로봇 정지
            stop_robot();
            return;
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // 경로 관리 함수들
    // ─────────────────────────────────────────────────────────────────────
    // global path의 중간으로 approach path를 통해 들어왔을때
    // 가야하는 global path를 재정의
    void trimGlobalPathQueueToClosestPoint(const Point &startPoint) {
        if (path_queue_.empty()) return;

        // 큐 크기를 미리 얻어 벡터에 reserve 적용
        const size_t queueSize = path_queue_.size();
        std::vector<Point> globalPath;
        //vector에 큐 크기만큼 할당
        //reserve는 할당만 하고 초기화는 안함
        //resize는 할당하고 초기화까지 함
        globalPath.reserve(queueSize);
        
        // 큐의 모든 요소를 vector로 이동
        while (!path_queue_.empty()) {
            globalPath.push_back(path_queue_.front());
            path_queue_.pop();
        }

        // startPoint와의 제곱 거리가 가장 짧은 노드의 인덱스 찾기
        size_t closestIndex = 0;
        double minDistSq = std::numeric_limits<double>::max();
        for (size_t i = 0; i < globalPath.size(); ++i) {
            double dx = globalPath[i].x - startPoint.x;
            double dy = globalPath[i].y - startPoint.y;
            double distSq = dx * dx + dy * dy;
            if (distSq < minDistSq) {
                minDistSq = distSq;
                closestIndex = i;
            }
        }
        // closestIndex부터 vector의 나머지 경로를 다시 큐에 저장
        for (size_t i = closestIndex; i < globalPath.size(); ++i) {
            path_queue_.push(globalPath[i]);
        }
        RCLCPP_INFO(this->get_logger(), 
            "Global path 재구성: 기준점으로부터 인덱스 %zu (거리 %.2f m) 부터 재구성", 
            closestIndex, std::sqrt(minDistSq));
    }
    // ─────────────────────────────────────────────────────────────────────
    // 보조 함수들
    // ─────────────────────────────────────────────────────────────────────

    // 로봇 정지
    void stop_robot() {

        geometry_msgs::msg::Twist stop_msg;
        stop_msg.linear.x  = 0.0;
        stop_msg.angular.z = 0.0;
        cmd_vel_pub_->publish(stop_msg);

        linear_vel_ = 0.0;
        angular_vel_ = 0.0;

        // static 변수로 마지막 로그 출력 시점을 저장 (steady_clock 사용)
        static auto last_log_time = std::chrono::steady_clock::now();
        auto current_time = std::chrono::steady_clock::now();

        // 1초 간격으로 로그 출력 (간격은 원하는 값으로 변경 가능)
        if (std::chrono::duration_cast<std::chrono::seconds>(current_time - last_log_time).count() >= 1) {
            //RCLCPP_INFO(this->get_logger(), "로봇 정지");
            last_log_time = current_time;
        }
    }

    // waiting service 호출 함수 (대기모드 전환)
    // 목적지 도착하거나 길이 없으면 대기모드로 직접전환해야함.
    void callWaitingService() {
        auto client = this->create_client<robot_custom_interfaces::srv::Waiting>(waiting_service_name_);
        // 서비스가 응답할 때까지 반복 시도
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Waiting 서비스가 응답하지 않습니다. 로봇 정지 후 재시도.");
            stop_robot();
            rclcpp::sleep_for(std::chrono::seconds(1));  // 1초 대기 후 재시도
        }
        
        auto request = std::make_shared<robot_custom_interfaces::srv::Waiting::Request>();
        client->async_send_request(request,
            [this](rclcpp::Client<robot_custom_interfaces::srv::Waiting>::SharedFuture future) {
                try {
                    auto response = future.get();
                    RCLCPP_INFO(this->get_logger(), "Waiting 서비스 호출 성공");
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Waiting 서비스 호출 실패: %s", e.what());
                }
            }
        );
    }
    
    void callTempStop() {
        // robot_custom_interfaces::srv::Estop 타입으로 클라이언트를 생성
        auto client = this->create_client<robot_custom_interfaces::srv::Estop>(temp_stop_service_name_);
        
        // 서비스가 응답할 때까지 반복 시도
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Temp_stop 서비스가 응답하지 않습니다. 로봇 정지 후 재시도.");
            stop_robot();
            rclcpp::sleep_for(std::chrono::seconds(1));  // 1초 대기 후 재시도
        }
        
        // 요청 타입도 Estop으로 변경
        auto request = std::make_shared<robot_custom_interfaces::srv::Estop::Request>();
        client->async_send_request(request,
            [this](rclcpp::Client<robot_custom_interfaces::srv::Estop>::SharedFuture future) {
                try {
                    auto response = future.get();
                    RCLCPP_INFO(this->get_logger(), "Temp_stop 서비스 호출 성공");
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Temp_stop 서비스 호출 실패: %s", e.what());
                }
            }
        );
    }

    // 큐 비우기 (경로 갱신 시 사용)
    void clear_path_queue() {
        while (!path_queue_.empty()) {
            path_queue_.pop();
        }
    }

    // 역방향 순찰을 위한 경로 재구성 함수
    void construct_reverse_path_queue() {
        std::queue<Point> new_queue;
        std::vector<Point> temp_vec;
        std::queue<Point> temp = save_path_queue_;  // 항상 최신의 글로벌 원본 사용
        while (!temp.empty()) {
            temp_vec.push_back(temp.front());
            temp.pop();
        }
        std::reverse(temp_vec.begin(), temp_vec.end());
        for (auto &p : temp_vec) {
            new_queue.push(p);
        }
        path_queue_ = new_queue;
    }

    // 정방향 순찰을 위한 경로 재구성 함수
    void construct_forward_path_queue() {
        // 글로벌 경로의 원본을 복원 후 현재 위치 기준으로 트림
        path_queue_ = save_path_queue_;
        // 현재 위치(current_position_)에 가장 가까운 지점부터 남도록 트림
        trimGlobalPathQueueToClosestPoint(current_position_);
    }

    // 각도 보정
    double normalize_angle(double angle) {
        while (angle > M_PI)  angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

// ──────────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitNode>());
    rclcpp::shutdown();
    return 0;
}
