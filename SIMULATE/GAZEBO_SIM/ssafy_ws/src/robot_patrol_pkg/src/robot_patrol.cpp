#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <limits>
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
#include <queue>

// ─── 파라미터들 ─────────────────────────────────────────────────────────────
const double LOOKAHEAD_DISTANCE       = 0.4;  // Lookahead 거리 (m)
const double MAX_LINEAR_SPEED         = 0.3;  // 최대 선속도 (m/s)
const double MAX_ANGULAR_SPEED        = 0.6;  // 최대 각속도 (rad/s)
const double ACCEL_STEP               = 0.1;  // 선속도 가속도 계수
const double ANGLE_STEP               = 0.1;  // 각속도 가속도 계수
const double FRICTION_FACTOR_LINEAR   = 0.9;  // 선속도 감쇠 계수
const double FRICTION_FACTOR_ANGULAR  = 0.8;  // 각속도 감쇠 계수
const double POSITION_TOLERANCE       = 0.1;  // 목표점 도달 허용 오차 (m)
const double ANGLE_ERROR_THRESHOLD    = 0.05; // 각 오차 임계값 (rad)

// (중간 지점 스킵용) 이미 지나간 지점이라고 간주할 거리 기준
// POSITION_TOLERANCE와 동일하게 쓰거나, 약간 크게 설정해도 된다.
const double SKIP_THRESHOLD = 0.2;

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
        patrol_state_(PatrolState::NONE)
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

    int my_robot_number_;
    std::string my_robot_name_;

    // ─────────────
    // ※ 여기서 핵심: "인덱스+벡터" 대신 "큐"로 경로를 관리
    // ─────────────
    std::queue<Point> path_queue_; 

    // 현재 로봇 상태
    Point current_position_{0.0, 0.0, 0.0};
    double current_heading_;
    double linear_vel_;
    double angular_vel_;

    // 현재 모드 및 순찰 상태
    std::string current_mode_;
    PatrolState patrol_state_;

    // ─────────────────────────────────────────────────────────────────────
    // 상태, 콜백, 경로 관리, 제어 함수들
    // ─────────────────────────────────────────────────────────────────────

    // -------------------------------
    // 1) status 콜백
    // -------------------------------
    void status_callback(const robot_custom_interfaces::msg::Status::SharedPtr msg) {
        current_mode_ = msg->mode;
        if (current_mode_ == "patrol") {
            if (patrol_state_ == PatrolState::NONE) {
                patrol_state_ = PatrolState::APPROACH;
                RCLCPP_INFO(this->get_logger(), "Patrol 모드: APPROACH 상태 시작.");
            }
        } else {
            patrol_state_ = PatrolState::NONE;
        }
    }

    // -------------------------------
    // 2) global_path 콜백
    // -------------------------------
    void global_path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        // homing, navigating, 혹은 patrol의 (APPROACH가 아닌) 상태일 때만 글로벌 경로 업데이트
        if (!(current_mode_ == "homing" || current_mode_ == "navigating" ||
              (current_mode_ == "patrol" && patrol_state_ != PatrolState::APPROACH))) {
            return;
        }

        // 이전 큐를 비우고 새 경로로 채움
        clear_path_queue();

        // msg->poses에서 큐로 push
        for (auto & ps : msg->poses) {
            Point p;
            p.x = ps.pose.position.x;
            p.y = ps.pose.position.y;
            p.z = ps.pose.position.z;
            path_queue_.push(p);
        }

        RCLCPP_INFO(this->get_logger(),
            "Global path 업데이트: 큐에 %zu개 노드 저장", msg->poses.size());
    }

    // -------------------------------
    // 3) approach_path 콜백 (patrol-APPROACH 전용)
    // -------------------------------
    void approach_path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (current_mode_ == "patrol" && patrol_state_ == PatrolState::APPROACH) {
            // 이전 큐를 비우고 새 경로로 채움
            clear_path_queue();

            for (auto & ps : msg->poses) {
                Point p;
                p.x = ps.pose.position.x;
                p.y = ps.pose.position.y;
                p.z = ps.pose.position.z;
                path_queue_.push(p);
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
        // 경로가 비었으면 false
        if (path_queue_.empty()) {
            RCLCPP_WARN(this->get_logger(), "현재 경로(큐)가 비어 있습니다.");
            return false;
        }

        // ── (1) 이미 지나친 노드(너무 가까운 노드)는 pop으로 스킵 ─────────────
        while (!path_queue_.empty()) {
            const Point &front_node = path_queue_.front();
            double dist = std::hypot(front_node.x - current_position_.x,
                                     front_node.y - current_position_.y);
            if (dist < SKIP_THRESHOLD) {
                // 스킵
                RCLCPP_INFO(this->get_logger(),
                    "Skip node (%.2f, %.2f) dist=%.2f", front_node.x, front_node.y, dist);
                path_queue_.pop();
            } else {
                // 이제 충분히 멀리있는(가야할) 노드 발견
                break;
            }
        }

        // 스킵하다가 큐가 비었으면 경로 끝
        if (path_queue_.empty()) {
            RCLCPP_INFO(this->get_logger(), "큐 노드를 모두 스킵 → 경로 끝");
            return true;
        }

        // ── (2) 큐 front 노드를 목표로하여 Pure Pursuit 제어 ─────────────
        const Point &target = path_queue_.front();
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

        // ── (3) cmd_vel 퍼블리시 ──────────────────────────────────────────
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x  = linear_vel_;
        cmd_vel_msg.angular.z = angular_vel_;
        cmd_vel_pub_->publish(cmd_vel_msg);

        // ── (4) target point & heading 퍼블리시(디버그용) ─────────────────
        geometry_msgs::msg::Point target_point_msg;
        target_point_msg.x = target.x;
        target_point_msg.y = target.y;
        target_point_msg.z = target.z;
        target_point_pub_->publish(target_point_msg);

        std_msgs::msg::Float32 target_heading_msg;
        target_heading_msg.data = static_cast<float>(target_angle);
        target_heading_pub_->publish(target_heading_msg);

        // ── (5) 로그 출력 ────────────────────────────────────────────────
        RCLCPP_INFO(this->get_logger(),
            "\n🚨🚨🚨\nTarget: (%.2f, %.2f)\nDistance: %.2f\nTargetAngle: %.2f\nHeading: %.2f"
            "\nAngleError: %.2f\nLinearVel: %.2f\nAngularVel: %.2f\n🚨🚨🚨\n",
            target.x, target.y, distance, target_angle, current_heading_,
            angle_error, linear_vel_, angular_vel_);

        // ── (6) 목표점에 가까워졌으면(<= POSITION_TOLERANCE) pop & 다음 노드로
        if (distance < POSITION_TOLERANCE) {
            RCLCPP_INFO(this->get_logger(),
                "목표점 (%.2f, %.2f)에 도달 -> pop", target.x, target.y);
            path_queue_.pop();

            if (path_queue_.empty()) {
                // 마지막 노드까지 도달한 경우
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

        // ── homing, navigating ───────────────────────────────────────────
        if (current_mode_ == "homing" || current_mode_ == "navigating") {
            if (path_queue_.empty()) {
                RCLCPP_WARN(this->get_logger(), "Global path이 없습니다(큐 비어있음).");
                return;
            }
            bool reached = follow_current_path();
            if (reached) {
                // 경로 끝 -> 로봇 정지
                RCLCPP_INFO(this->get_logger(), "경로에 도달했습니다. 로봇 정지.");
                stop_robot();
            }
            return;
        }

        // ── 순찰 모드 ────────────────────────────────────────────────────
        if (current_mode_ == "patrol") {
            if (patrol_state_ == PatrolState::APPROACH) {
                // Approach path
                if (path_queue_.empty()) {
                    RCLCPP_WARN(this->get_logger(), "Approach path가 없습니다(큐 비어있음).");
                    return;
                }
                bool reached = follow_current_path();
                if (reached) {
                    RCLCPP_INFO(this->get_logger(),
                        "Approach path 도착. 이제 글로벌 path로 순찰 시작.");
                    patrol_state_ = PatrolState::PATROL_FORWARD;
                    // 여기서 큐를 비우고, 실제 global_path_callback에서 수신했을 수도 있음
                    // 혹은 이미 global path가 세팅되어 있으면 그것을 사용
                }
            }
            else if (patrol_state_ == PatrolState::PATROL_FORWARD) {
                // Forward 순찰
                if (path_queue_.empty()) {
                    RCLCPP_WARN(this->get_logger(), "Global patrol path가 없습니다(큐 비어있음).");
                    return;
                }
                bool reached = follow_current_path();
                if (reached) {
                    RCLCPP_INFO(this->get_logger(),
                        "순찰 경로 끝 도달. 역방향 순찰 시작.");
                    // ── (중요) 큐 뒤집어서 역순으로 만든다
                    reverse_path_queue();
                    patrol_state_ = PatrolState::PATROL_REVERSE;
                }
            }
            else if (patrol_state_ == PatrolState::PATROL_REVERSE) {
                // Reverse 순찰
                if (path_queue_.empty()) {
                    RCLCPP_WARN(this->get_logger(), "역순찰 경로가 없습니다(큐 비어있음).");
                    return;
                }
                bool reached = follow_current_path();
                if (reached) {
                    RCLCPP_INFO(this->get_logger(),
                        "역순찰 경로 끝 도달. 다시 순방향 순찰 시작.");
                    reverse_path_queue();
                    patrol_state_ = PatrolState::PATROL_FORWARD;
                }
            }
            return;
        }

        // ── 그 외 모드에서는 정지 ─────────────────────────────────────────
        else {
            stop_robot();
        }
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

        RCLCPP_INFO(this->get_logger(), "로봇 정지");
    }

    // 큐 비우기 (경로 갱신 시 사용)
    void clear_path_queue() {
        while (!path_queue_.empty()) {
            path_queue_.pop();
        }
    }

    // 큐를 뒤집는 함수 (역순찰 시 사용)
    void reverse_path_queue() {
        // 1) 큐에서 모든 원소를 꺼내 vector에 저장
        std::vector<Point> temp_vec;
        while (!path_queue_.empty()) {
            temp_vec.push_back(path_queue_.front());
            path_queue_.pop();
        }
        // 2) vector를 뒤집은 뒤, 다시 큐에 push
        std::reverse(temp_vec.begin(), temp_vec.end());
        for (auto & p : temp_vec) {
            path_queue_.push(p);
        }
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
