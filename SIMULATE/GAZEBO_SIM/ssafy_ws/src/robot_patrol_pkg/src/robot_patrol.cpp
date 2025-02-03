#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sstream>
#include <string>
#include <algorithm>
#include <nav_msgs/msg/path.hpp>
#include "robot_custom_interfaces/msg/status.hpp"

// 터틀봇3에 맞춘 파라미터들 (단위: m, m/s, rad/s)
const double LOOKAHEAD_DISTANCE = 2.5;  // Lookahead 거리 (m)
const double MAX_LINEAR_SPEED = 0.3;    // 최대 선속도 (m/s)
const double MAX_ANGULAR_SPEED = 1.0;   // 최대 각속도 (rad/s)
const double ACCEL_STEP = 0.1;
const double ANGLE_STEP = 0.2;
const double FRICTION_FACTOR_LINEAR = 1.0;
const double FRICTION_FACTOR_ANGULAR = 1.0;

struct Point {
    double x, y, z;
};

enum class PatrolState { NONE, APPROACH, PATROL_FORWARD, PATROL_REVERSE };

class PurePursuitNode : public rclcpp::Node {
public:
    PurePursuitNode()
      : Node("pure_pursuit_node"),
        target_index_(0),
        current_heading_(0.0),
        linear_vel_(0.0),
        angular_vel_(0.0),
        current_mode_("operational"),
        patrol_state_(PatrolState::NONE)
    {
        // 파라미터 선언
        this->declare_parameter<int>("robot_number", 1);
        this->declare_parameter<std::string>("robot_name", "not_defined");

        my_robot_number_ = this->get_parameter("robot_number").as_int();
        my_robot_name_ = this->get_parameter("robot_name").as_string();

        // 토픽 이름 설정
        std::string pose_topic = "/robot_" + std::to_string(my_robot_number_) + "/utm_pose";
        std::string heading_topic = "/robot_" + std::to_string(my_robot_number_) + "/heading";
        std::string cmd_vel_topic = "/" + my_robot_name_ + "/cmd_vel";
        std::string status_topic = "/robot_" + std::to_string(my_robot_number_) + "/status";
        std::string global_path_topic = "/robot_" + std::to_string(my_robot_number_) + "/global_path";
        std::string approach_path_topic = "/robot_" + std::to_string(my_robot_number_) + "/approach_path";

        // 구독자 생성
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic, 10, std::bind(&PurePursuitNode::pose_callback, this, std::placeholders::_1));

        heading_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            heading_topic, 10, std::bind(&PurePursuitNode::heading_callback, this, std::placeholders::_1));

        status_sub_ = this->create_subscription<robot_custom_interfaces::msg::Status>(
            status_topic, 10, std::bind(&PurePursuitNode::status_callback, this, std::placeholders::_1));

        global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            global_path_topic, 10, std::bind(&PurePursuitNode::global_path_callback, this, std::placeholders::_1));

        approach_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            approach_path_topic, 10, std::bind(&PurePursuitNode::approach_path_callback, this, std::placeholders::_1));

        // cmd_vel 퍼블리셔 생성
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    }

private:
    // 구독자 및 퍼블리셔
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr heading_sub_;
    rclcpp::Subscription<robot_custom_interfaces::msg::Status>::SharedPtr status_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr approach_path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    int my_robot_number_;
    std::string my_robot_name_;

    std::vector<Point> current_path_;  // 토픽으로 업데이트되는 경로 (절대 좌표)
    size_t target_index_;
    Point current_position_{0.0, 0.0, 0.0};
    double current_heading_;
    double linear_vel_;
    double angular_vel_;

    std::string current_mode_;
    PatrolState patrol_state_;

    // status 토픽 콜백: 모드를 업데이트하고, patrol 모드이면 상태 초기화
    void status_callback(const robot_custom_interfaces::msg::Status::SharedPtr msg) {
        current_mode_ = msg->mode;
        if (current_mode_ == "patrol") {
            if (patrol_state_ == PatrolState::NONE) {
                patrol_state_ = PatrolState::APPROACH;
                target_index_ = 0;
                RCLCPP_INFO(this->get_logger(), "Patrol 모드: APPROACH 상태 시작.");
            }
        } else {
            patrol_state_ = PatrolState::NONE;
            target_index_ = 0;
        }
    }

    // global_path 토픽 콜백: homing/navigating 또는 patrol 모드(접근 상태가 아닐 때)에서 경로 업데이트
    void global_path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        std::vector<Point> path;
        for (const auto & ps : msg->poses) {
            Point p;
            p.x = ps.pose.position.x;
            p.y = ps.pose.position.y;
            p.z = ps.pose.position.z;
            path.push_back(p);
        }
        if (current_mode_ == "homing" || current_mode_ == "navigating" ||
           (current_mode_ == "patrol" && patrol_state_ != PatrolState::APPROACH)) {
            current_path_ = path;
            target_index_ = 0;
            RCLCPP_INFO(this->get_logger(), "Global path 업데이트: 노드 개수 = %zu", current_path_.size());
        }
    }

    // approach_path 토픽 콜백: patrol 모드에서 APPROACH 상태인 경우 경로 업데이트
    void approach_path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        std::vector<Point> path;
        for (const auto & ps : msg->poses) {
            Point p;
            p.x = ps.pose.position.x;
            p.y = ps.pose.position.y;
            p.z = ps.pose.position.z;
            path.push_back(p);
        }
        if (current_mode_ == "patrol" && patrol_state_ == PatrolState::APPROACH) {
            current_path_ = path;
            target_index_ = 0;
            RCLCPP_INFO(this->get_logger(), "Approach path 업데이트: 노드 개수 = %zu", current_path_.size());
        }
    }

    // Pose 및 Heading 콜백
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_position_.x = msg->pose.position.x;
        current_position_.y = msg->pose.position.y;
        current_position_.z = msg->pose.position.z;
        RCLCPP_INFO(this->get_logger(), "현재 위치: (%.2f, %.2f)", current_position_.x, current_position_.y);
        follow_path();
    }

    void heading_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        current_heading_ = msg->data;
    }

    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    // Lookahead 방식으로 목표점 인덱스를 결정
    size_t find_lookahead_point() {
        if (current_path_.empty())
            return 0;
        for (size_t i = target_index_; i < current_path_.size(); ++i) {
            double dist = std::hypot(current_path_[i].x - current_position_.x,
                                     current_path_[i].y - current_position_.y);
            if (dist >= LOOKAHEAD_DISTANCE) {
                return i;
            }
        }
        // 만약 Lookahead 거리에 해당하는 점이 없으면 마지막 점을 선택
        return current_path_.size() - 1;
    }

    // Pure Pursuit 제어를 수행하여 현재 경로를 따라가고, 경로 끝에 도달하면 true 반환
    bool follow_current_path() {
        if (current_path_.empty()) {
            RCLCPP_WARN(this->get_logger(), "현재 경로가 비어 있습니다.");
            return false;
        }
        // Lookahead 방식으로 목표점 선정
        target_index_ = find_lookahead_point();
        if (target_index_ >= current_path_.size())
            target_index_ = current_path_.size() - 1;
        Point target = current_path_[target_index_];
        double distance = std::hypot(target.x - current_position_.x, target.y - current_position_.y);
        double target_angle = 0.0;
        if (distance > 0.01) {
            target_angle = std::atan2(target.y - current_position_.y, target.x - current_position_.x);
        }
        double angle_error = normalize_angle(target_angle - current_heading_);
        
        // 각도 오차에 따라 선속도 스케일링 (angle_error가 클 경우 선속도를 낮춤)
        double speed_scale = std::max(0.0, std::cos(angle_error)); // cos값이 1~0 사이
        double target_speed = std::min(distance, MAX_LINEAR_SPEED * speed_scale);

        // 가속/감속 적용
        linear_vel_ += ACCEL_STEP * (target_speed - linear_vel_);
        linear_vel_ = std::clamp(linear_vel_, 0.0, MAX_LINEAR_SPEED);
        angular_vel_ += ANGLE_STEP * angle_error;
        angular_vel_ = std::clamp(angular_vel_, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

        // friction factor 제거 (이미 조정했으므로)
        linear_vel_ *= FRICTION_FACTOR_LINEAR;
        angular_vel_ *= FRICTION_FACTOR_ANGULAR;

        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = linear_vel_;
        cmd_vel_msg.angular.z = angular_vel_;
        cmd_vel_pub_->publish(cmd_vel_msg);

        RCLCPP_INFO(this->get_logger(), "Target: (%.2f, %.2f), Distance: %.2f, Angle Error: %.2f",
                    target.x, target.y, distance, angle_error);

        if (distance < LOOKAHEAD_DISTANCE * 0.5) { // 목표점에 충분히 근접하면 다음 점으로
            target_index_++;
            if (target_index_ >= current_path_.size()) {
                return true; // 경로 끝 도달
            }
        }
        return false;
    }

    // 상태(mode)에 따라 로봇의 행동을 결정하는 함수
    void follow_path() {
        if (current_mode_ == "emergency stop") {
            geometry_msgs::msg::Twist stop_msg;
            stop_msg.linear.x = 0.0;
            stop_msg.angular.z = 0.0;
            cmd_vel_pub_->publish(stop_msg);
            RCLCPP_INFO(this->get_logger(), "Emergency stop: 로봇 정지");
            return;
        }
        if (current_mode_ == "homing" || current_mode_ == "navigating") {
            if (current_path_.empty()) {
                RCLCPP_WARN(this->get_logger(), "Global path이 없습니다.");
                return;
            }
            bool reached = follow_current_path();
            if (reached) {
                RCLCPP_INFO(this->get_logger(), "경로에 도달했습니다. 로봇 정지.");
                linear_vel_ = 0.0;
                angular_vel_ = 0.0;
            }
            return;
        }
        if (current_mode_ == "patrol") {
            if (patrol_state_ == PatrolState::APPROACH) {
                if (current_path_.empty()) {
                    RCLCPP_WARN(this->get_logger(), "Approach path가 없습니다.");
                    return;
                }
                bool reached = follow_current_path();
                if (reached) {
                    RCLCPP_INFO(this->get_logger(), "Approach path 도착. 이제 글로벌 path로 순찰 시작.");
                    patrol_state_ = PatrolState::PATROL_FORWARD;
                    target_index_ = 0;
                }
            } else if (patrol_state_ == PatrolState::PATROL_FORWARD) {
                if (current_path_.empty()) {
                    RCLCPP_WARN(this->get_logger(), "Global patrol path가 없습니다.");
                    return;
                }
                bool reached = follow_current_path();
                if (reached) {
                    RCLCPP_INFO(this->get_logger(), "순찰 경로 끝 도달. 역방향 순찰 시작.");
                    std::reverse(current_path_.begin(), current_path_.end());
                    patrol_state_ = PatrolState::PATROL_REVERSE;
                    target_index_ = 0;
                }
            } else if (patrol_state_ == PatrolState::PATROL_REVERSE) {
                if (current_path_.empty()) {
                    RCLCPP_WARN(this->get_logger(), "역순찰 경로가 없습니다.");
                    return;
                }
                bool reached = follow_current_path();
                if (reached) {
                    RCLCPP_INFO(this->get_logger(), "역순찰 경로 끝 도달. 다시 순방향 순찰 시작.");
                    std::reverse(current_path_.begin(), current_path_.end());
                    patrol_state_ = PatrolState::PATROL_FORWARD;
                    target_index_ = 0;
                }
            }
            return;
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitNode>());
    rclcpp::shutdown();
    return 0;
}
