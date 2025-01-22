#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sstream>
#include <string>

// 경로 설정
const std::string PATROL_PATH_FILE = "map/patrol_path1.csv";
constexpr double LOOKAHEAD_DISTANCE = 0.5;  // Pure Pursuit 룩어헤드 거리 (단위: m)
constexpr double LINEAR_VELOCITY = 0.2;      // 직선 속도 (단위: m/s)
constexpr double ANGULAR_GAIN = 1.0;         // 각속도 조절 게인
constexpr double TURNING_SPEED = 0.5;        // 회전 속도 (단위: rad/s)
constexpr double TURN_THRESHOLD = 3.14;      // 180도 (단위: rad)

struct Point {
    double x, y, z;
};

class PurePursuitNode : public rclcpp::Node {
public:

    std::string robot_name;
    int robot_num;

    PurePursuitNode() : Node("pure_pursuit_node"), patrol_direction_(1), is_turning_(false) {
        this->declare_parameter<std::string>("robot_name", "robot");
        robot_name = this->get_parameter("robot_name").as_string();

        this->declare_parameter<int>("robot_number", 1);
        robot_num = this->get_parameter("robot_number").as_int();

        std::string pose_topic = "/robot_" + std::to_string(robot_num) + "/utm_pose";
        std::string heading_topic = "/robot_" + std::to_string(robot_num) + "/heading";
        std::string cmd_vel_topic = "/" + robot_name + "/cmd_vel";

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic, 10, std::bind(&PurePursuitNode::poseCallback, this, std::placeholders::_1));

        heading_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            heading_topic, 10, std::bind(&PurePursuitNode::headingCallback, this, std::placeholders::_1));

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

        loadPath(PATROL_PATH_FILE);
        findClosestPoint();
        findFarthestPoint();
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr heading_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    std::vector<Point> path_;
    double current_heading_ = 0.0;
    Point current_position_{0.0, 0.0, 0.0};
    size_t current_target_index_;
    size_t farthest_target_index_;
    int patrol_direction_;
    bool is_turning_;
    double initial_heading_;

    void loadPath(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "경로 파일을 열 수 없습니다: %s", filename.c_str());
            return;
        }
        std::string line;
        bool first_line = true;
        while (std::getline(file, line)) {
            if (first_line) {
                first_line = false;
                continue;  // 첫 번째 라인(헤더) 무시
            }
            std::stringstream ss(line);
            std::string x_str, y_str, z_str;
            std::getline(ss, x_str, ',');
            std::getline(ss, y_str, ',');
            std::getline(ss, z_str, ',');
            path_.emplace_back(Point{std::stod(x_str), std::stod(y_str), std::stod(z_str)});
        }
        file.close();
    }

    void findClosestPoint() {
        double min_distance = std::numeric_limits<double>::max();
        for (size_t i = 0; i < path_.size(); ++i) {
            double dist = std::hypot(path_[i].x - current_position_.x, path_[i].y - current_position_.y);
            if (dist < min_distance) {
                min_distance = dist;
                current_target_index_ = i;
            }
        }
    }

    void findFarthestPoint() {
        double max_distance = 0;
        for (size_t i = 0; i < path_.size(); ++i) {
            double dist = std::hypot(path_[i].x - path_[current_target_index_].x, path_[i].y - path_[current_target_index_].y);
            if (dist > max_distance) {
                max_distance = dist;
                farthest_target_index_ = i;
            }
        }
    }

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_position_.x = msg->pose.position.x;
        current_position_.y = msg->pose.position.y;
        current_position_.z = msg->pose.position.z;

        if (!is_turning_) {
            followPath();
        } else {
            checkTurnCompletion();
        }
    }

    void headingCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        current_heading_ = msg->data;
    }

    void followPath() {
        if (path_.empty()) {
            RCLCPP_WARN(this->get_logger(), "경로가 비어 있습니다.");
            return;
        }

        Point target = path_[current_target_index_];
        double distance = std::hypot(target.x - current_position_.x, target.y - current_position_.y);

        if (distance < LOOKAHEAD_DISTANCE) {
            if (current_target_index_ == farthest_target_index_) {
                startTurn();
            } else {
                current_target_index_ += patrol_direction_;
            }
        }

        double target_angle = atan2(target.y - current_position_.y, target.x - current_position_.x);
        double angle_error = normalizeAngle(target_angle - current_heading_);

        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = LINEAR_VELOCITY;
        cmd_vel_msg.angular.z = ANGULAR_GAIN * angle_error;
        cmd_vel_pub_->publish(cmd_vel_msg);
    }

    void startTurn() {
        is_turning_ = true;
        initial_heading_ = normalizeAngle(current_heading_);
        RCLCPP_INFO(this->get_logger(), "회전 시작: 현재 헤딩 = %.2f rad", initial_heading_);
    }

    void checkTurnCompletion() {
        double heading_difference = normalizeAngle(current_heading_ - initial_heading_);

        if (std::abs(heading_difference) >= M_PI) {  // π 라디안(180도) 확인
            RCLCPP_INFO(this->get_logger(), "회전 완료: 목표 방향 도달");

            // 회전 종료 후 타겟 변경
            is_turning_ = false;
            patrol_direction_ *= -1;
            current_target_index_ = (patrol_direction_ == 1) ? farthest_target_index_ : 0;

            // 정지 명령 발행
            geometry_msgs::msg::Twist stop_cmd;
            stop_cmd.linear.x = 0.0;
            stop_cmd.angular.z = 0.0;
            cmd_vel_pub_->publish(stop_cmd);
        } else {
            // 제자리 회전: 선속도 0, 각속도 유지
            geometry_msgs::msg::Twist cmd_vel_msg;
            cmd_vel_msg.linear.x = 0.0;  // 제자리 회전
            cmd_vel_msg.angular.z = TURNING_SPEED * ((patrol_direction_ == 1) ? 1.0 : -1.0); 
            cmd_vel_pub_->publish(cmd_vel_msg);
        }
    }

    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitNode>());
    rclcpp::shutdown();
    return 0;
}
