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
#include <algorithm>

// 경로 설정
const std::string PATROL_PATH_FILE = "map/patrol_path1.csv";
// 수정해야하는 파라미터
constexpr double LOOKAHEAD_DISTANCE = 1.5;  // 수정된 탐색 거리 미터단위
constexpr double MAX_LINEAR_SPEED = 2.0;    // 수정된 최대 선속도
constexpr double MAX_ANGULAR_SPEED = 1.5;   // 수정된 최대 각속도
constexpr double ACCEL_STEP = 0.05;
// 각변화 수치 계수
constexpr double ANGLE_STEP = 0.10;
// 마찰계수
constexpr double FRICTION_FACTOR_LINEAR = 0.98;
constexpr double FRICTION_FACTOR_ANGULAR = 0.95;

struct Point {
    double x, y, z;
};

class PurePursuitNode : public rclcpp::Node {
public:
    PurePursuitNode() 
        : Node("pure_pursuit_node"), 
          current_heading(0.0),
          linear_vel(0.0), 
          angular_vel(0.0)
    {
        this->declare_parameter<int>("robot_number", 1);
        this->declare_parameter<std::string>("robot_name", "not_defined");

        my_robot_name = this->get_parameter("robot_name").as_string();
        my_robot_number = this->get_parameter("robot_number").as_int();

        std::string pose_topic = "/robot_" + std::to_string(my_robot_number) + "/utm_pose";
        std::string heading_topic = "/robot_" + std::to_string(my_robot_number) + "/heading";
        std::string cmd_vel_topic = "/" + my_robot_name + "/cmd_vel";

        pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic, 10, std::bind(&PurePursuitNode::pose_callback, this, std::placeholders::_1));

        heading_sub = this->create_subscription<std_msgs::msg::Float32>(
            heading_topic, 10, std::bind(&PurePursuitNode::heading_callback, this, std::placeholders::_1));

        cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

        load_path(PATROL_PATH_FILE);
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr heading_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    
    int my_robot_number;
    std::string my_robot_name;
    std::vector<Point> path;
    Point current_position{0.0, 0.0, 0.0};
    double current_heading;
    unsigned long long target_index = 0;
    double linear_vel;
    double angular_vel;

    void load_path(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "경로 파일을 열 수 없습니다: %s", filename.c_str());
            return;
        }
        std::string line;
        std::getline(file, line);

        while (std::getline(file, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            std::string x_str, y_str, z_str;

            if (std::getline(ss, x_str, ',') &&
                std::getline(ss, y_str, ',') &&
                std::getline(ss, z_str, ',')) {
                try {
                    double x = std::stod(x_str);
                    double y = std::stod(y_str);
                    double z = std::stod(z_str);
                    path.emplace_back(Point{x, y, z});
                } catch (...) {
                    RCLCPP_ERROR(this->get_logger(), "CSV 변환 오류: %s", line.c_str());
                }
            }
        }
        file.close();
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_position.x = msg->pose.position.x;
        current_position.y = msg->pose.position.y;
        current_position.z = msg->pose.position.z;
        follow_path();
    }

    void heading_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        current_heading = msg->data;
    }

    double normalize_angle(double angle) {
        if (angle > M_PI) {
            return angle - 2.0 * M_PI;
        }
        if (angle < -M_PI) {
            return angle + 2.0 * M_PI;
        }
        return angle;
    }

    void follow_path() {
        if (path.empty()) {
            RCLCPP_WARN(this->get_logger(), "경로가 비어 있습니다.");
            return;
        }

        target_index = find_closest_point();
        Point target = path[target_index];
        double distance = std::hypot(target.x - current_position.x, target.y - current_position.y);
        
        double target_angle = 0.0;
        if (distance > 0.01) { // Ensure distance is above a small threshold
            target_angle = atan2(current_position.y - target.y, current_position.x - target.x);
        }

        double angle_error = normalize_angle(target_angle - current_heading);

        double target_speed = std::min(distance, MAX_LINEAR_SPEED);
        linear_vel += ACCEL_STEP * (target_speed - linear_vel);

        linear_vel = std::clamp(linear_vel, 0.0, MAX_LINEAR_SPEED);

        angular_vel += ANGLE_STEP * angle_error;
        angular_vel = std::clamp(angular_vel, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
        
        linear_vel *= FRICTION_FACTOR_LINEAR;
        angular_vel *= FRICTION_FACTOR_ANGULAR;

        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = linear_vel;
        cmd_vel_msg.angular.z = angular_vel;
        // unsigned long long 출력 %llu

        RCLCPP_INFO(this->get_logger(), "🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨");
        
        
        RCLCPP_INFO(this->get_logger(), "Target Point: (%.2f, %.2f, %.2f)\nCurrent Position: (%.2f, %.2f, %.2f)", 
                    target.x, target.y, target.z, current_position.x, current_position.y, current_position.z);
                    
        RCLCPP_INFO(this->get_logger(), "Target Heading: %.2f\nCurrent Heading: %.2f", 
                    target_angle, current_heading);
        
        RCLCPP_INFO(this->get_logger(), "🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨");

        cmd_vel_pub->publish(cmd_vel_msg);

        if (distance < LOOKAHEAD_DISTANCE) {
            if (target_index == path.size() - 1) {
                RCLCPP_INFO(this->get_logger(), "Reached the end of the path. Stopping the robot.");
                linear_vel = 0.0;
                angular_vel = 0.0;
            } else {
                target_index++;
            }
        }
    }

    size_t find_closest_point() {
        double min_distance = std::numeric_limits<double>::max();
        size_t closest_index = target_index;  // 기존 타겟 인덱스에서 시작

        for (size_t i = target_index; i < path.size(); ++i) {
            double dist = std::hypot(path[i].x - current_position.x, path[i].y - current_position.y);
            
            // LOOKAHEAD_DISTANCE 이상인 점을 찾고, 현재 위치보다 앞에 있는 점을 선택
            if (dist >= LOOKAHEAD_DISTANCE && dist < min_distance) {
                min_distance = dist;
                closest_index = i;
            }
        }

        // 만약 LOOKAHEAD_DISTANCE 이상인 점이 없으면 가장 가까운 점 사용
        if (min_distance == std::numeric_limits<double>::max()) {
            for (size_t i = 0; i < path.size(); ++i) {
                double dist = std::hypot(path[i].x - current_position.x, path[i].y - current_position.y);
                if (dist < min_distance) {
                    min_distance = dist;
                    closest_index = i;
                }
            }
        }

        return closest_index;
    }



};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitNode>());
    rclcpp::shutdown();
    return 0;
}
