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
constexpr double LOOKAHEAD_DISTANCE = 0.2;  // 수정된 탐색 거리 미터단위
constexpr double MAX_LINEAR_SPEED = 2.0;    // 수정된 최대 선속도
constexpr double MAX_ANGULAR_SPEED = 1.5;   // 수정된 최대 각속도
constexpr double ACCEL_STEP = 0.05;
// 각변화 수치 계수
constexpr double ANGLE_STEP = 0.1;
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
    size_t target_index;
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
        follow_path();
    }

    void heading_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        current_heading = msg->data;
    }

    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
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
        double target_angle = atan2(target.y - current_position.y, target.x - current_position.x);
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
        RCLCPP_INFO(this->get_logger(), "Target index: %d, Distance: %.2f, Angle Error: %.2f", 
            target_index, distance, angle_error);
        RCLCPP_INFO(this->get_logger(), "Linear Vel: %.2f, Angular Vel: %.2f", 
                    linear_vel, angular_vel);
        cmd_vel_pub->publish(cmd_vel_msg);

        if (distance < LOOKAHEAD_DISTANCE) {
            if (target_index == path.size() - 1) {
                std::reverse(path.begin(), path.end());
                target_index = 0;
            } else {
                target_index++;
            }
        }
    }

    size_t find_closest_point() {
        double min_distance = std::numeric_limits<double>::max();
        size_t closest_index = 0;
        for (size_t i = 0; i < path.size(); ++i) {
            double dist = std::hypot(path[i].x - current_position.x, path[i].y - current_position.y);
            if (dist < min_distance) {
                min_distance = dist;
                closest_index = i;
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
