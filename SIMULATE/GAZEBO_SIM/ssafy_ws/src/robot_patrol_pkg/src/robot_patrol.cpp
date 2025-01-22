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

struct Point {
    double x, y, z;
};

class PurePursuitNode : public rclcpp::Node {
public:
    std::string robot_name;
    int robot_num;

    PurePursuitNode() : Node("pure_pursuit_node"), current_target_index_(0), patrol_direction_(1) {
        this->declare_parameter<std::string>("robot_name", "not_defined");
        robot_name = this->get_parameter("robot_name").as_string();

        this->declare_parameter<int>("robot_number", -1);
        robot_num = this->get_parameter("robot_number").as_int();

        std::string pose_topic = "/robot_" + std::to_string(robot_num) + "/utm_pose";
        std::string heading_topic = "/robot_" + std::to_string(robot_num) + "/heading";
        std::string cmd_vel_topic = "/" + robot_name + "/cmd_vel";

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic, 8, std::bind(&PurePursuitNode::poseCallback, this, std::placeholders::_1));

        heading_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            heading_topic, 200, std::bind(&PurePursuitNode::headingCallback, this, std::placeholders::_1));

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

        loadPath(PATROL_PATH_FILE);
        findClosestEndPoint();
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr heading_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    std::vector<Point> path_;
    double current_heading_ = 0.0;
    Point current_position_{0.0, 0.0, 0.0};
    size_t current_target_index_;
    int patrol_direction_;

    void loadPath(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "파일을 열 수 없습니다: %s", filename.c_str());
            return;
        }
        std::string line;
        bool first_line = true;
        while (std::getline(file, line)) {
            if (first_line) {
                first_line = false;
                continue;
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

    void findClosestEndPoint() {
        double min_distance = std::numeric_limits<double>::max();
        for (size_t i = 0; i < path_.size(); i += (path_.size() - 1)) {
            double dist = std::hypot(path_[i].x - current_position_.x, path_[i].y - current_position_.y);
            if (dist < min_distance) {
                min_distance = dist;
                current_target_index_ = i;
                patrol_direction_ = (i == 0) ? 1 : -1;
            }
        }
    }

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_position_.x = msg->pose.position.x;
        current_position_.y = msg->pose.position.y;
        current_position_.z = msg->pose.position.z;

        followPath();
    }

    void headingCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        current_heading_ = msg->data;
    }

    void followPath() {
        Point target = path_[current_target_index_];
        double distance = std::hypot(target.x - current_position_.x, target.y - current_position_.y);

        if (distance < LOOKAHEAD_DISTANCE) {
            if (current_target_index_ == 0 || current_target_index_ == path_.size() - 1) {
                patrol_direction_ *= -1;
                turnAround();
            }
            current_target_index_ += patrol_direction_;
        }

        double angle_to_target = atan2(target.y - current_position_.y, target.x - current_position_.x);
        double angle_error = normalizeAngle(angle_to_target - current_heading_);

        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = 0.1;
        cmd_vel_msg.angular.z = angle_error;
        cmd_vel_pub_->publish(cmd_vel_msg);
    }

    void turnAround() {
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.angular.z = M_PI;
        cmd_vel_pub_->publish(cmd_vel_msg);
        rclcpp::sleep_for(std::chrono::seconds(2));
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
