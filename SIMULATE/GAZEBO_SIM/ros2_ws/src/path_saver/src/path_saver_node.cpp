#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <fstream>
#include <string>
#include <chrono>
#include <ctime>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <cerrno>
#include <cstring>

class PathSaverNode : public rclcpp::Node {
public:
    PathSaverNode() : Node("path_saver_node") {
        this->declare_parameter<int>("robot_number", -1);
        int robot_number = this->get_parameter("robot_number").as_int();

        // 절대 경로로 디렉토리 설정
        std::string directory_path = "/home/ubuntu/csv_files";
        createDirectoryIfNotExists(directory_path);

        // 동적 토픽 이름 설정
        std::string topic_name = "/robot_" + std::to_string(robot_number) + "/utm_pose";
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        topic_name, 10, std::bind(&PathSaverNode::utmPoseCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribing to topic: %s", topic_name.c_str());

        // CSV 파일 이름 설정
        std::string file_name = directory_path + "/robot_" + std::to_string(robot_number) + "_path_" + getCurrentDateTime() + ".csv";
        RCLCPP_INFO(this->get_logger(), "CSV file will be created at: %s", file_name.c_str());
        file_.open(file_name, std::ios::out | std::ios::trunc);
        if (file_.is_open()) {
            file_ << "x,y,z" << std::endl;
            RCLCPP_INFO(this->get_logger(), "Saving data to file: %s", file_name.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s. Ensure directory exists and is writable.", file_name.c_str());
        }
    }

    ~PathSaverNode() {
        if (file_.is_open()) {
            file_.close();
        }
    }

private:
    void createDirectoryIfNotExists(const std::string &directory_path) {
        struct stat info;
        if (stat(directory_path.c_str(), &info) != 0) {
            RCLCPP_INFO(this->get_logger(), "Directory does not exist. Creating: %s", directory_path.c_str());
            if (mkdir(directory_path.c_str(), 0777) == -1) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create directory: %s. Error: %s", directory_path.c_str(), strerror(errno));
            } else {
                RCLCPP_INFO(this->get_logger(), "Directory created successfully: %s", directory_path.c_str());
            }
        } else if (!(info.st_mode & S_IFDIR)) {
            RCLCPP_ERROR(this->get_logger(), "%s exists but is not a directory!", directory_path.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Directory already exists: %s", directory_path.c_str());
        }
    }

    void utmPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (file_.is_open()) {
            
            auto x = msg->pose.position.x;
            auto y = msg->pose.position.y;
            auto z = msg->pose.position.z;

            file_  << x << "," << y << "," << z << std::endl;
            file_.flush(); // 데이터를 즉시 디스크에 기록
            RCLCPP_INFO(this->get_logger(), "Saved pose:  x=%.2f, y=%.2f, z=%.2f",
                        x, y, z);
        } else {
            RCLCPP_ERROR(this->get_logger(), "File is not open. Unable to save data.");
        }
    }

    std::string getCurrentDateTime() {
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);

        std::ostringstream oss;
        oss << std::put_time(&tm, "%y%m%d_%H%M");
        return oss.str();
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    std::ofstream file_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathSaverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
