#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>
#include <algorithm> // std::clamp

class IMUVelocityNode : public rclcpp::Node
{
public:
    IMUVelocityNode() : Node("imu_velocity_node"),
                        velocity_x_(0.0),
                        velocity_y_(0.0),
                        velocity_z_(0.0),
                        prev_time_(this->now())
    {
        // 1) 파라미터 선언
        this->declare_parameter<std::string>("robot_name", "not_defined");
        this->declare_parameter<int>("robot_number", -1);
        

        robot_name_ = this->get_parameter("robot_name").as_string();
        robot_num_ = this->get_parameter("robot_number").as_int();
        

        // 2) 구독 토픽 결정
        std::string imu_topic = "/" + robot_name_ + "/imu";
        std::string gps_topic = "/" + robot_name_ + "/gps";

        // 3) 발행 토픽 결정
        std::string speed_mps_topic = "/robot_" + std::to_string(robot_num_) +  "/speed_mps";
        std::string speed_kph_topic = "/robot_" + std::to_string(robot_num_) +  "/speed_kph";

        // IMU 구독
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic,
            10,
            std::bind(&IMUVelocityNode::imuCallback, this, std::placeholders::_1)
        );

        // GPS 구독
        gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            gps_topic,
            10,
            std::bind(&IMUVelocityNode::gpsCallback, this, std::placeholders::_1)
        );

        // 속도 발행
        velocity_publisher_mps_ = this->create_publisher<std_msgs::msg::Float64>(
            speed_mps_topic, 10);

        velocity_publisher_kph_ = this->create_publisher<std_msgs::msg::Float64>(
            speed_kph_topic, 10);

        RCLCPP_INFO(this->get_logger(), "IMU Velocity Node started.");
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // 현재 시간과 경과 시간 계산
        rclcpp::Time current_time = this->now();
        double delta_time = (current_time - prev_time_).seconds();
        prev_time_ = current_time;

        // 경과 시간 보정
        delta_time = std::clamp(delta_time, 0.001, 0.1); // 최소 1ms, 최대 100ms

        // IMU 데이터 보정 및 노이즈 제거
        double ax = applyNoiseThreshold(msg->linear_acceleration.x);
        double ay = applyNoiseThreshold(msg->linear_acceleration.y);
        double az = applyNoiseThreshold(msg->linear_acceleration.z - 9.81); // 중력 제거

        // 속도 계산 (적분)
        velocity_x_ += ax * delta_time;
        velocity_y_ += ay * delta_time;
        velocity_z_ += az * delta_time;

        publishVelocity();
    }

    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        static double prev_latitude = 0.0;
        static double prev_longitude = 0.0;
        static rclcpp::Time prev_time = this->now();

        // 현재 GPS 위치와 시간 가져오기
        double current_latitude = msg->latitude;
        double current_longitude = msg->longitude;
        rclcpp::Time current_time = this->now();

        // 이전 데이터가 있으면 속도 계산
        if (prev_latitude != 0.0 && prev_longitude != 0.0)
        {
            // 경과 시간 (초)
            double delta_time = (current_time - prev_time).seconds();

            // GPS 위치 변화량 (단위: 미터)
            double delta_distance = haversineDistance(prev_latitude, prev_longitude, current_latitude, current_longitude);

            // GPS 속도 계산 (m/s)
            double gps_speed = delta_distance / delta_time;

            // IMU 속도를 GPS 속도로 보정 (융합)
            velocity_x_ = (velocity_x_ + gps_speed) / 2.0; // 간단히 평균을 사용
            velocity_y_ = (velocity_y_ + gps_speed) / 2.0;
            velocity_z_ = 0.0; // GPS는 보통 2D 데이터이므로 Z축 속도는 유지하거나 보정하지 않음

            // 로그 출력
            //RCLCPP_INFO(this->get_logger(), "GPS Speed: %.2f m/s, IMU Velocity Adjusted.", gps_speed);
        }

        // 이전 데이터를 업데이트
        prev_latitude = current_latitude;
        prev_longitude = current_longitude;
        prev_time = current_time;

        publishVelocity();
    }

    // GPS 거리 계산 함수 (Haversine 공식)
    double haversineDistance(double lat1, double lon1, double lat2, double lon2)
    {
        constexpr double R = 6371000.0; // 지구 반지름 (미터)
        double dlat = degreesToRadians(lat2 - lat1);
        double dlon = degreesToRadians(lon2 - lon1);

        double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
                   std::cos(degreesToRadians(lat1)) * std::cos(degreesToRadians(lat2)) *
                   std::sin(dlon / 2) * std::sin(dlon / 2);

        double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
        return R * c;
    }

    // 도(degree) 단위를 라디안으로 변환
    double degreesToRadians(double degrees)
    {
        return degrees * M_PI / 180.0;
    }

    void publishVelocity()
    {
        // 속력 계산
        double linear_velocity = std::sqrt(velocity_x_ * velocity_x_ + velocity_y_ * velocity_y_ + velocity_z_ * velocity_z_);

        // 속도 값이 0.02m/s 이하이면 0으로 설정
        if (linear_velocity < 0.03)
        {
            linear_velocity = 0.0;
        }

        double velocity_mps = std::clamp(linear_velocity, 0.0, 2.0);
        double velocity_kph = velocity_mps * 3.6;

        // m/s 단위 속도 발행
        auto speed_msg_mps = std_msgs::msg::Float64();
        speed_msg_mps.data = velocity_mps;
        velocity_publisher_mps_->publish(speed_msg_mps);

        // km/h 단위 속도 발행
        auto speed_msg_kph = std_msgs::msg::Float64();
        speed_msg_kph.data = velocity_kph;
        velocity_publisher_kph_->publish(speed_msg_kph);

        // 로그 출력
        // RCLCPP_INFO(this->get_logger(),
        //             "Velocity: %.2f m/s, %.2f km/h",
        //             velocity_mps, velocity_kph);
    }

    double applyNoiseThreshold(double value, double threshold = 0.05)
    {
        return (std::fabs(value) > threshold) ? value : 0.0;
    }

    // 멤버 변수
    std::string robot_name_;
    int robot_num_;
    double robot_radius_; // 로봇 회전 반경
    double velocity_x_;   // X축 속도
    double velocity_y_;   // Y축 속도
    double velocity_z_;   // Z축 속도
    rclcpp::Time prev_time_; // 이전 시간

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_; // IMU 구독자
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_; // GPS 구독자
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_publisher_mps_; // m/s 발행
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_publisher_kph_; // km/h 발행
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUVelocityNode>());
    rclcpp::shutdown();
    return 0;
}
