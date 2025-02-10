/**********************************
    Created on : 10th Jan  2024
    Ported to ROS2 by: Shin Hyeon-hak 
    Github : Carepediem324
**********************************/

#ifndef UKF_UKF_H
#define UKF_UKF_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>
#include <utility>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

// 2D motion 가정, Vehicle state: {x, y, theta, vx, vy}
typedef Matrix<double, 5, 5> Matrix5d;
typedef Matrix<double, 5, 1> Vector5d;

/* state: 5 요소 [x, y, theta, vx, vy]와 5x5 공분산 행렬 */
class VehicleState
{
public:
    VehicleState();
    ~VehicleState() = default;

    Vector5d vec;
    Matrix5d cov_mat;
};

class MMInput
{
public:
    MMInput();
    ~MMInput() = default;

    double ax;      // 가속도 (x)
    double ay;      // 가속도 (y)
    double omega;   // yaw rate
    double delT;    // 시간 간격
};

class UKF
{
public:
    // 생성자: ROS2 Node의 shared pointer를 인자로 받음
    UKF(const rclcpp::Node::SharedPtr& node);
    ~UKF() = default;

    void estimateVehicleState(); // UKF 추정 수행

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);       // IMU 콜백
    void gnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);  // GNSS 콜백

    void initVehicleState(const double yaw);         // 초기 state 설정
    void predict();                                  // 예측 단계
    // 수정된 선언 (const reference 사용)
    Vector5d applyMotionModel(const Vector5d & vec);  // 모션 모델 적용

    void correct(const double x, const double y);    // 보정 단계
    void publishVehicleState();                      // Odometry 메시지로 상태 퍼블리시
    static double normalizeAngle(double angle) {
        while (angle >  M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    // ROS2 Node pointer
    rclcpp::Node::SharedPtr node_;

    // 구독자와 퍼블리셔 (ROS2에서는 create_subscription, create_publisher 사용)
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_mps_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_kmph_pub_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr heading_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr utm_pub_;
    // 타이머 (ROS2에서는 create_wall_timer 사용)
    rclcpp::TimerBase::SharedPtr _timer;
    // 토픽 이름
    string _imu_topic, _gnss_topic, _odom_topic;
    string _vel_kmph_topic, _vel_mps_topic; // 선속도 토픽 이름

    string heading_topic;
    string utm_topic;
    // 노이즈 분산 등 파라미터
    double _noise_var_pos, _noise_var_yaw, _noise_var_vel, _noise_var_meas;
    bool _init_state_provided;
    double _init_x, _init_y, _init_yaw, _init_velx, _init_vely;
    double _init_var_pos, _init_var_yaw, _init_var_vel;
    
    double _curr_altitude; // 고도 값을 저장할 멤버 변수 추가
    double _latest_heading;

    VehicleState _curr_state, _predicted_state;
    pair<double, double> _init_utm, _curr_utm;       // (easting, northing)
    pair<double, double> _vp_at_first_gnss;            // 초기 GNSS 수신 시 차량 위치
    bool _is_first_imu, _is_first_gnss;
    bool _imu_available, _gnss_available;
    double _prev_imu_time, _curr_imu_time;
    MMInput _mmi;                                     // 현재 모션 모델 입력
    Matrix<double, 5, 11> _propagated_sigma_pt_mat;     // 모션 모델을 통해 전파된 sigma point 행렬
    Matrix5d _process_noise_mat;                        // 프로세스 노이즈 공분산
    Matrix2d _meas_noise_mat;                           // 측정 노이즈 공분산
};

#endif // UKF_UKF_H
