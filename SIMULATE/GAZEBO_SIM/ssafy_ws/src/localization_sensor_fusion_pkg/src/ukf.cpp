/**********************************
    Created on : 10th Jan  2024
    Ported to ROS2 by: Shin Hyeon-hak 
    Github : Carepediem324
**********************************/

#include "localization_sensor_fusion_pkg/ukf.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//gps_common is not supported in ROS2
// use GeographicLib instead
#include <GeographicLib/UTMUPS.hpp>


#include <cmath>
#include <Eigen/LU>

// ── helper 함수 ──────────────────────────────────────────────
// quaternion을 받아서 yaw 각도를 반환
double getYaw(const geometry_msgs::msg::Quaternion & q_msg)
{
   tf2::Quaternion quat;
   tf2::fromMsg(q_msg, quat);

   tf2::Matrix3x3 mat(quat);
   double roll, pitch, yaw;
   mat.getRPY(roll, pitch, yaw);

   return yaw;
}

// ── VehicleState 생성자 ───────────────────────────────────────
VehicleState::VehicleState()
{
    vec = Vector5d::Zero();
    cov_mat = 1e-4 * Matrix5d::Identity(); // 작은 분산으로 초기화
}

// ── MMInput 생성자 ───────────────────────────────────────────
MMInput::MMInput()
{
    ax = 0.0;
    ay = 0.0;
    omega = 0.0;
    delT = 0.0;
}

// ── UKF 생성자 ───────────────────────────────────────────────
UKF::UKF(const rclcpp::Node::SharedPtr& node) : node_(node)
{
    // 파라미터 로딩 (declare_parameter 사용)
    _noise_var_pos = node_->declare_parameter("noise_var_pos", 1e-4);
    RCLCPP_INFO(node_->get_logger(), "noise_var_pos: %f", _noise_var_pos);
    _noise_var_yaw = node_->declare_parameter("noise_var_yaw", 1e-4);
    RCLCPP_INFO(node_->get_logger(), "noise_var_yaw: %f", _noise_var_yaw);
    _noise_var_vel = node_->declare_parameter("noise_var_vel", 1e-4);
    RCLCPP_INFO(node_->get_logger(), "noise_var_vel: %f", _noise_var_vel);
    _noise_var_meas = node_->declare_parameter("noise_var_meas", 1e-4);
    RCLCPP_INFO(node_->get_logger(), "noise_var_meas: %f", _noise_var_meas);
    _init_state_provided = node_->declare_parameter("init_state_provided", false);
    RCLCPP_INFO(node_->get_logger(), "init_state_provided: %s", _init_state_provided ? "true" : "false");
    _init_x = node_->declare_parameter("init_x", 0.0);
    RCLCPP_INFO(node_->get_logger(), "init_x: %f", _init_x);
    _init_y = node_->declare_parameter("init_y", 0.0);
    RCLCPP_INFO(node_->get_logger(), "init_y: %f", _init_y);
    _init_yaw = node_->declare_parameter("init_yaw", 0.0);
    RCLCPP_INFO(node_->get_logger(), "init_yaw: %f", _init_yaw);
    _init_velx = node_->declare_parameter("init_velx", 0.0);
    RCLCPP_INFO(node_->get_logger(), "init_velx: %f", _init_velx);
    _init_vely = node_->declare_parameter("init_vely", 0.0);
    RCLCPP_INFO(node_->get_logger(), "init_vely: %f", _init_vely);
    _init_var_pos = node_->declare_parameter("init_var_pos", 1e-4);
    RCLCPP_INFO(node_->get_logger(), "init_var_pos: %f", _init_var_pos);
    _init_var_yaw = node_->declare_parameter("init_var_yaw", 1e-4);
    RCLCPP_INFO(node_->get_logger(), "init_var_yaw: %f", _init_var_yaw);
    _init_var_vel = node_->declare_parameter("init_var_vel", 1e-4);
    RCLCPP_INFO(node_->get_logger(), "init_var_vel: %f", _init_var_vel);

    _imu_topic = node_->declare_parameter("imu_topic", std::string("/imu/data"));
    RCLCPP_INFO(node_->get_logger(), "imu_topic: %s", _imu_topic.c_str());
    _gnss_topic = node_->declare_parameter("gnss_topic", std::string("/fix"));
    RCLCPP_INFO(node_->get_logger(), "gnss_topic: %s", _gnss_topic.c_str());
    _odom_topic = node_->declare_parameter("odom_topic", std::string("/odometry/car"));
    RCLCPP_INFO(node_->get_logger(), "odom_topic: %s", _odom_topic.c_str());

    // 구독자 및 퍼블리셔 생성
    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        _imu_topic, 1, std::bind(&UKF::imuCallback, this, std::placeholders::_1));
    gnss_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        _gnss_topic, 1, std::bind(&UKF::gnssCallback, this, std::placeholders::_1));
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(_odom_topic, 1);

    // 초기 state 값 설정 (제공된 경우)
    if(_init_state_provided)
    {
        _curr_state.vec(0) = _init_x;
        _curr_state.vec(1) = _init_y;
        _curr_state.vec(2) = _init_yaw;
        _curr_state.vec(3) = _init_velx;
        _curr_state.vec(4) = _init_vely;
        _curr_state.cov_mat.diagonal() << _init_var_pos, _init_var_pos, _init_var_yaw,
                                          _init_var_vel, _init_var_vel;
    }

    _process_noise_mat.diagonal() << _noise_var_pos, _noise_var_pos, _noise_var_yaw,
                                     _noise_var_vel, _noise_var_vel;
    _meas_noise_mat.diagonal() << _noise_var_meas, _noise_var_meas;

    _is_first_imu = true;
    _is_first_gnss = true;
    _imu_available = false;
    _gnss_available = false;

    _prev_imu_time = 0.0;
    _curr_imu_time = 0.0;

    _init_utm = make_pair(0.0, 0.0);
    _curr_utm = make_pair(0.0, 0.0);
    _vp_at_first_gnss = make_pair(0.0, 0.0);

    _propagated_sigma_pt_mat = Matrix<double, 5, 11>::Zero();

    RCLCPP_INFO(node_->get_logger(), "UKF Constructor done");
}

/* @brief IMU 데이터 콜백 */
void UKF::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    if(_is_first_imu)
    {
        // ROS2의 타임스탬프는 sec와 nanosec로 구성됨
        _prev_imu_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        _curr_imu_time = _prev_imu_time;

        if(!_init_state_provided)
            initVehicleState(getYaw(msg->orientation));

        _is_first_imu = false;
    }
    else
    {
        _imu_available = true;
        _prev_imu_time = _curr_imu_time;
        _curr_imu_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        // 모션 모델 입력 업데이트
        _mmi.ax = msg->linear_acceleration.x;
        _mmi.ay = msg->linear_acceleration.y;
        _mmi.omega = msg->angular_velocity.z;
        _mmi.delT = _curr_imu_time - _prev_imu_time;
    }
}

/* @brief GNSS 데이터 콜백 */
void UKF::gnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr fix)
{
    int zone;
    bool northp;
    double easting, northing;
    // GeographicLib를 사용하여 UTM 좌표 변환 수행
    GeographicLib::UTMUPS::Forward(fix->latitude, fix->longitude, zone, northp, easting, northing);

    if(_is_first_gnss)
    {
        // 첫 번째 GNSS 수신 시, 초기 UTM 좌표를 저장 (easting -> first, northing -> second)
        _init_utm.first = easting;
        _init_utm.second = northing;

        // 첫 번째 GNSS 수신 시 차량 위치 저장
        _vp_at_first_gnss.first = _curr_state.vec(0);
        _vp_at_first_gnss.second = _curr_state.vec(1);

        RCLCPP_INFO(node_->get_logger(), "Pos at 1st gnss, x: %f, y: %f", _curr_state.vec(0), _curr_state.vec(1));

        _is_first_gnss = false;
    }
    else
    {
        _gnss_available = true;
        // 이후 GNSS 수신 시 현재 UTM 좌표를 업데이트 (easting -> first, northing -> second)
        _curr_utm.first = easting;
        _curr_utm.second = northing;
    }
}

/* @brief 초기 차량 state 설정 (yaw 값 초기화) */
void UKF::initVehicleState(const double yaw)
{
    RCLCPP_INFO(node_->get_logger(), "Initializing vehicle state with yaw = %f", yaw);
    _curr_state.vec(2) = yaw;
}

/* @brief IMU 데이터로 모션 모델 적용 */
Vector5d UKF::applyMotionModel(Vector5d vec)
{
    Vector5d propagated_vec;
    double del_theta = _mmi.omega * _mmi.delT;
    Rotation2D<double> rot1(del_theta);
    Vector2d del_vec(_mmi.ax * _mmi.delT, _mmi.ay * _mmi.delT);
    propagated_vec(2) = vec(2) + del_theta; // yaw 업데이트

    if(propagated_vec(2) > M_PI)
        propagated_vec(2) -= 2*M_PI;
    else if(propagated_vec(2) <= -M_PI)
        propagated_vec(2) += 2*M_PI;

    // 이전 base_link 프레임 속도를 현재 프레임으로 변환 후, 가속도 반영
    propagated_vec.segment(3,2) = rot1.toRotationMatrix() * vec.segment(3,2) + del_vec;

    del_vec(0) = propagated_vec(3)*_mmi.delT - _mmi.ax * pow(_mmi.delT, 2);
    del_vec(1) = propagated_vec(4)*_mmi.delT - _mmi.ay * pow(_mmi.delT, 2);
    Rotation2D<double> rot2(propagated_vec(2));
    propagated_vec.segment(0,2) = rot2.inverse().toRotationMatrix() * del_vec + vec.segment(0,2);

    return propagated_vec;
}

/* @brief 예측 단계 */
void UKF::predict()
{
    Matrix5d L = _curr_state.cov_mat.llt().matrixL(); // Cholesky 분해

    // sigma point 계산 (N+k = 3.0 사용)
    _propagated_sigma_pt_mat.col(0) = applyMotionModel(_curr_state.vec);
    for (int i = 1; i <= 5; i++)
    {
        _propagated_sigma_pt_mat.col(i) = applyMotionModel(_curr_state.vec + sqrt(3.0) * L.col(i-1));
        _propagated_sigma_pt_mat.col(i+5) = applyMotionModel(_curr_state.vec - sqrt(3.0) * L.col(i-1));
    }

    // 예측 평균 계산
    _predicted_state.vec = (-2.0/3.0) * _propagated_sigma_pt_mat.col(0);
    for (int i = 1; i < 11; i++)
        _predicted_state.vec += (1.0/6.0) * _propagated_sigma_pt_mat.col(i);

    // 예측 공분산 계산
    Matrix<double, 5, 11> del_state_mat = _propagated_sigma_pt_mat.colwise() - _predicted_state.vec;
    
    _predicted_state.cov_mat = (-2.0/3.0) * (del_state_mat.col(0)) * (del_state_mat.col(0).transpose());
    for (int i = 1; i < 11; i++)
        _predicted_state.cov_mat += (1.0/6.0) * (del_state_mat.col(i)) * (del_state_mat.col(i).transpose());
    
    _predicted_state.cov_mat += _process_noise_mat; // 프로세스 노이즈 추가
}

/* @brief 보정 단계 */
void UKF::correct(const double x, const double y)
{
    Matrix<double,2,11> predicted_meas_mat = _propagated_sigma_pt_mat.block<2,11>(0,0);
    Vector2d predicted_meas_vec(_predicted_state.vec(0), _predicted_state.vec(1));
    Matrix<double,2,11> del_meas_mat = predicted_meas_mat.colwise() - predicted_meas_vec;
    Matrix<double,5,11> del_state_mat = _propagated_sigma_pt_mat.colwise() - _predicted_state.vec;

    Matrix2d predicted_cov_mat;
    Matrix<double,5,2> cross_cov_mat;

    predicted_cov_mat = (-2.0/3.0) * del_meas_mat.col(0) * (del_meas_mat.col(0).transpose());
    cross_cov_mat = (-2.0/3.0) * del_state_mat.col(0) * (del_meas_mat.col(0).transpose());
    for (int i = 1; i < 11; i++)
    {
        predicted_cov_mat += (1.0/6.0) * del_meas_mat.col(i) * (del_meas_mat.col(i).transpose());
        cross_cov_mat += (1.0/6.0) * del_state_mat.col(i) * (del_meas_mat.col(i).transpose());
    }
    
    predicted_cov_mat += _meas_noise_mat; // 측정 노이즈 추가

    Matrix<double,5,2> kalman_gain = cross_cov_mat * predicted_cov_mat.inverse();

    Vector2d meas_vec(x, y);

    _curr_state.vec = _predicted_state.vec + kalman_gain * (meas_vec - predicted_meas_vec);
    _curr_state.cov_mat = _predicted_state.cov_mat - kalman_gain * predicted_cov_mat * (kalman_gain.transpose());
}

/* @brief 차량 상태를 nav_msgs::Odometry 메시지로 퍼블리시 */
void UKF::publishVehicleState()
{
    nav_msgs::msg::Odometry vs_msg;
    vs_msg.header.stamp = node_->now();
    vs_msg.header.frame_id = "map";
    vs_msg.child_frame_id = "base_link";

    vs_msg.pose.pose.position.x = _curr_state.vec(0);
    vs_msg.pose.pose.position.y = _curr_state.vec(1);
    vs_msg.pose.pose.position.z = 0.0;

    Eigen::AngleAxisd rot_mat(_curr_state.vec(2), Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond quat(rot_mat);
    vs_msg.pose.pose.orientation.x = quat.x();
    vs_msg.pose.pose.orientation.y = quat.y();
    vs_msg.pose.pose.orientation.z = quat.z();
    vs_msg.pose.pose.orientation.w = quat.w();

    vs_msg.twist.twist.linear.x = _curr_state.vec(3);
    vs_msg.twist.twist.linear.y = _curr_state.vec(4);
    vs_msg.twist.twist.linear.z = 0.0;
    vs_msg.twist.twist.angular.x = 0.0;
    vs_msg.twist.twist.angular.y = 0.0;
    vs_msg.twist.twist.angular.z = _mmi.omega;

    // covariance 값은 0 대신 작은 수로 초기화
    for (int i = 0; i < 36; i++)
    {
        vs_msg.pose.covariance[i] = 1e-6;
        vs_msg.twist.covariance[i] = 1e-6;
    }

    vs_msg.pose.covariance[0] = _curr_state.cov_mat(0,0);   // x 분산
    vs_msg.pose.covariance[7] = _curr_state.cov_mat(1,1);   // y 분산
    vs_msg.pose.covariance[35] = _curr_state.cov_mat(2,2);  // theta 분산
    vs_msg.twist.covariance[0] = _curr_state.cov_mat(3,3);  // vx 분산
    vs_msg.twist.covariance[7] = _curr_state.cov_mat(4,4);  // vy 분산

    odom_pub_->publish(vs_msg);
}

/* @brief UKF 추정을 실행 */
void UKF::estimateVehicleState()
{
    if(_imu_available)
    {
        predict(); // 예측 단계 실행
        _imu_available = false;
    }
    
    if(_gnss_available)
    {
        double meas_x = (_curr_utm.first - _init_utm.first) + _vp_at_first_gnss.first;
        double meas_y = (_curr_utm.second - _init_utm.second) + _vp_at_first_gnss.second;
        correct(meas_x, meas_y); // 보정 단계 실행
        _gnss_available = false;
    }
    else
    {
        _curr_state = _predicted_state; // 새로운 측정이 없으면 예측값 사용
    }

    publishVehicleState();
}
