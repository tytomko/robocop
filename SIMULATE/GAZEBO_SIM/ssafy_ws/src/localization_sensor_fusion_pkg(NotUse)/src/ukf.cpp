/**********************************
    Created on : 10th Jan 2024
    Ported to ROS2 by: Shin Hyeon-hak 
    Github : Carepediem324
**********************************/

#include "localization_sensor_fusion_pkg/ukf.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <GeographicLib/UTMUPS.hpp>  // ROS2에서는 gps_common 대신 GeographicLib 사용

#include <cmath>
#include <Eigen/LU>

// ─────────────────────────────────────────────────────────────
// Helper function: Extract yaw angle from a quaternion message
double getYaw(const geometry_msgs::msg::Quaternion &q_msg)
{
    tf2::Quaternion quat;
    tf2::fromMsg(q_msg, quat);
    tf2::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    return yaw;
}

// ─────────────────────────────────────────────────────────────
// VehicleState constructor: initialize 5D state vector and covariance matrix
VehicleState::VehicleState()
{
    vec = Vector5d::Zero();
    cov_mat = 1e-4 * Matrix5d::Identity();  // 작은 초기 분산
}

// ─────────────────────────────────────────────────────────────
// MMInput constructor: initialize motion model input values
MMInput::MMInput()
{
    ax = 0.0;
    ay = 0.0;
    omega = 0.0;
    delT = 0.0;
}

// ─────────────────────────────────────────────────────────────
// UKF constructor: declare parameters, initialize noise matrices, create subscribers/publishers, and set up a timer
UKF::UKF(const rclcpp::Node::SharedPtr& node) : node_(node)
{
    // Declare noise and initial state parameters
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

    // Declare sensor and output topic names
    _imu_topic = node_->declare_parameter("imu_topic", std::string("/imu/data"));
    RCLCPP_INFO(node_->get_logger(), "imu_topic: %s", _imu_topic.c_str());
    _gnss_topic = node_->declare_parameter("gnss_topic", std::string("/fix"));
    RCLCPP_INFO(node_->get_logger(), "gnss_topic: %s", _gnss_topic.c_str());
    _odom_topic = node_->declare_parameter("odom_topic", std::string("/odometry/car"));
    RCLCPP_INFO(node_->get_logger(), "odom_topic: %s", _odom_topic.c_str());
    
    // Additional topics for vehicle speed publishing
    _vel_mps_topic = node_->declare_parameter("vel_mps_topic", std::string("/velocity/mps"));
    _vel_kmph_topic = node_->declare_parameter("vel_kmph_topic", std::string("/velocity/kmph"));
    RCLCPP_INFO(node_->get_logger(), "vel_mps_topic: %s", _vel_mps_topic.c_str());
    

    // 추가: Heading 전용 토픽 (필터 출력 사용)
    heading_topic = node_->declare_parameter("heading_topic", std::string("/heading"));
    RCLCPP_INFO(node_->get_logger(), "heading_topic: %s", heading_topic.c_str());
    // 추가: UTM 좌표계 토픽 (필터 출력 사용)
    utm_topic = node_->declare_parameter("utm_topic", std::string("/utm_pose"));
    RCLCPP_INFO(node_->get_logger(), "utm_topic: %s", utm_topic.c_str());

    // Create subscribers for IMU and GNSS data
    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        _imu_topic, 1, std::bind(&UKF::imuCallback, this, std::placeholders::_1));
    gnss_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        _gnss_topic, 1, std::bind(&UKF::gnssCallback, this, std::placeholders::_1));
    
    heading_pub_ = node_->create_publisher<std_msgs::msg::Float32>(heading_topic, 1);
    utm_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(utm_topic, 1);

    // Create publishers for odometry and speed
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(_odom_topic, 1);
    vel_mps_pub_ = node_->create_publisher<std_msgs::msg::Float64>(_vel_mps_topic, 1);
    vel_kmph_pub_ = node_->create_publisher<std_msgs::msg::Float64>(_vel_kmph_topic, 1);

    // Initialize flags
    _is_first_imu = true;
    _is_first_gnss = true;
    _imu_available = false;
    _gnss_available = false;

    // Initialize process and measurement noise matrices
    _process_noise_mat = Matrix5d::Identity() * _noise_var_pos;   // 상태 노이즈
    _meas_noise_mat = Matrix2d::Identity() * _noise_var_meas;     // 측정 노이즈

    // Initialize latest heading with zero.
    _latest_heading = 0.0;

    // Create a timer to periodically run the state estimation (50ms period)
    _timer = node_->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&UKF::estimateVehicleState, this));

    RCLCPP_INFO(node_->get_logger(), "UKF Constructor done");
}

// ─────────────────────────────────────────────────────────────
// IMU callback: Update motion model inputs from IMU data only.
// Heading is now published using the filtered state.
void UKF::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    double time_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    
    if (_is_first_imu)
    {
        _prev_imu_time = time_sec;
        _curr_imu_time = time_sec;

        if (!_init_state_provided)
            //constexpr double OFFSET = -M_PI;
            initVehicleState(normalizeAngle(getYaw(msg->orientation) - M_PI));

        _is_first_imu = false;
    }
    else
    {
        _imu_available = true;
        _prev_imu_time = _curr_imu_time;
        _curr_imu_time = time_sec;

        // Update motion model inputs from IMU
        _mmi.ax = msg->linear_acceleration.x;
        _mmi.ay = msg->linear_acceleration.y;
        _mmi.omega = msg->angular_velocity.z;
        _mmi.delT = _curr_imu_time - _prev_imu_time;
    }
    // 이제 raw IMU 데이터로 heading을 발행하지 않습니다.
}

// ─────────────────────────────────────────────────────────────
// GNSS callback: Convert GNSS (NavSatFix) to UTM and update state measurements.
void UKF::gnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr fix)
{
    int zone;
    bool northp;
    double easting, northing;
    GeographicLib::UTMUPS::Forward(fix->latitude, fix->longitude, zone, northp, easting, northing);

    _curr_altitude = fix->altitude;

    if (_is_first_gnss)
    {
        // Record initial GNSS UTM coordinates always
        _init_utm.first = easting;
        _init_utm.second = northing;
        _vp_at_first_gnss.first = easting;
        _vp_at_first_gnss.second = northing;

        // If external initial state is not provided, update vehicle state position
        if (!_init_state_provided)
        {
            _curr_state.vec(0) = easting;
            _curr_state.vec(1) = northing;
        }

        RCLCPP_INFO(node_->get_logger(), "Initializing GNSS pos, x: %f, y: %f, altitude: %f",
                    easting, northing, _curr_altitude);

        _is_first_gnss = false;
    }
    else
    {
        _gnss_available = true;
        _curr_utm.first = easting;
        _curr_utm.second = northing;
    }
}

// ─────────────────────────────────────────────────────────────
// Initialize vehicle state: Set initial yaw from IMU.
void UKF::initVehicleState(const double yaw)
{
    RCLCPP_INFO(node_->get_logger(), "Initializing vehicle state with yaw = %f", yaw);
    _curr_state.vec(2) = yaw;
    _latest_heading = yaw;  // 초기 heading 저장
}

// ─────────────────────────────────────────────────────────────
// Apply motion model: Propagate state using IMU inputs.
// Use a minimum dt value to prevent numerical issues.
Vector5d UKF::applyMotionModel(const Vector5d &vec)
{
    double dt = _mmi.delT;
    if (dt < 1e-6) dt = 1e-6;

    Vector5d new_state = vec;
    // (1) Update yaw
    new_state(2) = vec(2) + _mmi.omega * dt;
    if (new_state(2) > M_PI)
        new_state(2) -= 2 * M_PI;
    else if (new_state(2) <= -M_PI)
        new_state(2) += 2 * M_PI;
    // (2) Convert acceleration to map frame
    Eigen::Vector2d acc_body(_mmi.ax, _mmi.ay);
    Eigen::Rotation2D<double> R(vec(2));
    Eigen::Vector2d acc_map = R.toRotationMatrix() * acc_body;
    // (3) Update velocity
    new_state(3) = vec(3) + acc_map(0) * dt;
    new_state(4) = vec(4) + acc_map(1) * dt;
    // (4) Update position
    new_state(0) = vec(0) + vec(3) * dt + 0.5 * acc_map(0) * dt * dt;
    new_state(1) = vec(1) + vec(4) * dt + 0.5 * acc_map(1) * dt * dt;
    return new_state;
}

// ─────────────────────────────────────────────────────────────
// Predict step: Generate sigma points, propagate through motion model,
// and compute predicted state mean and covariance.
void UKF::predict()
{
    Matrix5d L = _curr_state.cov_mat.llt().matrixL();
    _propagated_sigma_pt_mat.col(0) = applyMotionModel(_curr_state.vec);
    for (int i = 1; i <= 5; i++)
    {
        _propagated_sigma_pt_mat.col(i) = applyMotionModel(_curr_state.vec + sqrt(3.0) * L.col(i - 1));
        _propagated_sigma_pt_mat.col(i + 5) = applyMotionModel(_curr_state.vec - sqrt(3.0) * L.col(i - 1));
    }
    _predicted_state.vec = (-2.0 / 3.0) * _propagated_sigma_pt_mat.col(0);
    for (int i = 1; i < 11; i++)
        _predicted_state.vec += (1.0 / 6.0) * _propagated_sigma_pt_mat.col(i);
    Matrix<double, 5, 11> del_state_mat = _propagated_sigma_pt_mat.colwise() - _predicted_state.vec;
    _predicted_state.cov_mat = (-2.0 / 3.0) * (del_state_mat.col(0)) * (del_state_mat.col(0).transpose());
    for (int i = 1; i < 11; i++)
        _predicted_state.cov_mat += (1.0 / 6.0) * (del_state_mat.col(i)) * (del_state_mat.col(i).transpose());
    _predicted_state.cov_mat += _process_noise_mat;
}

// ─────────────────────────────────────────────────────────────
// Correct step: Compute measurement covariance and cross covariance from sigma points,
// then update state using the Kalman gain.
void UKF::correct(const double x, const double y)
{
    Matrix<double, 2, 11> predicted_meas_mat = _propagated_sigma_pt_mat.block<2, 11>(0, 0);
    Vector2d predicted_meas_vec(_predicted_state.vec(0), _predicted_state.vec(1));
    Matrix<double, 2, 11> del_meas_mat = predicted_meas_mat.colwise() - predicted_meas_vec;
    Matrix<double, 5, 11> del_state_mat = _propagated_sigma_pt_mat.colwise() - _predicted_state.vec;
    Matrix2d predicted_cov_mat = (-2.0 / 3.0) * del_meas_mat.col(0) * (del_meas_mat.col(0).transpose());
    Matrix<double, 5, 2> cross_cov_mat = (-2.0 / 3.0) * del_state_mat.col(0) * (del_meas_mat.col(0).transpose());
    for (int i = 1; i < 11; i++)
    {
        predicted_cov_mat += (1.0 / 6.0) * del_meas_mat.col(i) * (del_meas_mat.col(i).transpose());
        cross_cov_mat += (1.0 / 6.0) * del_state_mat.col(i) * (del_meas_mat.col(i).transpose());
    }
    predicted_cov_mat += Matrix2d::Identity() * 1e-6;  // 안정성 보완
    predicted_cov_mat += _meas_noise_mat;
    Matrix<double, 5, 2> kalman_gain = cross_cov_mat * predicted_cov_mat.inverse();
    Vector2d meas_vec(x, y);
    _curr_state.vec = _predicted_state.vec + kalman_gain * (meas_vec - predicted_meas_vec);
    _curr_state.cov_mat = _predicted_state.cov_mat - kalman_gain * predicted_cov_mat * (kalman_gain.transpose());
}

// ─────────────────────────────────────────────────────────────
// Publish vehicle state: Publish odometry message using the filtered state,
// and also publish vehicle speed and filtered heading.
void UKF::publishVehicleState()
{
    nav_msgs::msg::Odometry vs_msg;
    vs_msg.header.stamp = node_->now();
    vs_msg.header.frame_id = "map";
    vs_msg.child_frame_id = "base_link";

    // Use filtered state for position and orientation
    vs_msg.pose.pose.position.x = _curr_state.vec(0);
    vs_msg.pose.pose.position.y = _curr_state.vec(1);
    vs_msg.pose.pose.position.z = _curr_altitude;

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

    // 기본 covariance 초기화 (1e-6)
    for (int i = 0; i < 36; i++)
    {
        vs_msg.pose.covariance[i] = 1e-6;
        vs_msg.twist.covariance[i] = 1e-6;
    }
    vs_msg.pose.covariance[0] = _curr_state.cov_mat(0, 0);
    vs_msg.pose.covariance[7] = _curr_state.cov_mat(1, 1);
    vs_msg.pose.covariance[35] = _curr_state.cov_mat(2, 2);
    vs_msg.twist.covariance[0] = _curr_state.cov_mat(3, 3);
    vs_msg.twist.covariance[7] = _curr_state.cov_mat(4, 4);

    odom_pub_->publish(vs_msg);

    // Publish vehicle speed in m/s and km/h
    double v_mps = std::sqrt(_curr_state.vec(3) * _curr_state.vec(3) +
                               _curr_state.vec(4) * _curr_state.vec(4));
    double v_kmph = v_mps * 3.6;
    
    std_msgs::msg::Float64 vel_mps_msg;
    vel_mps_msg.data = v_mps;
    vel_mps_pub_->publish(vel_mps_msg);

    std_msgs::msg::Float64 vel_kmph_msg;
    vel_kmph_msg.data = v_kmph;
    vel_kmph_pub_->publish(vel_kmph_msg);

    // Publish filtered heading from the UKF state (heading in rad)
    std_msgs::msg::Float32 heading_msg;
    heading_msg.data = static_cast<float>(_curr_state.vec(2));
    heading_pub_->publish(heading_msg);

    geometry_msgs::msg::PoseStamped utm_msg;
    utm_msg.header.stamp = node_->now();
    utm_msg.header.frame_id = "map";
    utm_msg.pose.position.x = _curr_state.vec(0);
    utm_msg.pose.position.y = _curr_state.vec(1);
    utm_msg.pose.position.z = _curr_altitude;
    //
    utm_pub_->publish(utm_msg);
}

// ─────────────────────────────────────────────────────────────
// Estimate vehicle state: If new IMU or GNSS data is available, perform prediction
// and correction, then publish the estimated state.
void UKF::estimateVehicleState()
{
    if (_imu_available)
    {
        predict();
        _imu_available = false;
    }
    
    if (_gnss_available)
    {
        double meas_x = (_curr_utm.first - _init_utm.first) + _vp_at_first_gnss.first;
        double meas_y = (_curr_utm.second - _init_utm.second) + _vp_at_first_gnss.second;
        correct(meas_x, meas_y);
        _gnss_available = false;
    }
    else
    {
        _curr_state = _predicted_state;
    }

    publishVehicleState();
}
