// 파일: src/velodyne_detection.cpp

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp" 
#include "visualization_msgs/msg/marker_array.hpp"
#include "robot_custom_interfaces/srv/estop.hpp"
#include "robot_custom_interfaces/msg/status.hpp" // status 메시지 타입

// PCL 관련 헤더들
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h> // EuclideanClusterExtraction
#include <pcl/common/common.h>       // getMinMax3D 등 공통 함수
#include <pcl/common/centroid.h>     // compute3DCentroid 사용

#include <chrono>
#include <memory>
#include <vector>

using namespace std::chrono_literals;

// 필터 및 클러스터링 상수
constexpr float VOXEL_GRID_SIZE = 0.1f;   // Voxel 필터 리프 사이즈
constexpr float CLUSTER_TOLERANCE = 0.7f;  // 클러스터링 허용 오차


//클러스터 자체가 '의미 있는' 객체임을 보장하기 위한 최소 포인트 수로, 노이즈 제거에 중점을 둡니다.
constexpr int MIN_CLUSTER_SIZE = 15;       // 최소 클러스터 포인트 수
// CropBox ROI (전방 5미터, 좌우 2미터, 높이 2미터 내의 영역)
// X: 0 ~ 5, Y: -2 ~ 2, Z: -0.4 ~ 2 (여기서는 X 최대값 8로 제한)
// 수정 후 (바닥 제거를 위해 z 최소값을 0.2로 올림):
const Eigen::Vector4f CROP_MIN(0.0, -1.0, 0.1, 0.0);
const Eigen::Vector4f CROP_MAX(1.0, 1.0, 1.0, 0.0);

// 객체 크기 조건 (바운딩 박스 생성 조건, 필요에 따라 조정)
// (여기서는 y축, z축 조건으로 사용)
const double object_min_y = 0.5; // 객체의 최소 높이  범위는 위의 Z축 바닥거르기용
const double object_max_y = 0.7; // 객체의 최대 높이 

const double object_min_x = 0.2; // 객체의 최소 너비 범위는 위의 y축
const double object_max_x = 0.6; // 객체의 최대 너비 

bool mode_allowed(const std::string &mode)
{
  return (mode == "patrol" || mode == "homing" || mode == "navigate" || mode == "manual");
}

class LidarSubscriber : public rclcpp::Node
{
public:
  LidarSubscriber()
  : Node("lidar_subscriber"),
    is_stopped_(false),
    obstacle_detected_(false),
    absent_time_recorded_(false)
  {
    // 노드 시작 시각 저장 (초기 구동 시 정지 명령 지연을 위한 기준)
    start_time_ = this->now();
    
    // robot_name 및 robot_number 파라미터 선언
    this->declare_parameter<std::string>("robot_name", "not_defined");
    this->declare_parameter<int>("robot_number", -1);
    std::string my_robot_name = this->get_parameter("robot_name").as_string();
    int my_robot_number = this->get_parameter("robot_number").as_int();
    
    // 토픽 및 서비스 이름 생성
    std::string velodyne_topic_name = "/" + my_robot_name + "/velodyne_points";
    std::string clustered_topic_name = my_robot_name + "/clustered_points";
    std::string filtered_topic_name = my_robot_name + "/filtered_points";
    std::string roi_topic_name = my_robot_name + "/roi_marker";
    std::string detected_objects_topic_name = my_robot_name + "/detected_objects";
    std::string status_topic_name = "/robot_" + std::to_string(my_robot_number) + "/status";
    
    temp_stop_service_ = "/robot_" + std::to_string(my_robot_number) + "/temp_stop";
    resume_service_    = "/robot_" + std::to_string(my_robot_number) + "/resume";

    RCLCPP_INFO(this->get_logger(), "Subscribing to topic: %s", velodyne_topic_name.c_str());

    // 포인트 클라우드 구독자 생성
    velodyne_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      velodyne_topic_name, 10,
      std::bind(&LidarSubscriber::lidar_callback, this, std::placeholders::_1)
    );

    // Status 토픽 구독 (상태 메시지: mode 업데이트)
    status_subscription_ = this->create_subscription<robot_custom_interfaces::msg::Status>(
      status_topic_name, 10,
      std::bind(&LidarSubscriber::status_callback, this, std::placeholders::_1)
    );

    // 퍼블리셔 생성
    velodyne_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(clustered_topic_name, 1);
    filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(filtered_topic_name, 1);
    roi_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(roi_topic_name, 10);
    object_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(detected_objects_topic_name, 10);

    // 서비스 클라이언트 생성
    temp_stop_client_ = this->create_client<robot_custom_interfaces::srv::Estop>(temp_stop_service_);
    resume_client_    = this->create_client<robot_custom_interfaces::srv::Estop>(resume_service_);

    // 재개 타이머 생성
    resume_timer_ = this->create_wall_timer(
      500ms, std::bind(&LidarSubscriber::check_resume, this)
    );

    RCLCPP_INFO(this->get_logger(), "Lidar Subscriber Node has been started.");
  }

private:
  // Status 메시지 콜백: 현재 모드를 업데이트
  void status_callback(const robot_custom_interfaces::msg::Status::SharedPtr msg)
  {
    current_mode_ = msg->mode;
    RCLCPP_DEBUG(this->get_logger(), "Status mode updated: %s", current_mode_.c_str());
  }

  // 포인트 클라우드 콜백
  void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // 1. ROS 메시지를 pcl::PointXYZI 타입으로 변환
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);

    // 2. Voxel Grid 필터 적용
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(VOXEL_GRID_SIZE, VOXEL_GRID_SIZE, VOXEL_GRID_SIZE);
    voxel_filter.filter(*cloud_filtered);

    // 2-1. 다운샘플링된 raw 데이터 발행
    {
      pcl::PCLPointCloud2 pcl_filtered;
      pcl::toPCLPointCloud2(*cloud_filtered, pcl_filtered);
      sensor_msgs::msg::PointCloud2 filtered_msg;
      pcl_conversions::fromPCL(pcl_filtered, filtered_msg);
      filtered_msg.header = msg->header;
      filtered_pub_->publish(filtered_msg);
    }

    // 3. CropBox 필터로 ROI 추출
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::CropBox<pcl::PointXYZI> crop_filter;
    crop_filter.setInputCloud(cloud_filtered);
    crop_filter.setMin(CROP_MIN);
    crop_filter.setMax(CROP_MAX);
    crop_filter.filter(*cloud_roi);

    // ROI 영역 시각화 (Marker)
    {
      visualization_msgs::msg::Marker roi_marker;
      roi_marker.header = msg->header;
      roi_marker.ns = "ROI";
      roi_marker.id = 0;
      roi_marker.type = visualization_msgs::msg::Marker::CUBE;
      roi_marker.action = visualization_msgs::msg::Marker::ADD;
      roi_marker.pose.position.x = (CROP_MIN[0] + CROP_MAX[0]) / 2.0;
      roi_marker.pose.position.y = (CROP_MIN[1] + CROP_MAX[1]) / 2.0;
      roi_marker.pose.position.z = (CROP_MIN[2] + CROP_MAX[2]) / 2.0;
      roi_marker.pose.orientation.w = 1.0;
      roi_marker.scale.x = CROP_MAX[0] - CROP_MIN[0];
      roi_marker.scale.y = CROP_MAX[1] - CROP_MIN[1];
      roi_marker.scale.z = CROP_MAX[2] - CROP_MIN[2];
      roi_marker.color.r = 0.0;
      roi_marker.color.g = 1.0;
      roi_marker.color.b = 0.0;
      roi_marker.color.a = 0.3;
      roi_marker.lifetime = rclcpp::Duration(0, 0);
      roi_marker_pub_->publish(roi_marker);
    }

    // 4. 클러스터링
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    if (!cloud_roi->empty()) {
      tree->setInputCloud(cloud_roi);
      // KD-트리 사용 코드
    } else {
      //RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "입력 포인트 클라우드가 비어 있습니다. KD-트리 생성 건너뛰기.");
    }

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(CLUSTER_TOLERANCE);
    ec.setMinClusterSize(MIN_CLUSTER_SIZE);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_roi);
    ec.extract(cluster_indices);

    // 5. 각 클러스터별 TotalCloud 생성 및 클러스터 id 부여
    pcl::PointCloud<pcl::PointXYZI> TotalCloud;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
    clusters.clear();
    int cluster_id = 0;
    for (const auto &indices : cluster_indices)
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
      for (const auto &idx : indices.indices)
      {
        cluster->points.push_back(cloud_roi->points[idx]);
        pcl::PointXYZI pt = cloud_roi->points[idx];
        pt.intensity = static_cast<float>(cluster_id + 1);
        TotalCloud.push_back(pt);
      }
      cluster->width = cluster->points.size();
      cluster->height = 1;
      cluster->is_dense = true;
      clusters.push_back(cluster);
      cluster_id++;
    }

    // 6. TotalCloud 발행 ("/clustered_points")
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(TotalCloud, pcl_pc2);
    sensor_msgs::msg::PointCloud2 output;
    pcl_conversions::fromPCL(pcl_pc2, output);
    output.header = msg->header;
    velodyne_pub_->publish(output);

    // 7. 각 클러스터에 대해 바운딩 박스 생성 (MarkerArray)
    visualization_msgs::msg::MarkerArray bbox_marker_array;
    int marker_id = 0;
    for (size_t i = 0; i < clusters.size(); i++)
    {
      Eigen::Vector4f centroid, min_p, max_p;
      pcl::compute3DCentroid(*clusters[i], centroid);
      pcl::getMinMax3D(*clusters[i], min_p, max_p);

      // 조건: y축(높이)와 z축(너비) 조건을 이용 (필요에 따라 조정)
      if ((max_p[1] - min_p[1]) < object_max_y && (max_p[1] - min_p[1]) > object_min_y &&
          (max_p[2] - min_p[2]) > object_min_x && (max_p[2] - min_p[2]) < object_max_x)
      {
        visualization_msgs::msg::Marker bbox_marker;
        bbox_marker.header = output.header;
        bbox_marker.ns = "object_bbox";
        bbox_marker.id = marker_id++;
        bbox_marker.type = visualization_msgs::msg::Marker::CUBE;
        bbox_marker.action = visualization_msgs::msg::Marker::ADD;
        bbox_marker.pose.position.x = (min_p[0] + max_p[0]) / 2.0;
        bbox_marker.pose.position.y = (min_p[1] + max_p[1]) / 2.0;
        bbox_marker.pose.position.z = (min_p[2] + max_p[2]) / 2.0;
        bbox_marker.pose.orientation.w = 1.0;
        bbox_marker.scale.x = max_p[0] - min_p[0];
        bbox_marker.scale.y = max_p[1] - min_p[1];
        bbox_marker.scale.z = max_p[2] - min_p[2];
        bbox_marker.color.r = 0.0;
        bbox_marker.color.g = 0.0;
        bbox_marker.color.b = 1.0;
        bbox_marker.color.a = 0.5;
        bbox_marker.lifetime = rclcpp::Duration(0, 0);
        bbox_marker_array.markers.push_back(bbox_marker);
      }
    }
    if (!bbox_marker_array.markers.empty())
    {
      object_pub_->publish(bbox_marker_array);
    }
    else
    {
      visualization_msgs::msg::MarkerArray delete_array;
      visualization_msgs::msg::Marker delete_marker;
      delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      delete_array.markers.push_back(delete_marker);
      object_pub_->publish(delete_array);
    }

    // 7. (바운딩 박스 생성 후)
    // bbox_marker_array가 비어있지 않으면 장애물이 있다고 판단합니다.
    bool obstacles_exist = !bbox_marker_array.markers.empty();

    // 8. 정지/재개 명령:
    // 장애물이 있으면 (bbox_marker_array.markers가 비어있지 않으면) 정지 조건 검사,
    // 없으면 장애물 부재 처리.
    if (obstacles_exist)
    {
      if ((this->now() - start_time_).seconds() > 5.0)
      {
        // manual모드 에서는 정지 명령을 무시
        if (current_mode_ != "manual" && mode_allowed(current_mode_))
        {
          // 장애물이 존재하면 obstacle_detected_를 true로 유지
          obstacle_detected_ = true;
          // update last_object_time_
          last_object_time_ = this->now();
          if (!is_stopped_)
          {
            RCLCPP_INFO(this->get_logger(), "Stop command conditions met (bounding boxes exist). Calling /temp_stop service.");
            call_temp_stop_service();
            is_stopped_ = true;
          }
        }
      }
    }
    else
    {
      // 장애물이 없으면, 장애물이 처음 사라진 경우에만 absent_start_time_ 기록
      if (obstacle_detected_ && !absent_time_recorded_) {
        absent_start_time_ = this->now();
        absent_time_recorded_ = true;
        RCLCPP_INFO(this->get_logger(), "Obstacles disappeared. Recording absent_start_time_.");
      }
      obstacle_detected_ = false;
      // ROI Marker 삭제 처리
      visualization_msgs::msg::Marker delete_marker;
      delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      roi_marker_pub_->publish(delete_marker);
    }
  }

  // 타이머 콜백: 장애물이 2초 이상 지속적으로 사라진 경우 재개 명령 전송
  void check_resume()
  {
    if (is_stopped_ && !obstacle_detected_)
    {
      auto absent_duration = this->now() - absent_start_time_;
      RCLCPP_INFO(this->get_logger(), "Absent duration: %f seconds", absent_duration.seconds());
      if (absent_duration.seconds() > 2.0)
      {
        RCLCPP_INFO(this->get_logger(), "No obstacles for %f seconds. Calling /resume service.", absent_duration.seconds());
        call_resume_service();
        is_stopped_ = false;
        absent_time_recorded_ = false; // 재개 후 기록 초기화
      }
    }
  }

  // /temp_stop 서비스 호출
  void call_temp_stop_service()
  {
    auto request = std::make_shared<robot_custom_interfaces::srv::Estop::Request>();
    RCLCPP_INFO(this->get_logger(), "Calling /temp_stop service");
    if (!temp_stop_client_->wait_for_service(1s))
    {
      RCLCPP_WARN(this->get_logger(), "/temp_stop service not available, retrying...");
      this->create_wall_timer(500ms, [this]() { call_temp_stop_service(); });
      return;
    }
    temp_stop_client_->async_send_request(
      request,
      [this](rclcpp::Client<robot_custom_interfaces::srv::Estop>::SharedFuture future)
      {
        try {
          auto response = future.get();
          if(response->success)
          {
            RCLCPP_INFO(this->get_logger(), "/temp_stop service call succeeded");
          }
          else
          {
            RCLCPP_ERROR(this->get_logger(), "/temp_stop service call failed, retrying...");
            call_temp_stop_service();
          }
        } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(), "/temp_stop service exception: %s, retrying...", e.what());
          call_temp_stop_service();
        }
      }
    );
  }

  // /resume 서비스 호출
  void call_resume_service()
  {
    auto request = std::make_shared<robot_custom_interfaces::srv::Estop::Request>();
    RCLCPP_INFO(this->get_logger(), "Calling /resume service");
    if (!resume_client_->wait_for_service(1s))
    {
      RCLCPP_WARN(this->get_logger(), "/resume service not available, retrying...");
      this->create_wall_timer(500ms, [this]() { call_resume_service(); });
      return;
    }
    resume_client_->async_send_request(
      request,
      [this](rclcpp::Client<robot_custom_interfaces::srv::Estop>::SharedFuture future)
      {
        try {
          auto response = future.get();
          if(response->success)
          {
            RCLCPP_INFO(this->get_logger(), "/resume service call succeeded");
          }
          else
          {
            RCLCPP_ERROR(this->get_logger(), "/resume service call failed, retrying...");
            call_resume_service();
          }
        } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(), "/resume service exception: %s, retrying...", e.what());
          call_resume_service();
        }
      }
    );
  }

  // 멤버 변수
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_subscription_;
  rclcpp::Subscription<robot_custom_interfaces::msg::Status>::SharedPtr status_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr roi_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr object_pub_;
  rclcpp::Client<robot_custom_interfaces::srv::Estop>::SharedPtr temp_stop_client_;
  rclcpp::Client<robot_custom_interfaces::srv::Estop>::SharedPtr resume_client_;
  rclcpp::TimerBase::SharedPtr resume_timer_;

  std::string temp_stop_service_;
  std::string resume_service_;

  bool is_stopped_;
  rclcpp::Time last_object_time_;
  rclcpp::Time start_time_;
  rclcpp::Time absent_start_time_; // 장애물 부재 시작 시각

  // 현재 상태 모드 (예: "patrol", "homing", "navigate", "manual" 등)
  std::string current_mode_;

  // 장애물 감지 여부 플래그
  bool obstacle_detected_;

  // 장애물 부재 시각 기록 여부
  bool absent_time_recorded_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarSubscriber>());
  rclcpp::shutdown();
  return 0;
}
