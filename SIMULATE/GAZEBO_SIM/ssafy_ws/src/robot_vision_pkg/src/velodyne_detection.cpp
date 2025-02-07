// 파일: src/velodyne_detection.cpp

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp" 
#include "visualization_msgs/msg/marker_array.hpp"
#include "robot_custom_interfaces/srv/estop.hpp"

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

// 필터 및 클러스터링 상수 (개발자가 쉽게 변경할 수 있도록 Const 처리)
constexpr float VOXEL_GRID_SIZE = 0.1f;  // Voxel 필터 리프 사이즈 (LiDAR_big_static 기준 0.1f)
constexpr float CLUSTER_TOLERANCE = 0.7f; // 클러스터링 허용 오차
constexpr int MIN_CLUSTER_SIZE = 10;      // 최소 클러스터 포인트 수

// CropBox ROI (LiDAR_big_static 코드 기준)
//전방10미터까지 좌우 2미터까지 높이 2미터 이내의 영역만 추출
const Eigen::Vector4f CROP_MIN(0.0, -2.0, -0.4, 0.0);
const Eigen::Vector4f CROP_MAX(10.0, 2.0, 2.0, 0.0);

class LidarSubscriber : public rclcpp::Node
{
public:
  LidarSubscriber()
  : Node("lidar_subscriber"),
    is_stopped_(false)
  {
    // robot_name 및 robot_number 파라미터 선언 (기본값 제공)
    this->declare_parameter<std::string>("robot_name", "not_defined");
    this->declare_parameter<int>("robot_number", -1);
    std::string my_robot_name = this->get_parameter("robot_name").as_string();
    int my_robot_number = this->get_parameter("robot_number").as_int();
    
    // 동적으로 토픽 및 서비스 이름 생성
    std::string velodyne_topic_name = "/" + my_robot_name + "/velodyne_points";
    std::string clustered_topic_name = my_robot_name + "/clustered_points";
    std::string filtered_topic_name = my_robot_name + "/filtered_points";

    std::string roi_topic_name = my_robot_name + "/roi_marker";
    std::string detected_objects_topic_name = my_robot_name + "/detected_objects";
    
    temp_stop_service_ = "/robot_" + std::to_string(my_robot_number) + "/temp_stop";
    resume_service_    = "/robot_" + std::to_string(my_robot_number) + "/resume";

    RCLCPP_INFO(this->get_logger(), "Subscribing to topic: %s", velodyne_topic_name.c_str());

    // 포인트 클라우드 구독자 생성
    velodyne_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      velodyne_topic_name, 10,
      std::bind(&LidarSubscriber::lidar_callback, this, std::placeholders::_1)
    );

    // 클러스터 결과 (TotalCloud)를 발행하는 퍼블리셔 ("/Cluster/big_static")
    velodyne_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(clustered_topic_name, 1);

    // 다운샘플링된 raw 데이터를 발행하는 퍼블리셔 ("/filtered_points")
    filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(filtered_topic_name, 1);

    // ROI Marker 퍼블리셔 ("/roi_marker")
    roi_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(roi_topic_name, 10);

    // 인식한 객체 바운딩 박스 MarkerArray 퍼블리셔 ("/bounding_boxes")
    object_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(detected_objects_topic_name, 10);

    // 서비스 클라이언트 생성 (/temp_stop, /resume)
    temp_stop_client_ = this->create_client<robot_custom_interfaces::srv::Estop>(temp_stop_service_);
    resume_client_    = this->create_client<robot_custom_interfaces::srv::Estop>(resume_service_);

    // 객체 부재 시 /resume 명령을 위한 타이머 (500ms 주기)
    resume_timer_ = this->create_wall_timer(
      500ms, std::bind(&LidarSubscriber::check_resume, this)
    );

    RCLCPP_INFO(this->get_logger(), "Lidar Subscriber Node has been started.");
  }

private:
  // 포인트 클라우드 메시지 수신 콜백 함수
  void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // 1. ROS 메시지를 intensity 채널이 있는 PCL 포인트 클라우드 (pcl::PointXYZI)로 변환
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);

    // 2. Voxel Grid 필터를 이용한 다운샘플링 (리프 사이즈: 0.1)
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(VOXEL_GRID_SIZE, VOXEL_GRID_SIZE, VOXEL_GRID_SIZE);
    voxel_filter.filter(*cloud_filtered);

    // 2-1. 다운샘플링된 raw 데이터 발행 ("/filtered_points")
    {
      pcl::PCLPointCloud2 pcl_filtered;
      pcl::toPCLPointCloud2(*cloud_filtered, pcl_filtered);
      sensor_msgs::msg::PointCloud2 filtered_msg;
      pcl_conversions::fromPCL(pcl_filtered, filtered_msg);
      filtered_msg.header = msg->header;
      filtered_pub_->publish(filtered_msg);
    }

    // 3. CropBox 필터를 이용해 전방 영역(ROI)만 추출 (min: CROP_MIN, max: CROP_MAX)
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::CropBox<pcl::PointXYZI> crop_filter;
    crop_filter.setInputCloud(cloud_filtered);
    crop_filter.setMin(CROP_MIN);
    crop_filter.setMax(CROP_MAX);
    crop_filter.filter(*cloud_roi);

    // **ROI 영역을 시각화하는 Marker 생성 및 발행**
    {
      visualization_msgs::msg::Marker roi_marker;
      roi_marker.header = msg->header;
      roi_marker.ns = "ROI";
      roi_marker.id = 0;
      roi_marker.type = visualization_msgs::msg::Marker::CUBE;
      roi_marker.action = visualization_msgs::msg::Marker::ADD;
      // ROI 중앙 계산: (CROP_MIN + CROP_MAX)/2
      roi_marker.pose.position.x = (CROP_MIN[0] + CROP_MAX[0]) / 2.0;
      roi_marker.pose.position.y = (CROP_MIN[1] + CROP_MAX[1]) / 2.0;
      roi_marker.pose.position.z = (CROP_MIN[2] + CROP_MAX[2]) / 2.0;
      roi_marker.pose.orientation.x = 0.0;
      roi_marker.pose.orientation.y = 0.0;
      roi_marker.pose.orientation.z = 0.0;
      roi_marker.pose.orientation.w = 1.0;
      // ROI 크기: CROP_MAX - CROP_MIN
      roi_marker.scale.x = CROP_MAX[0] - CROP_MIN[0];
      roi_marker.scale.y = CROP_MAX[1] - CROP_MIN[1];
      roi_marker.scale.z = CROP_MAX[2] - CROP_MIN[2];
      // 색상 설정 (연두색, 반투명)
      roi_marker.color.r = 0.0;
      roi_marker.color.g = 1.0;
      roi_marker.color.b = 0.0;
      roi_marker.color.a = 0.3;
      roi_marker.lifetime = rclcpp::Duration(0, 0); // 영구 표시
      roi_marker_pub_->publish(roi_marker);
    }

    // 4. 클러스터링: ROI 영역 내에서 KD-Tree 및 EuclideanClusterExtraction 이용
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud_roi);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(CLUSTER_TOLERANCE);
    ec.setMinClusterSize(MIN_CLUSTER_SIZE);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_roi);
    ec.extract(cluster_indices);

    // 5. 각 클러스터별로 TotalCloud 생성 (각 포인트 intensity에 클러스터 id 부여)
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

    // 6. TotalCloud를 sensor_msgs::msg::PointCloud2로 변환하여 발행 ("/Cluster/big_static")
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(TotalCloud, pcl_pc2);
    sensor_msgs::msg::PointCloud2 output;
    pcl_conversions::fromPCL(pcl_pc2, output);
    output.header = msg->header;
    velodyne_pub_->publish(output);

    // 7. 각 클러스터에 대해 (조건 만족 시) 바운딩 박스 및 중심 계산 후 Marker 생성
    //    인식한 객체(클러스터)의 크기에 맞는 바운딩 박스를 생성합니다.
    visualization_msgs::msg::MarkerArray bbox_marker_array;
    int marker_id = 0;
    for (size_t i = 0; i < clusters.size(); i++)
    {
      Eigen::Vector4f centroid, min_p, max_p;
      pcl::compute3DCentroid(*clusters[i], centroid);
      pcl::getMinMax3D(*clusters[i], min_p, max_p);

      // (조건 예시) 폭과 높이가 특정 범위일 때만 표시 (조건은 필요에 따라 수정)
      if ((max_p[1] - min_p[1]) < 3.0 && (max_p[1] - min_p[1]) > 0.8 &&
          (max_p[2] - min_p[2]) > 0.1 && (max_p[2] - min_p[2]) < 1.6)
      {
        visualization_msgs::msg::Marker bbox_marker;
        bbox_marker.header = output.header;
        bbox_marker.ns = "object_bbox";
        bbox_marker.id = marker_id++;
        bbox_marker.type = visualization_msgs::msg::Marker::CUBE;
        bbox_marker.action = visualization_msgs::msg::Marker::ADD;
        // 중심: (min + max)/2
        bbox_marker.pose.position.x = (min_p[0] + max_p[0]) / 2.0;
        bbox_marker.pose.position.y = (min_p[1] + max_p[1]) / 2.0;
        bbox_marker.pose.position.z = (min_p[2] + max_p[2]) / 2.0;
        // 회전: 기본 (0,0,0,1)
        bbox_marker.pose.orientation.x = 0.0;
        bbox_marker.pose.orientation.y = 0.0;
        bbox_marker.pose.orientation.z = 0.0;
        bbox_marker.pose.orientation.w = 1.0;
        // 크기: max - min
        bbox_marker.scale.x = max_p[0] - min_p[0];
        bbox_marker.scale.y = max_p[1] - min_p[1];
        bbox_marker.scale.z = max_p[2] - min_p[2];
        // 색상 설정 (파란색, 약간 투명)
        bbox_marker.color.r = 0.0;
        bbox_marker.color.g = 0.0;
        bbox_marker.color.b = 1.0;
        bbox_marker.color.a = 0.5;
        bbox_marker.lifetime = rclcpp::Duration(0, 0); // 영구 표시
        bbox_marker_array.markers.push_back(bbox_marker);
      }
    }
    // bbox_marker_array가 비어있지 않으면 발행, 그렇지 않으면 기존 Marker 삭제 메시지 발행
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

    // 8. (기존) 클러스터(객체)가 존재하면 /temp_stop 서비스 호출, 없으면 마커 삭제 (ROI 및 중심 표시)
    if (!clusters.empty())
    {
      last_object_time_ = this->now();
      if (!is_stopped_)
      {
        call_temp_stop_service();
        is_stopped_ = true;
      }
    }
    else
    {
      visualization_msgs::msg::Marker delete_marker;
      delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      roi_marker_pub_->publish(delete_marker);
    }
  }

  // 객체가 사라진 지 2초 이상 경과하면 /resume 서비스 호출 (타이머 콜백)
  void check_resume()
  {
    if (is_stopped_)
    {
      auto time_diff = this->now() - last_object_time_;
      if (time_diff.seconds() > 2.0)
      {
        call_resume_service();
        is_stopped_ = false;
      }
    }
  }

  // /temp_stop 서비스 호출 (응답 확인 및 실패 시 재시도)
  void call_temp_stop_service()
  {
    auto request = std::make_shared<robot_custom_interfaces::srv::Estop::Request>();
    // 요청 데이터 추가 필요 시 설정

    RCLCPP_INFO(this->get_logger(), "Calling /temp_stop service");
    if (!temp_stop_client_->wait_for_service(1s))
    {
      RCLCPP_WARN(this->get_logger(), "/temp_stop service not available, retrying...");
      this->create_wall_timer(500ms, [this]() {
        call_temp_stop_service();
      });
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

  // /resume 서비스 호출 (응답 확인 및 실패 시 재시도)
  void call_resume_service()
  {
    auto request = std::make_shared<robot_custom_interfaces::srv::Estop::Request>();
    // 요청 데이터 추가 필요 시 설정

    RCLCPP_INFO(this->get_logger(), "Calling /resume service");
    if (!resume_client_->wait_for_service(1s))
    {
      RCLCPP_WARN(this->get_logger(), "/resume service not available, retrying...");
      this->create_wall_timer(500ms, [this]() {
        call_resume_service();
      });
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
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_pub_;    // 클러스터링 결과 (TotalCloud)를 발행
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;    // 다운샘플링된 raw 데이터 (filtered_points)를 발행
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr roi_marker_pub_; // ROI Marker 발행
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr object_pub_;  // 인식한 객체의 바운딩 박스 MarkerArray 발행
  rclcpp::Client<robot_custom_interfaces::srv::Estop>::SharedPtr temp_stop_client_;
  rclcpp::Client<robot_custom_interfaces::srv::Estop>::SharedPtr resume_client_;
  rclcpp::TimerBase::SharedPtr resume_timer_;

  // 서비스 이름 (동적으로 설정)
  std::string temp_stop_service_;
  std::string resume_service_;

  // 객체 정지 상태 및 마지막 객체 검출 시간
  bool is_stopped_;
  rclcpp::Time last_object_time_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarSubscriber>());
  rclcpp::shutdown();
  return 0;
}
