//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#ifndef LIVOX_ROS_DRIVER2_LDDC_H_
#define LIVOX_ROS_DRIVER2_LDDC_H_

#include "include/livox_ros_driver2.h"

#include "driver_node.h"
#include "lds.h"

namespace livox_ros {

/** Send pointcloud message Data to ros subscriber or save them in rosbag file */
typedef enum {
  kOutputToRos = 0,
  kOutputToRosBagFile = 1,
} DestinationOfMessageOutput;

/** The message type of transfer */
typedef enum {
  kPointCloud2Msg = 0,
  kLaserScanMsg = 1,
  kLivoxCustomMsg = 2,
  kPclPxyziMsg = 3,
  kLivoxImuMsg = 4,
} TransferType;

/** Type-Definitions based on ROS versions */
#ifdef BUILDING_ROS1
using Publisher = ros::Publisher;
using PublisherPtr = ros::Publisher*;
using PointCloud2 = sensor_msgs::PointCloud2;
using PointField = sensor_msgs::PointField;
using CustomMsg = livox_ros_driver2::CustomMsg;
using CustomPoint = livox_ros_driver2::CustomPoint;
using ImuMsg = sensor_msgs::Imu;
#elif defined BUILDING_ROS2
template <typename MessageT> using Publisher = rclcpp::Publisher<MessageT>;
using PublisherPtr = std::shared_ptr<rclcpp::PublisherBase>;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;
using LaserScan = sensor_msgs::msg::LaserScan;
using CustomMsg = livox_ros_driver2::msg::CustomMsg;
using CustomPoint = livox_ros_driver2::msg::CustomPoint;
using ImuMsg = sensor_msgs::msg::Imu;
#endif

using PointCloud = pcl::PointCloud<pcl::PointXYZI>;

class DriverNode;

class Lddc final {
 public:
#ifdef BUILDING_ROS1
  Lddc(int format, int multi_topic, int data_src, int output_type, double frq,
      std::string &frame_id, bool lidar_bag, bool imu_bag);
#elif defined BUILDING_ROS2
  Lddc(int format, int multi_topic, int data_src, int output_type, double frq,
      std::string &frame_id);
#endif
  ~Lddc();

  int RegisterLds(Lds *lds);
  void DistributePointCloudData(void);
  void DistributeImuData(void);
  void CreateBagFile(const std::string &file_name);
  void PrepareExit(void);

  uint8_t GetTransferFormat(void) { return transfer_format_; }
  uint8_t IsMultiTopic(void) { return use_multi_topic_; }
  void SetRosNode(livox_ros::DriverNode *node) { cur_node_ = node; }

  // void SetRosPub(ros::Publisher *pub) { global_pub_ = pub; };  // NOT USED
  void SetPublishFrq(uint32_t frq) { publish_frq_ = frq; }

  void SetFilterParams(float range_min, float range_max, float z_min, float z_max) {
    filter_range_min_ = range_min;
    filter_range_max_ = range_max;
    filter_z_min_ = z_min;
    filter_z_max_ = z_max;
  }
  void SetLaserScanParams(float angle_min, float angle_max, float angle_increment,
                          float scan_time, bool use_inf) {
    scan_angle_min_ = angle_min;
    scan_angle_max_ = angle_max;
    scan_angle_increment_ = angle_increment;
    scan_time_ = scan_time;
    scan_use_inf_ = use_inf;
  }

 public:
  Lds *lds_;

 private:
  void PollingLidarPointCloudData(uint8_t index, LidarDevice *lidar);
  void PollingLidarImuData(uint8_t index, LidarDevice *lidar);

  void PublishPointcloud2(LidarDataQueue *queue, uint8_t index);
  void PublishLaserScan(LidarDataQueue *queue, uint8_t index);
  void PublishCustomPointcloud(LidarDataQueue *queue, uint8_t index);
  void PublishPclMsg(LidarDataQueue *queue, uint8_t index);

  void PublishImuData(LidarImuDataQueue& imu_data_queue, const uint8_t index);

  void InitPointcloud2MsgHeader(PointCloud2& cloud);
  void InitPointcloud2Msg(const StoragePacket& pkg, PointCloud2& cloud, uint64_t& timestamp);
  void PublishPointcloud2Data(const uint8_t index, uint64_t timestamp, const PointCloud2& cloud);

  void InitLaserScanMsg(const StoragePacket& pkg, LaserScan& scan, uint64_t& timestamp);
  void PublishLaserScanData(const uint8_t index, const LaserScan& scan);

  void InitCustomMsg(CustomMsg& livox_msg, const StoragePacket& pkg, uint8_t index);
  void FillPointsToCustomMsg(CustomMsg& livox_msg, const StoragePacket& pkg);
  void PublishCustomPointData(const CustomMsg& livox_msg, const uint8_t index);

  void InitPclMsg(const StoragePacket& pkg, PointCloud& cloud, uint64_t& timestamp);
  void FillPointsToPclMsg(const StoragePacket& pkg, PointCloud& pcl_msg);
  void PublishPclData(const uint8_t index, const uint64_t timestamp, const PointCloud& cloud);

  void InitImuMsg(const ImuData& imu_data, ImuMsg& imu_msg, uint64_t& timestamp);

  void FillPointsToPclMsg(PointCloud& pcl_msg, LivoxPointXyzrtlt* src_point, uint32_t num);
  void FillPointsToCustomMsg(CustomMsg& livox_msg, LivoxPointXyzrtlt* src_point, uint32_t num,
      uint32_t offset_time, uint32_t point_interval, uint32_t echo_num);

#ifdef BUILDING_ROS2
  PublisherPtr CreatePublisher(uint8_t msg_type, std::string &topic_name, uint32_t queue_size);
#endif

  PublisherPtr GetCurrentPublisher(uint8_t index);
  PublisherPtr GetCurrentImuPublisher(uint8_t index);

 private:
  uint8_t transfer_format_;
  uint8_t use_multi_topic_;
  uint8_t data_src_;
  uint8_t output_type_;
  double publish_frq_;
  uint32_t publish_period_ns_;
  std::string frame_id_;

#ifdef BUILDING_ROS1
  bool enable_lidar_bag_;
  bool enable_imu_bag_;
  PublisherPtr private_pub_[kMaxSourceLidar];
  PublisherPtr global_pub_;
  PublisherPtr private_imu_pub_[kMaxSourceLidar];
  PublisherPtr global_imu_pub_;
  rosbag::Bag *bag_;
#elif defined BUILDING_ROS2
  PublisherPtr private_pub_[kMaxSourceLidar];
  PublisherPtr global_pub_;
  PublisherPtr private_imu_pub_[kMaxSourceLidar];
  PublisherPtr global_imu_pub_;
#endif

  livox_ros::DriverNode *cur_node_;

  float filter_range_min_ = 0.05f;
  float filter_range_max_ = 5.0f;
  float filter_z_min_ = -0.30f;
  float filter_z_max_ = 1.20f;
  float scan_angle_min_ = -3.1415927f;
  float scan_angle_max_ =  3.1415927f;
  float scan_angle_increment_ = 0.0087f;
  float scan_time_ = 0.0333f;
  bool  scan_use_inf_ = true;
};

}  // namespace livox_ros

#endif // LIVOX_ROS_DRIVER2_LDDC_H_
