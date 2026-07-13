#include "lddc.h"
#include "comm/ldq.h"
#include "comm/comm.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "include/ros_headers.h"

#include "driver_node.h"
#include "lds_lidar.h"

namespace livox_ros
{

  Lddc::Lddc(int format, int multi_topic, std::string &frame_id)
      : transfer_format_(format),
        use_multi_topic_(multi_topic),
        frame_id_(frame_id)
  {
    lds_ = nullptr;
  }

  Lddc::~Lddc()
  {
    PrepareExit();
  }

  int Lddc::RegisterLds(Lds *lds)
  {
    if (lds_ == nullptr)
    {
      lds_ = lds;
      return 0;
    }
    return -1;
  }

  void Lddc::DistributePointCloudData(void)
  {
    if (!lds_ || lds_->IsRequestExit())
    {
      return;
    }

    lds_->pcd_semaphore_.Wait();
    for (uint32_t i = 0; i < lds_->lidar_count_; i++)
    {
      LidarDevice *lidar = &lds_->lidars_[i];
      LidarDataQueue *p_queue = &lidar->data;
      if ((kConnectStateSampling != lidar->connect_state) || (p_queue == nullptr))
      {
        continue;
      }
      PollingLidarPointCloudData(i, lidar);
    }
  }

  void Lddc::DistributeImuData(void)
  {
    if (!lds_ || lds_->IsRequestExit())
    {
      return;
    }

    lds_->imu_semaphore_.Wait();
    for (uint32_t i = 0; i < lds_->lidar_count_; i++)
    {
      LidarDevice *lidar = &lds_->lidars_[i];
      LidarImuDataQueue *p_queue = &lidar->imu_data;
      if ((kConnectStateSampling != lidar->connect_state) || (p_queue == nullptr))
      {
        continue;
      }
      PollingLidarImuData(i, lidar);
    }
  }

  void Lddc::PollingLidarPointCloudData(uint8_t index, LidarDevice *lidar)
  {
    LidarDataQueue *p_queue = &lidar->data;
    if (p_queue == nullptr || p_queue->storage_packet == nullptr)
    {
      return;
    }

    while (!lds_->IsRequestExit() && !QueueIsEmpty(p_queue))
    {
      if (kPointCloud2Msg == transfer_format_)
      {
        PublishPointcloud2(p_queue, index);
      }
      else if (kLivoxCustomMsg == transfer_format_)
      {
        PublishCustomPointcloud(p_queue, index);
      }
    }
  }

  void Lddc::PollingLidarImuData(uint8_t index, LidarDevice *lidar)
  {
    LidarImuDataQueue &p_queue = lidar->imu_data;
    while (!lds_->IsRequestExit() && !p_queue.Empty())
    {
      PublishImuData(p_queue, index);
    }
  }

  void Lddc::PrepareExit(void)
  {
    if (lds_)
    {
      lds_->PrepareExit();
      lds_ = nullptr;
    }
  }

  void Lddc::PublishPointcloud2(LidarDataQueue *queue, uint8_t index)
  {
    while (!QueueIsEmpty(queue))
    {
      StoragePacket pkg;
      QueuePop(queue, &pkg);
      if (pkg.points.empty())
      {
        continue;
      }

      auto cloud = std::make_unique<PointCloud2>();
      uint64_t timestamp = 0;
      InitPointcloud2Msg(pkg, *cloud, timestamp);
      PublishPointcloud2Data(index, std::move(cloud));
    }
  }

  void Lddc::PublishCustomPointcloud(LidarDataQueue *queue, uint8_t index)
  {
    while (!QueueIsEmpty(queue))
    {
      StoragePacket pkg;
      QueuePop(queue, &pkg);
      if (pkg.points.empty())
      {
        continue;
      }

      auto livox_msg = std::make_unique<CustomMsg>();
      InitCustomMsg(*livox_msg, pkg, index);
      FillPointsToCustomMsg(*livox_msg, pkg);
      PublishCustomPointData(std::move(livox_msg), index);
    }
  }

  void Lddc::InitPointcloud2MsgHeader(PointCloud2 &cloud)
  {
    cloud.header.frame_id.assign(frame_id_);
    cloud.height = 1;
    cloud.width = 0;
    cloud.fields.resize(7);
    cloud.fields[0].offset = 0;
    cloud.fields[0].name = "x";
    cloud.fields[0].count = 1;
    cloud.fields[0].datatype = PointField::FLOAT32;
    cloud.fields[1].offset = 4;
    cloud.fields[1].name = "y";
    cloud.fields[1].count = 1;
    cloud.fields[1].datatype = PointField::FLOAT32;
    cloud.fields[2].offset = 8;
    cloud.fields[2].name = "z";
    cloud.fields[2].count = 1;
    cloud.fields[2].datatype = PointField::FLOAT32;
    cloud.fields[3].offset = 12;
    cloud.fields[3].name = "intensity";
    cloud.fields[3].count = 1;
    cloud.fields[3].datatype = PointField::FLOAT32;
    cloud.fields[4].offset = 16;
    cloud.fields[4].name = "tag";
    cloud.fields[4].count = 1;
    cloud.fields[4].datatype = PointField::UINT8;
    cloud.fields[5].offset = 17;
    cloud.fields[5].name = "line";
    cloud.fields[5].count = 1;
    cloud.fields[5].datatype = PointField::UINT8;
    cloud.fields[6].offset = 18;
    cloud.fields[6].name = "timestamp";
    cloud.fields[6].count = 1;
    cloud.fields[6].datatype = PointField::FLOAT64;
    cloud.point_step = sizeof(LivoxPointXyzrtlt);
  }

  bool Lddc::PointPassFilter(float x, float y, float z) const
  {
    if (!yaml_config_.loaded || !yaml_config_.filters.enable)
    {
      return true;
    }
    const YamlFilterConfig &f = yaml_config_.filters;
    if (z < f.min_height || z > f.max_height)
    {
      return false;
    }
    const double range = std::sqrt(static_cast<double>(x) * x +
                                   static_cast<double>(y) * y);
    if (range < f.min_range || range > f.max_range)
    {
      return false;
    }
    return true;
  }

  void Lddc::InitPointcloud2Msg(const StoragePacket &pkg, PointCloud2 &cloud, uint64_t &timestamp)
  {
    InitPointcloud2MsgHeader(cloud);

    cloud.point_step = sizeof(LivoxPointXyzrtlt);
    cloud.is_bigendian = false;
    cloud.is_dense = true;

    if (!pkg.points.empty())
    {
      timestamp = pkg.base_time;
    }
    cloud.header.stamp = rclcpp::Time(timestamp);

    std::vector<LivoxPointXyzrtlt> points;
    points.reserve(pkg.points_num);
    for (size_t i = 0; i < pkg.points_num; ++i)
    {
      if (!PointPassFilter(pkg.points[i].x, pkg.points[i].y, pkg.points[i].z))
      {
        continue;
      }
      LivoxPointXyzrtlt point;
      point.x = pkg.points[i].x;
      point.y = pkg.points[i].y;
      point.z = pkg.points[i].z;
      point.reflectivity = pkg.points[i].intensity;
      point.tag = pkg.points[i].tag;
      point.line = pkg.points[i].line;
      point.timestamp = static_cast<double>(pkg.points[i].offset_time);
      points.push_back(std::move(point));
    }

    cloud.width = static_cast<uint32_t>(points.size());
    cloud.row_step = cloud.width * cloud.point_step;
    cloud.data.resize(points.size() * sizeof(LivoxPointXyzrtlt));
    memcpy(cloud.data.data(), points.data(), points.size() * sizeof(LivoxPointXyzrtlt));
  }

  void Lddc::PublishPointcloud2Data(const uint8_t index, std::unique_ptr<PointCloud2> cloud)
  {
    Publisher<PointCloud2>::SharedPtr publisher_ptr =
        std::dynamic_pointer_cast<Publisher<PointCloud2>>(GetCurrentPublisher(index));
    publisher_ptr->publish(std::move(cloud));
  }

  void Lddc::InitCustomMsg(CustomMsg &livox_msg, const StoragePacket &pkg, uint8_t index)
  {
    livox_msg.header.frame_id.assign(frame_id_);

    uint64_t timestamp = 0;
    if (!pkg.points.empty())
    {
      timestamp = pkg.base_time;
    }
    livox_msg.timebase = timestamp;
    livox_msg.header.stamp = rclcpp::Time(timestamp);

    livox_msg.point_num = pkg.points_num;
    if (lds_->lidars_[index].lidar_type == kLivoxLidarType)
    {
      livox_msg.lidar_id = lds_->lidars_[index].handle;
    }
    else
    {
      livox_msg.lidar_id = 0;
    }
  }

  void Lddc::FillPointsToCustomMsg(CustomMsg &livox_msg, const StoragePacket &pkg)
  {
    uint32_t points_num = pkg.points_num;
    const std::vector<PointXyzlt> &points = pkg.points;
    for (uint32_t i = 0; i < points_num; ++i)
    {
      if (!PointPassFilter(points[i].x, points[i].y, points[i].z))
      {
        continue;
      }
      CustomPoint point;
      point.x = points[i].x;
      point.y = points[i].y;
      point.z = points[i].z;
      point.reflectivity = points[i].intensity;
      point.tag = points[i].tag;
      point.line = points[i].line;
      point.offset_time = static_cast<uint32_t>(points[i].offset_time - pkg.base_time);
      livox_msg.points.push_back(std::move(point));
    }
    livox_msg.point_num = static_cast<uint32_t>(livox_msg.points.size());
  }

  void Lddc::PublishCustomPointData(std::unique_ptr<CustomMsg> livox_msg, const uint8_t index)
  {
    Publisher<CustomMsg>::SharedPtr publisher_ptr =
        std::dynamic_pointer_cast<Publisher<CustomMsg>>(GetCurrentPublisher(index));
    publisher_ptr->publish(std::move(livox_msg));
  }

  void Lddc::InitImuMsg(const ImuData &imu_data, ImuMsg &imu_msg, uint64_t &timestamp)
  {
    imu_msg.header.frame_id = "livox_frame";

    timestamp = imu_data.time_stamp;
    imu_msg.header.stamp = rclcpp::Time(timestamp);

    imu_msg.angular_velocity.x = imu_data.gyro_x;
    imu_msg.angular_velocity.y = imu_data.gyro_y;
    imu_msg.angular_velocity.z = imu_data.gyro_z;
    imu_msg.linear_acceleration.x = imu_data.acc_x;
    imu_msg.linear_acceleration.y = imu_data.acc_y;
    imu_msg.linear_acceleration.z = imu_data.acc_z;
  }

  void Lddc::PublishImuData(LidarImuDataQueue &imu_data_queue, const uint8_t index)
  {
    ImuData imu_data;
    if (!imu_data_queue.Pop(imu_data))
    {
      return;
    }

    auto imu_msg = std::make_unique<ImuMsg>();
    uint64_t timestamp;
    InitImuMsg(imu_data, *imu_msg, timestamp);

    Publisher<ImuMsg>::SharedPtr publisher_ptr =
        std::dynamic_pointer_cast<Publisher<ImuMsg>>(GetCurrentImuPublisher(index));
    publisher_ptr->publish(std::move(imu_msg));
  }

  rclcpp::QoS Lddc::BuildQos(uint32_t depth)
  {
    return rclcpp::SensorDataQoS(rclcpp::KeepLast(depth));
  }

  std::shared_ptr<rclcpp::PublisherBase> Lddc::CreatePublisher(uint8_t msg_type,
                                                               std::string &topic_name, uint32_t queue_size)
  {
    const rclcpp::QoS qos = BuildQos(queue_size);
    if (kPointCloud2Msg == msg_type)
    {
      return cur_node_->create_publisher<PointCloud2>(topic_name, qos);
    }
    else if (kLivoxCustomMsg == msg_type)
    {
      return cur_node_->create_publisher<CustomMsg>(topic_name, qos);
    }
    else if (kLivoxImuMsg == msg_type)
    {
      return cur_node_->create_publisher<ImuMsg>(topic_name, qos);
    }
    return PublisherPtr(nullptr);
  }

  std::shared_ptr<rclcpp::PublisherBase> Lddc::GetCurrentPublisher(uint8_t handle)
  {
    uint32_t queue_size = kMinEthPacketQueueSize;
    if (use_multi_topic_)
    {
      if (!private_pub_[handle])
      {
        char name_str[48];
        memset(name_str, 0, sizeof(name_str));
        std::string ip_string = IpNumToString(lds_->lidars_[handle].handle);
        snprintf(name_str, sizeof(name_str), "livox/lidar_%s",
                 ReplacePeriodByUnderline(ip_string).c_str());
        std::string topic_name(name_str);
        queue_size = 20;
        private_pub_[handle] = CreatePublisher(transfer_format_, topic_name, queue_size);
      }
      return private_pub_[handle];
    }
    if (!global_pub_)
    {
      std::string topic_name = yaml_config_.point_cloud_topic.empty()
                                   ? std::string("livox/lidar")
                                   : yaml_config_.point_cloud_topic;
      queue_size = 20;
      global_pub_ = CreatePublisher(transfer_format_, topic_name, queue_size);
    }
    return global_pub_;
  }

  std::shared_ptr<rclcpp::PublisherBase> Lddc::GetCurrentImuPublisher(uint8_t handle)
  {
    uint32_t queue_size = kMinEthPacketQueueSize;
    if (use_multi_topic_)
    {
      if (!private_imu_pub_[handle])
      {
        char name_str[48];
        memset(name_str, 0, sizeof(name_str));
        std::string ip_string = IpNumToString(lds_->lidars_[handle].handle);
        snprintf(name_str, sizeof(name_str), "livox/imu_%s",
                 ReplacePeriodByUnderline(ip_string).c_str());
        std::string topic_name(name_str);
        queue_size = 10;
        private_imu_pub_[handle] = CreatePublisher(kLivoxImuMsg, topic_name, queue_size);
      }
      return private_imu_pub_[handle];
    }
    if (!global_imu_pub_)
    {
      std::string topic_name = yaml_config_.imu_topic.empty()
                                   ? std::string("livox/imu")
                                   : yaml_config_.imu_topic;
      queue_size = 10;
      global_imu_pub_ = CreatePublisher(kLivoxImuMsg, topic_name, queue_size);
    }
    return global_imu_pub_;
  }

}
