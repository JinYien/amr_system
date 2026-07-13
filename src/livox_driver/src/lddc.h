#ifndef LIVOX_DRIVER_LDDC_H_
#define LIVOX_DRIVER_LDDC_H_

#include <memory>

#include "driver_node.h"
#include "lds.h"
#include "parse_cfg_file/parse_yaml_config.h"

namespace livox_ros
{

  typedef enum
  {
    kPointCloud2Msg = 0,
    kLivoxCustomMsg = 1,
    kLivoxImuMsg = 3,
  } TransferType;

  template <typename MessageT>
  using Publisher = rclcpp::Publisher<MessageT>;
  using PublisherPtr = std::shared_ptr<rclcpp::PublisherBase>;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PointField = sensor_msgs::msg::PointField;
  using CustomMsg = livox_driver::msg::CustomMsg;
  using CustomPoint = livox_driver::msg::CustomPoint;
  using ImuMsg = sensor_msgs::msg::Imu;

  class DriverNode;

  class Lddc final
  {
  public:
    Lddc(int format, int multi_topic, std::string &frame_id);
    ~Lddc();

    int RegisterLds(Lds *lds);
    void DistributePointCloudData(void);
    void DistributeImuData(void);
    void PrepareExit(void);

    void SetRosNode(livox_ros::DriverNode *node) { cur_node_ = node; }
    void SetYamlConfig(const YamlConfig &config) { yaml_config_ = config; }

    Lds *lds_;

  private:
    void PollingLidarPointCloudData(uint8_t index, LidarDevice *lidar);
    void PollingLidarImuData(uint8_t index, LidarDevice *lidar);

    void PublishPointcloud2(LidarDataQueue *queue, uint8_t index);
    void PublishCustomPointcloud(LidarDataQueue *queue, uint8_t index);
    void PublishImuData(LidarImuDataQueue &imu_data_queue, const uint8_t index);

    void InitPointcloud2MsgHeader(PointCloud2 &cloud);
    void InitPointcloud2Msg(const StoragePacket &pkg, PointCloud2 &cloud, uint64_t &timestamp);
    void PublishPointcloud2Data(const uint8_t index, std::unique_ptr<PointCloud2> cloud);

    void InitCustomMsg(CustomMsg &livox_msg, const StoragePacket &pkg, uint8_t index);
    void FillPointsToCustomMsg(CustomMsg &livox_msg, const StoragePacket &pkg);
    void PublishCustomPointData(std::unique_ptr<CustomMsg> livox_msg, const uint8_t index);

    void InitImuMsg(const ImuData &imu_data, ImuMsg &imu_msg, uint64_t &timestamp);

    PublisherPtr CreatePublisher(uint8_t msg_type, std::string &topic_name, uint32_t queue_size);
    rclcpp::QoS BuildQos(uint32_t default_depth);
    bool PointPassFilter(float x, float y, float z) const;

    PublisherPtr GetCurrentPublisher(uint8_t index);
    PublisherPtr GetCurrentImuPublisher(uint8_t index);

    uint8_t transfer_format_;
    uint8_t use_multi_topic_;
    std::string frame_id_;

    PublisherPtr private_pub_[kMaxSourceLidar];
    PublisherPtr global_pub_;
    PublisherPtr private_imu_pub_[kMaxSourceLidar];
    PublisherPtr global_imu_pub_;

    YamlConfig yaml_config_;

    livox_ros::DriverNode *cur_node_;
  };

}

#endif
