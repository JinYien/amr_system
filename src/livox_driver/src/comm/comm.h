#ifndef LIVOX_DRIVER_COMM_H_
#define LIVOX_DRIVER_COMM_H_

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <map>

#include "lidar_imu_data_queue.h"

namespace livox_ros
{

  const uint8_t kMaxSourceLidar = 32;

  const uint32_t kMaxPointPerEthPacket = 100;
  const uint32_t kMinEthPacketQueueSize = 32;
  const uint32_t kMaxEthPacketQueueSize = 131072;
  const uint32_t kImuEthPacketQueueSize = 256;

  const uint32_t KEthPacketMaxLength = 1500;
  const uint32_t KEthPacketHeaderLength = 18;
  const uint32_t KCartesianPointSize = 13;
  const uint32_t KSphericalPointSzie = 9;

  const uint64_t kRosTimeMax = 4294967296000000000;
  const int64_t kPacketTimeGap = 1000000;

  const int64_t kMaxPacketTimeGap = 1700000;

  const int64_t kDeviceDisconnectThreshold = 1000000000;
  const uint32_t kNsPerSecond = 1000000000;
  const uint32_t kNsTolerantFrameTimeDeviation = 1000000;
  const uint32_t kRatioOfMsToNs = 1000000;

  const int kPathStrMinSize = 4;
  const int kPathStrMaxSize = 256;
  const int kBdCodeSize = 15;

  const uint32_t kPointXYZRSize = 16;
  const uint32_t kPointXYZRTRSize = 18;

  const double PI = 3.14159265358979323846;

  constexpr uint32_t kMaxBufferSize = 0x8000;

  const uint8_t kLineNumberDefault = 1;
  const uint8_t kLineNumberMid360 = 4;

  typedef enum
  {
    kIndustryLidarType = 1,
    kVehicleLidarType = 2,
    kDirectLidarType = 4,
    kLivoxLidarType = 8
  } LidarProtoType;

  typedef enum
  {
    kTimestampTypeNoSync = 0,
    kTimestampTypeGptpOrPtp = 1,
    kTimestampTypeGps = 2
  } TimestampType;

  typedef enum
  {
    kConnectStateOff = 0,
    kConnectStateOn = 1,
    kConnectStateConfig = 2,
    kConnectStateSampling = 3,
  } LidarConnectState;

  typedef enum
  {
    kSourceRawLidar = 0,
    kSourceRawHub = 1,
    kSourceLvxFile,
    kSourceUndef,
  } LidarDataSourceType;

  typedef enum
  {
    kCoordinateCartesian = 0,
    kCoordinateSpherical
  } CoordinateType;

  typedef enum
  {
    kConfigDataType = 1 << 0,
    kConfigScanPattern = 1 << 1,
    kConfigBlindSpot = 1 << 2,
    kConfigDualEmit = 1 << 3,
    kConfigUnknown
  } LivoxLidarConfigCodeBit;

  typedef enum
  {
    kNoneExtrinsicParameter,
    kExtrinsicParameterFromLidar,
    kExtrinsicParameterFromXml
  } ExtrinsicParameterType;

  typedef struct
  {
    uint8_t lidar_type{};
  } LidarSummaryInfo;

  typedef union
  {
    struct
    {
      uint32_t low;
      uint32_t high;
    } stamp_word;

    uint8_t stamp_bytes[8];
    int64_t stamp;
  } LdsStamp;

#pragma pack(1)

  typedef struct
  {
    float x;
    float y;
    float z;
    float reflectivity;
    uint8_t tag;
    uint8_t line;
    double timestamp;
  } LivoxPointXyzrtlt;

  typedef struct
  {
    float x;
    float y;
    float z;
    float intensity;
    uint8_t tag;
    uint8_t line;
    uint64_t offset_time;
  } PointXyzlt;

  typedef struct
  {
    uint32_t handle;
    uint8_t lidar_type;
    uint32_t points_num;
    PointXyzlt *points;
  } PointPacket;

  typedef struct
  {
    uint64_t base_time[kMaxSourceLidar]{};
    uint8_t lidar_num{};
    PointPacket lidar_point[kMaxSourceLidar]{};
  } PointFrame;

#pragma pack()

  typedef struct
  {
    LidarProtoType lidar_type;
    uint32_t handle;
    uint64_t base_time;
    uint32_t points_num;
    std::vector<PointXyzlt> points;
  } StoragePacket;

  typedef struct
  {
    LidarProtoType lidar_type;
    uint32_t handle;
    bool extrinsic_enable;
    uint32_t point_num;
    uint8_t data_type;
    uint8_t line_num;
    uint64_t time_stamp;
    uint64_t point_interval;
    std::vector<uint8_t> raw_data;
  } RawPacket;

  typedef struct
  {
    StoragePacket *storage_packet;
    volatile uint32_t rd_idx;
    volatile uint32_t wr_idx;
    uint32_t mask;
    uint32_t size;
  } LidarDataQueue;

  typedef struct
  {
    float roll;
    float pitch;
    float yaw;
    int32_t x;
    int32_t y;
    int32_t z;
  } ExtParameter;

  typedef float TranslationVector[3];
  typedef float RotationMatrix[3][3];

  typedef struct
  {
    TranslationVector trans;
    RotationMatrix rotation;
  } ExtParameterDetailed;

  typedef struct
  {
    LidarProtoType lidar_type;
    uint32_t handle;
    ExtParameter param;
  } LidarExtParameter;

  typedef struct
  {
    char broadcast_code[16];
    bool enable_connect;
    bool enable_fan;
    uint32_t return_mode;
    uint32_t coordinate;
    uint32_t imu_rate;
    uint32_t extrinsic_parameter_source;
    bool enable_high_sensitivity;
  } UserRawConfig;

  typedef struct
  {
    bool enable_fan;
    uint32_t return_mode;
    uint32_t coordinate;
    uint32_t imu_rate;
    uint32_t extrinsic_parameter_source;
    bool enable_high_sensitivity;
    volatile uint32_t set_bits;
    volatile uint32_t get_bits;
  } UserConfig;

  typedef struct
  {
    uint32_t handle;
    int8_t pcl_data_type;
    int8_t pattern_mode;
    int32_t blind_spot_set;
    int8_t dual_emit_en;
    ExtParameter extrinsic_param;
    volatile uint32_t set_bits;
    volatile uint32_t get_bits;
  } UserLivoxLidarConfig;

  typedef struct
  {
    uint8_t lidar_type;
    uint32_t handle;

    uint8_t data_src;
    volatile LidarConnectState connect_state;

    LidarDataQueue data;
    LidarImuDataQueue imu_data;

    uint32_t firmware_ver;
    UserLivoxLidarConfig livox_config;
  } LidarDevice;

  bool IsFilePathValid(const char *path_str);
  uint32_t CalculatePacketQueueSize(const double publish_freq);
  std::string IpNumToString(uint32_t ip_num);
  uint32_t IpStringToNum(std::string ip_string);
  std::string ReplacePeriodByUnderline(std::string str);

}

#endif
