#ifndef LIVOX_ROS_DRIVER_LIDAR_IMU_DATA_QUEUE_H_
#define LIVOX_ROS_DRIVER_LIDAR_IMU_DATA_QUEUE_H_

#include <list>
#include <mutex>
#include <cstdint>

namespace livox_ros
{

  typedef struct
  {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
  } RawImuPoint;

  typedef struct
  {
    uint8_t lidar_type;
    uint32_t handle;
    uint8_t slot;

    uint64_t time_stamp;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
  } ImuData;

  class LidarImuDataQueue
  {
  public:
    void Push(ImuData *imu_data);
    bool Pop(ImuData &imu_data);
    bool Empty();
    void Clear();

  private:
    std::mutex mutex_;
    std::list<ImuData> imu_data_queue_;
  };

}

#endif
