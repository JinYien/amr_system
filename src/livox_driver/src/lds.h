#ifndef LIVOX_ROS_DRIVER_LDS_H_
#define LIVOX_ROS_DRIVER_LDS_H_

#include <map>

#include "comm/semaphore.h"
#include "comm/comm.h"
#include "comm/cache_index.h"

namespace livox_ros
{

  class Lds
  {
  public:
    Lds(const double publish_freq, const uint8_t data_src);
    virtual ~Lds();

    void StorageImuData(ImuData *imu_data);
    void StoragePointData(PointFrame *frame);
    void StorageLvxPointData(PointFrame *frame);

    int8_t GetHandle(const uint8_t lidar_type, const PointPacket *lidar_point);
    void PushLidarData(PointPacket *lidar_data, const uint8_t index, const uint64_t base_time);

    static void ResetLidar(LidarDevice *lidar, uint8_t data_src);
    static void SetLidarDataSrc(LidarDevice *lidar, uint8_t data_src);
    void ResetLds(uint8_t data_src);

    void RequestExit();

    bool IsAllQueueEmpty();
    bool IsAllQueueReadStop();

    void CleanRequestExit() { request_exit_ = false; }
    bool IsRequestExit() { return request_exit_; }
    virtual void PrepareExit(void);

    double GetLdsFrequency() { return publish_freq_; }

  public:
    uint8_t lidar_count_;
    LidarDevice lidars_[kMaxSourceLidar];
    Semaphore pcd_semaphore_;
    Semaphore imu_semaphore_;
    static CacheIndex cache_index_;

  protected:
    double publish_freq_;
    uint8_t data_src_;

  private:
    volatile bool request_exit_;
  };

}

#endif
