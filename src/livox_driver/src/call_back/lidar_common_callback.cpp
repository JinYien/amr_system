#include "lidar_common_callback.h"

#include "../lds_lidar.h"

#include <string>

namespace livox_ros
{

  void LidarCommonCallback::OnLidarPointClounCb(PointFrame *frame, void *client_data)
  {
    if (frame == nullptr)
    {
      printf("LidarPointCloudCb frame is nullptr.\n");
      return;
    }

    if (client_data == nullptr)
    {
      printf("Lidar point cloud cb failed, client data is nullptr.\n");
      return;
    }

    if (frame->lidar_num == 0)
    {
      return;
    }

    LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);

    lds_lidar->StoragePointData(frame);
  }

  void LidarCommonCallback::LidarImuDataCallback(ImuData *imu_data, void *client_data)
  {
    if (imu_data == nullptr)
    {
      printf("Imu data is nullptr.\n");
      return;
    }
    if (client_data == nullptr)
    {
      printf("Lidar point cloud cb failed, client data is nullptr.\n");
      return;
    }

    LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);
    lds_lidar->StorageImuData(imu_data);
  }

}
