#include "livox_lidar_callback.h"

#include "livox_lidar_api.h"
#include <string>
#include <thread>
#include <iostream>

namespace livox_ros
{

  void LivoxLidarCallback::LidarInfoChangeCallback(const uint32_t handle,
                                                   const LivoxLidarInfo *info,
                                                   void *client_data)
  {
    if (client_data == nullptr)
    {
      std::cout << "lidar info change callback failed, client data is nullptr" << std::endl;
      return;
    }
    LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);

    LidarDevice *lidar_device = GetLidarDevice(handle, client_data);
    if (lidar_device == nullptr)
    {
      uint8_t index = 0;
      int8_t ret = lds_lidar->cache_index_.GetFreeIndex(kLivoxLidarType, handle, index);
      if (ret != 0)
      {
        std::cout << "failed to add lidar device, lidar ip: " << IpNumToString(handle) << std::endl;
        return;
      }
      LidarDevice *p_lidar = &(lds_lidar->lidars_[index]);
      p_lidar->lidar_type = kLivoxLidarType;
    }
    else
    {
      const UserLivoxLidarConfig &config = lidar_device->livox_config;

      {
        std::lock_guard<std::mutex> lock(lds_lidar->config_mutex_);
        if (config.pcl_data_type != -1)
        {
          lidar_device->livox_config.set_bits |= kConfigDataType;
          SetLivoxLidarPclDataType(handle, static_cast<LivoxLidarPointDataType>(config.pcl_data_type),
                                   LivoxLidarCallback::SetDataTypeCallback, lds_lidar);
        }
        if (config.pattern_mode != -1)
        {
          lidar_device->livox_config.set_bits |= kConfigScanPattern;
          SetLivoxLidarScanPattern(handle, static_cast<LivoxLidarScanPattern>(config.pattern_mode),
                                   LivoxLidarCallback::SetPatternModeCallback, lds_lidar);
        }
        if (config.blind_spot_set != -1)
        {
          lidar_device->livox_config.set_bits |= kConfigBlindSpot;
          SetLivoxLidarBlindSpot(handle, config.blind_spot_set,
                                 LivoxLidarCallback::SetBlindSpotCallback, lds_lidar);
        }
        if (config.dual_emit_en != -1)
        {
          lidar_device->livox_config.set_bits |= kConfigDualEmit;
          SetLivoxLidarDualEmit(handle, (config.dual_emit_en == 0 ? false : true),
                                LivoxLidarCallback::SetDualEmitCallback, lds_lidar);
        }
      }

      LivoxLidarInstallAttitude attitude{
          config.extrinsic_param.roll,
          config.extrinsic_param.pitch,
          config.extrinsic_param.yaw,
          config.extrinsic_param.x,
          config.extrinsic_param.y,
          config.extrinsic_param.z};
      SetLivoxLidarInstallAttitude(config.handle, &attitude,
                                   LivoxLidarCallback::SetAttitudeCallback, lds_lidar);
    }

    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeChangedCallback, nullptr);
    EnableLivoxLidarImuData(handle, LivoxLidarCallback::EnableLivoxLidarImuDataCallback, lds_lidar);
    return;
  }

  void LivoxLidarCallback::WorkModeChangedCallback(livox_status status,
                                                   uint32_t handle,
                                                   LivoxLidarAsyncControlResponse *response,
                                                   void *client_data)
  {
    if (status != kLivoxLidarStatusSuccess)
    {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeChangedCallback, nullptr);
      return;
    }
    return;
  }

  void LivoxLidarCallback::SetDataTypeCallback(livox_status status, uint32_t handle,
                                               LivoxLidarAsyncControlResponse *response,
                                               void *client_data)
  {
    LidarDevice *lidar_device = GetLidarDevice(handle, client_data);
    if (lidar_device == nullptr)
    {
      std::cout << "failed to set data type since no lidar device found, handle: "
                << handle << std::endl;
      return;
    }
    LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);

    if (status == kLivoxLidarStatusSuccess)
    {
      std::lock_guard<std::mutex> lock(lds_lidar->config_mutex_);
      lidar_device->livox_config.set_bits &= ~((uint32_t)(kConfigDataType));
      if (!lidar_device->livox_config.set_bits)
      {
        lidar_device->connect_state = kConnectStateSampling;
      }
    }
    else if (status == kLivoxLidarStatusTimeout)
    {
      const UserLivoxLidarConfig &config = lidar_device->livox_config;
      SetLivoxLidarPclDataType(handle, static_cast<LivoxLidarPointDataType>(config.pcl_data_type),
                               LivoxLidarCallback::SetDataTypeCallback, client_data);
    }
    else
    {
      std::cout << "failed to set data type, handle: " << handle
                << ", return code: " << response->ret_code
                << ", error key: " << response->error_key << std::endl;
    }
    return;
  }

  void LivoxLidarCallback::SetPatternModeCallback(livox_status status, uint32_t handle,
                                                  LivoxLidarAsyncControlResponse *response,
                                                  void *client_data)
  {
    LidarDevice *lidar_device = GetLidarDevice(handle, client_data);
    if (lidar_device == nullptr)
    {
      std::cout << "failed to set pattern mode since no lidar device found, handle: "
                << handle << std::endl;
      return;
    }
    LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);

    if (status == kLivoxLidarStatusSuccess)
    {
      std::lock_guard<std::mutex> lock(lds_lidar->config_mutex_);
      lidar_device->livox_config.set_bits &= ~((uint32_t)(kConfigScanPattern));
      if (!lidar_device->livox_config.set_bits)
      {
        lidar_device->connect_state = kConnectStateSampling;
      }
    }
    else if (status == kLivoxLidarStatusTimeout)
    {
      const UserLivoxLidarConfig &config = lidar_device->livox_config;
      SetLivoxLidarScanPattern(handle, static_cast<LivoxLidarScanPattern>(config.pattern_mode),
                               LivoxLidarCallback::SetPatternModeCallback, client_data);
    }
    else
    {
      std::cout << "failed to set pattern mode, handle: " << handle
                << ", return code: " << response->ret_code
                << ", error key: " << response->error_key << std::endl;
    }
    return;
  }

  void LivoxLidarCallback::SetBlindSpotCallback(livox_status status, uint32_t handle,
                                                LivoxLidarAsyncControlResponse *response,
                                                void *client_data)
  {
    LidarDevice *lidar_device = GetLidarDevice(handle, client_data);
    if (lidar_device == nullptr)
    {
      std::cout << "failed to set blind spot since no lidar device found, handle: "
                << handle << std::endl;
      return;
    }
    LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);

    if (status == kLivoxLidarStatusSuccess)
    {
      std::lock_guard<std::mutex> lock(lds_lidar->config_mutex_);
      lidar_device->livox_config.set_bits &= ~((uint32_t)(kConfigBlindSpot));
      if (!lidar_device->livox_config.set_bits)
      {
        lidar_device->connect_state = kConnectStateSampling;
      }
    }
    else if (status == kLivoxLidarStatusTimeout)
    {
      const UserLivoxLidarConfig &config = lidar_device->livox_config;
      SetLivoxLidarBlindSpot(handle, config.blind_spot_set,
                             LivoxLidarCallback::SetBlindSpotCallback, client_data);
    }
    else
    {
      std::cout << "failed to set blind spot, handle: " << handle
                << ", return code: " << response->ret_code
                << ", error key: " << response->error_key << std::endl;
    }
    return;
  }

  void LivoxLidarCallback::SetDualEmitCallback(livox_status status, uint32_t handle,
                                               LivoxLidarAsyncControlResponse *response,
                                               void *client_data)
  {
    LidarDevice *lidar_device = GetLidarDevice(handle, client_data);
    if (lidar_device == nullptr)
    {
      std::cout << "failed to set dual emit mode since no lidar device found, handle: "
                << handle << std::endl;
      return;
    }

    LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);
    if (status == kLivoxLidarStatusSuccess)
    {
      std::lock_guard<std::mutex> lock(lds_lidar->config_mutex_);
      lidar_device->livox_config.set_bits &= ~((uint32_t)(kConfigDualEmit));
      if (!lidar_device->livox_config.set_bits)
      {
        lidar_device->connect_state = kConnectStateSampling;
      }
    }
    else if (status == kLivoxLidarStatusTimeout)
    {
      const UserLivoxLidarConfig &config = lidar_device->livox_config;
      SetLivoxLidarDualEmit(handle, config.dual_emit_en,
                            LivoxLidarCallback::SetDualEmitCallback, client_data);
    }
    else
    {
      std::cout << "failed to set dual emit mode, handle: " << handle
                << ", return code: " << response->ret_code
                << ", error key: " << response->error_key << std::endl;
    }
    return;
  }

  void LivoxLidarCallback::SetAttitudeCallback(livox_status status, uint32_t handle,
                                               LivoxLidarAsyncControlResponse *response,
                                               void *client_data)
  {
    LidarDevice *lidar_device = GetLidarDevice(handle, client_data);
    if (lidar_device == nullptr)
    {
      std::cout << "failed to set attitude since no lidar device found, handle: "
                << handle << std::endl;
      return;
    }

    LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);
    if (status == kLivoxLidarStatusTimeout)
    {
      const UserLivoxLidarConfig &config = lidar_device->livox_config;
      LivoxLidarInstallAttitude attitude{
          config.extrinsic_param.roll,
          config.extrinsic_param.pitch,
          config.extrinsic_param.yaw,
          config.extrinsic_param.x,
          config.extrinsic_param.y,
          config.extrinsic_param.z};
      SetLivoxLidarInstallAttitude(config.handle, &attitude,
                                   LivoxLidarCallback::SetAttitudeCallback, lds_lidar);
    }
    else if (status != kLivoxLidarStatusSuccess)
    {
      std::cout << "failed to set lidar attitude, ip: " << IpNumToString(handle) << std::endl;
    }
  }

  void LivoxLidarCallback::EnableLivoxLidarImuDataCallback(livox_status status, uint32_t handle,
                                                           LivoxLidarAsyncControlResponse *response,
                                                           void *client_data)
  {
    LidarDevice *lidar_device = GetLidarDevice(handle, client_data);
    if (lidar_device == nullptr)
    {
      std::cout << "failed to enable imu since no lidar device found, handle: "
                << handle << std::endl;
      return;
    }
    LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);

    if (response == nullptr)
    {
      std::cout << "failed to get response since no lidar IMU sensor found, handle: "
                << handle << std::endl;
      return;
    }

    if (status == kLivoxLidarStatusTimeout)
    {
      EnableLivoxLidarImuData(handle, LivoxLidarCallback::EnableLivoxLidarImuDataCallback, lds_lidar);
    }
    else if (status != kLivoxLidarStatusSuccess)
    {
      std::cout << "failed to enable Livox Lidar imu, ip: " << IpNumToString(handle) << std::endl;
    }
  }

  LidarDevice *LivoxLidarCallback::GetLidarDevice(const uint32_t handle, void *client_data)
  {
    if (client_data == nullptr)
    {
      return nullptr;
    }

    LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);
    uint8_t index = 0;
    int8_t ret = lds_lidar->cache_index_.GetIndex(kLivoxLidarType, handle, index);
    if (ret != 0)
    {
      return nullptr;
    }

    return &(lds_lidar->lidars_[index]);
  }

}
