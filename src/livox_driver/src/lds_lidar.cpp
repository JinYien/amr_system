#include "lds_lidar.h"

#include <stdio.h>
#include <string.h>
#include <memory>
#include <mutex>
#include <thread>

#ifdef WIN32
#include <winsock2.h>
#include <ws2def.h>
#pragma comment(lib, "Ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif
#include "livox_lidar_api.h"
#include "comm/comm.h"
#include "comm/pub_handler.h"

#include "parse_cfg_file/parse_cfg_file.h"
#include "parse_cfg_file/parse_livox_lidar_cfg.h"

#include "call_back/lidar_common_callback.h"
#include "call_back/livox_lidar_callback.h"

using namespace std;

namespace livox_ros
{

  LdsLidar *g_lds_ldiar = nullptr;

  LdsLidar::LdsLidar(double publish_freq)
      : Lds(publish_freq, kSourceRawLidar),
        is_initialized_(false)
  {
    ResetLdsLidar();
  }

  LdsLidar::~LdsLidar() {}

  void LdsLidar::ResetLdsLidar(void) { ResetLds(kSourceRawLidar); }

  bool LdsLidar::InitLdsLidar(const std::string &path_name)
  {
    if (is_initialized_)
    {
      printf("Lds is already inited!\n");
      return false;
    }

    if (g_lds_ldiar == nullptr)
    {
      g_lds_ldiar = this;
    }

    path_ = path_name;
    if (!InitLidars())
    {
      return false;
    }
    SetLidarPubHandle();
    if (!Start())
    {
      return false;
    }
    is_initialized_ = true;
    return true;
  }

  bool LdsLidar::InitLidars()
  {
    if (!ParseSummaryConfig())
    {
      return false;
    }

    if (lidar_summary_info_.lidar_type & kLivoxLidarType)
    {
      if (!InitLivoxLidar())
      {
        return false;
      }
    }
    return true;
  }

  bool LdsLidar::Start()
  {
    if (lidar_summary_info_.lidar_type & kLivoxLidarType)
    {
      if (!LivoxLidarStart())
      {
        return false;
      }
    }
    return true;
  }

  bool LdsLidar::ParseSummaryConfig()
  {
    return ParseCfgFile(path_).ParseSummaryInfo(lidar_summary_info_);
  }

  bool LdsLidar::InitLivoxLidar()
  {
    DisableLivoxSdkConsoleLogger();

    LivoxLidarConfigParser parser(path_);
    std::vector<UserLivoxLidarConfig> user_configs;
    if (!parser.Parse(user_configs))
    {
      std::cout << "failed to parse user-defined config" << std::endl;
    }

    if (!LivoxLidarSdkInit(path_.c_str()))
    {
      std::cout << "Failed to init livox lidar sdk." << std::endl;
      return false;
    }

    for (auto &config : user_configs)
    {
      uint8_t index = 0;
      int8_t ret = g_lds_ldiar->cache_index_.GetFreeIndex(kLivoxLidarType, config.handle, index);
      if (ret != 0)
      {
        std::cout << "failed to get free index, lidar ip: " << IpNumToString(config.handle) << std::endl;
        continue;
      }
      LidarDevice *p_lidar = &(g_lds_ldiar->lidars_[index]);
      p_lidar->lidar_type = kLivoxLidarType;
      p_lidar->livox_config = config;
      p_lidar->handle = config.handle;

      LidarExtParameter lidar_param;
      lidar_param.handle = config.handle;
      lidar_param.lidar_type = kLivoxLidarType;
      if (config.pcl_data_type == kLivoxLidarCartesianCoordinateLowData)
      {

        lidar_param.param.roll = config.extrinsic_param.roll;
        lidar_param.param.pitch = config.extrinsic_param.pitch;
        lidar_param.param.yaw = config.extrinsic_param.yaw;
        lidar_param.param.x = config.extrinsic_param.x / 10;
        lidar_param.param.y = config.extrinsic_param.y / 10;
        lidar_param.param.z = config.extrinsic_param.z / 10;
      }
      else
      {
        lidar_param.param.roll = config.extrinsic_param.roll;
        lidar_param.param.pitch = config.extrinsic_param.pitch;
        lidar_param.param.yaw = config.extrinsic_param.yaw;
        lidar_param.param.x = config.extrinsic_param.x;
        lidar_param.param.y = config.extrinsic_param.y;
        lidar_param.param.z = config.extrinsic_param.z;
      }
      pub_handler().AddLidarsExtParam(lidar_param);
    }

    SetLivoxLidarInfoChangeCallback(LivoxLidarCallback::LidarInfoChangeCallback, g_lds_ldiar);
    return true;
  }

  void LdsLidar::SetLidarPubHandle()
  {
    pub_handler().SetPointCloudsCallback(LidarCommonCallback::OnLidarPointClounCb, g_lds_ldiar);
    pub_handler().SetImuDataCallback(LidarCommonCallback::LidarImuDataCallback, g_lds_ldiar);

    double publish_freq = Lds::GetLdsFrequency();
    pub_handler().SetPointCloudConfig(publish_freq);
  }

  bool LdsLidar::LivoxLidarStart()
  {
    return true;
  }

  int LdsLidar::DeInitLdsLidar(void)
  {
    if (!is_initialized_)
    {
      printf("LiDAR data source is not exit");
      return -1;
    }

    if (lidar_summary_info_.lidar_type & kLivoxLidarType)
    {
      LivoxLidarSdkUninit();
    }

    return 0;
  }

  void LdsLidar::PrepareExit(void) { DeInitLdsLidar(); }

}
