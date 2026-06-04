#ifndef AMR_MESSAGE__QOS_HPP_
#define AMR_MESSAGE__QOS_HPP_

#include <string>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/qos.hpp"
#include "yaml-cpp/yaml.h"

namespace amr_message
{

    inline rclcpp::QoS QosProfile()
    {
        const std::string config = ament_index_cpp::get_package_share_directory("amr_message") + "/config/qos.yaml";
        const YAML::Node qos = YAML::LoadFile(config)["qos"];

        const auto depth = qos["depth"].as<size_t>();
        rclcpp::QoS profile{depth};

        const auto history = qos["history"].as<std::string>();
        if (history == "keep_last")
        {
            profile.keep_last(depth);
        }
        else if (history == "keep_all")
        {
            profile.keep_all();
        }

        const auto reliability = qos["reliability"].as<std::string>();
        if (reliability == "best_effort")
        {
            profile.best_effort();
        }
        else if (reliability == "reliable")
        {
            profile.reliable();
        }

        const auto durability = qos["durability"].as<std::string>();
        if (durability == "volatile")
        {
            profile.durability_volatile();
        }
        else if (durability == "transient_local")
        {
            profile.transient_local();
        }

        return profile;
    }

    inline const rclcpp::QoS QOS_PROFILE = QosProfile();

} // namespace amr_message

#endif // AMR_MESSAGE__QOS_HPP_