#ifndef AMR_INTERSECTION__NODE_HPP_
#define AMR_INTERSECTION__NODE_HPP_

#include <memory>
#include <string>
#include "amr_message/msg/intersection.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace amr_intersection
{

    struct SectorConfig
    {
        double min_rad = 0.0;
        double max_rad = 0.0;
    };

    struct GapResult
    {
        bool open = false;
        double width = 0.0;
        double distance = 0.0;
    };

    struct SectorState
    {
        bool stable = false;
        bool last_raw = false;
        double last_raw_time = 0.0;
    };

    class IntersectionNode : public rclcpp::Node
    {
    public:
        IntersectionNode();

    private:
        void load_config(const std::string &path, const std::string &robot_path);
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

        bool debounce(bool raw, SectorState &state, double now_s) const;

        double footprint_free_distance(double lidar_x, double lidar_y,
                                       double dir_x, double dir_y, double range) const;

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        rclcpp::Publisher<amr_message::msg::Intersection>::SharedPtr intersection_pub_;

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        std::string lidar_topic_;
        std::string intersection_topic_;

        SectorConfig front_;
        SectorConfig left_;
        SectorConfig right_;

        std::string footprint_frame_ = "base_link";
        double robot_length_ = 0.37;
        double robot_width_ = 0.47;
        double safety_margin_ = 0.15;
        double min_open_depth_ = 1.0;
        int max_skip_beams_ = 1;
        double debounce_time_ = 0.0;

        SectorState front_state_;
        SectorState left_state_;
        SectorState right_state_;
    };

}

#endif