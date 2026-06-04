#include "amr_intersection/node.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include "amr_message/qos.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "yaml-cpp/yaml.h"

namespace amr_intersection
{

    namespace
    {
        constexpr double kDegToRad = M_PI / 180.0;

        double normalize_angle(double angle)
        {
            while (angle > M_PI)
            {
                angle -= 2.0 * M_PI;
            }
            while (angle <= -M_PI)
            {
                angle += 2.0 * M_PI;
            }
            return angle;
        }

        bool in_sector(double angle, const SectorConfig &sector)
        {
            return angle >= sector.min_rad && angle <= sector.max_rad;
        }

        class GapFinder
        {
        public:
            GapFinder(double required_clearance, double angle_increment, int max_skip)
                : required_clearance_(required_clearance),
                  angle_increment_(angle_increment),
                  max_skip_(max_skip)
            {
            }

            void add_open(double distance)
            {
                if (!in_run_)
                {
                    in_run_ = true;
                    run_beams_ = 0;
                    run_min_distance_ = std::numeric_limits<double>::infinity();
                }
                ++run_beams_;
                skipped_ = 0;
                run_min_distance_ = std::min(run_min_distance_, distance);
            }

            void add_unreliable()
            {
                if (in_run_ && skipped_ < max_skip_)
                {
                    ++skipped_;
                }
                else
                {
                    end_run();
                }
            }

            void add_blocked() { end_run(); }

            GapResult finish()
            {
                end_run();
                best_.open = best_.width >= required_clearance_;
                return best_;
            }

        private:
            void end_run()
            {
                if (run_beams_ > 0)
                {
                    const double span = (run_beams_ - 1) * angle_increment_;
                    const double width = 2.0 * run_min_distance_ * std::sin(0.5 * span);
                    if (width > best_.width)
                    {
                        best_.width = width;
                        best_.distance = run_min_distance_;
                    }
                }
                in_run_ = false;
                run_beams_ = 0;
                skipped_ = 0;
                run_min_distance_ = std::numeric_limits<double>::infinity();
            }

            double required_clearance_;
            double angle_increment_;
            int max_skip_;

            bool in_run_ = false;
            int run_beams_ = 0;
            int skipped_ = 0;
            double run_min_distance_ = std::numeric_limits<double>::infinity();
            GapResult best_;
        };
    }

    IntersectionNode::IntersectionNode() : rclcpp::Node("intersection_node")
    {
        const std::string config = declare_parameter<std::string>("config", "");
        if (config.empty())
        {
            throw std::runtime_error(
                "No 'config' parameter provided; pass config:=/path/to/intersection.yaml");
        }
        const std::string robot = declare_parameter<std::string>("robot", "");
        if (robot.empty())
        {
            throw std::runtime_error(
                "No 'robot' parameter provided; pass robot:=/path/to/amr_robot/robot.yaml");
        }
        load_config(config, robot);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            lidar_topic_, amr_message::QOS_PROFILE,
            std::bind(&IntersectionNode::scan_callback, this, std::placeholders::_1));

        intersection_pub_ = create_publisher<amr_message::msg::Intersection>(
            intersection_topic_, amr_message::QOS_PROFILE);
    }

    void IntersectionNode::load_config(const std::string &path, const std::string &robot_path)
    {
        const YAML::Node root = YAML::LoadFile(path)["amr_intersection"];

        const YAML::Node topics = root["topics"];
        lidar_topic_ = topics["lidar_subscriber"].as<std::string>();
        intersection_topic_ = topics["intersection_publisher"].as<std::string>();

        const YAML::Node footprint = YAML::LoadFile(robot_path)["amr_robot"]["footprint"];
        footprint_frame_ = footprint["frame"].as<std::string>();
        robot_length_ = footprint["length"].as<double>();
        robot_width_ = footprint["width"].as<double>();

        const YAML::Node geometry = root["geometry"];
        safety_margin_ = geometry["safety_margin"].as<double>();
        min_open_depth_ = geometry["min_open_depth"].as<double>();

        const YAML::Node sectors = root["sectors"];
        front_.min_rad = sectors["front_min"].as<double>() * kDegToRad;
        front_.max_rad = sectors["front_max"].as<double>() * kDegToRad;
        left_.min_rad = sectors["left_min"].as<double>() * kDegToRad;
        left_.max_rad = sectors["left_max"].as<double>() * kDegToRad;
        right_.min_rad = sectors["right_min"].as<double>() * kDegToRad;
        right_.max_rad = sectors["right_max"].as<double>() * kDegToRad;

        max_skip_beams_ = root["scan"]["max_skip_beams"].as<int>();
        debounce_time_ = root["filter"]["debounce_time"].as<double>();
    }

    bool IntersectionNode::debounce(bool raw, SectorState &state, double now_s) const
    {
        if (raw != state.last_raw)
        {
            state.last_raw = raw;
            state.last_raw_time = now_s;
        }
        if (raw != state.stable && (now_s - state.last_raw_time) >= debounce_time_)
        {
            state.stable = raw;
        }
        return state.stable;
    }

    double IntersectionNode::footprint_free_distance(double lidar_x, double lidar_y,
                                                     double dir_x, double dir_y, double range) const
    {
        const double half_length = 0.5 * robot_length_ + safety_margin_;
        const double half_width = 0.5 * robot_width_ + safety_margin_;

        constexpr double kEps = 1e-9;
        double t_enter = -std::numeric_limits<double>::infinity();
        double t_exit = std::numeric_limits<double>::infinity();

        const double origin[2] = {lidar_x, lidar_y};
        const double dir[2] = {dir_x, dir_y};
        const double half[2] = {half_length, half_width};
        for (int axis = 0; axis < 2; ++axis)
        {
            if (std::abs(dir[axis]) > kEps)
            {
                double t1 = (-half[axis] - origin[axis]) / dir[axis];
                double t2 = (half[axis] - origin[axis]) / dir[axis];
                if (t1 > t2)
                {
                    std::swap(t1, t2);
                }
                t_enter = std::max(t_enter, t1);
                t_exit = std::min(t_exit, t2);
            }
            else if (origin[axis] < -half[axis] || origin[axis] > half[axis])
            {
                t_exit = -std::numeric_limits<double>::infinity();
            }
        }

        const double exit = (t_enter > t_exit || t_exit < 0.0) ? 0.0 : t_exit;
        const double free_distance = range - exit;
        return free_distance > 0.0 ? free_distance : 0.0;
    }

    void IntersectionNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        geometry_msgs::msg::TransformStamped tf;
        try
        {
            tf = tf_buffer_->lookupTransform(footprint_frame_, scan->header.frame_id, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "TF %s <- %s unavailable, skipping scan: %s",
                                 footprint_frame_.c_str(), scan->header.frame_id.c_str(), ex.what());
            return;
        }

        const double lidar_x = tf.transform.translation.x;
        const double lidar_y = tf.transform.translation.y;
        const double q_x = tf.transform.rotation.x;
        const double q_y = tf.transform.rotation.y;
        const double q_z = tf.transform.rotation.z;
        const double q_w = tf.transform.rotation.w;
        const double lidar_yaw = std::atan2(2.0 * (q_w * q_z + q_x * q_y),
                                            1.0 - 2.0 * (q_y * q_y + q_z * q_z));

        const double required_clearance = robot_width_ + 2.0 * safety_margin_;
        const double angle_increment = std::abs(scan->angle_increment);

        GapFinder front(required_clearance, angle_increment, max_skip_beams_);
        GapFinder left(required_clearance, angle_increment, max_skip_beams_);
        GapFinder right(required_clearance, angle_increment, max_skip_beams_);

        const size_t count = scan->ranges.size();
        for (size_t i = 0; i < count; ++i)
        {
            const double angle = normalize_angle(
                scan->angle_min + static_cast<double>(i) * scan->angle_increment + lidar_yaw);

            GapFinder *finder = nullptr;
            if (in_sector(angle, front_))
            {
                finder = &front;
            }
            else if (in_sector(angle, left_))
            {
                finder = &left;
            }
            else if (in_sector(angle, right_))
            {
                finder = &right;
            }
            else
            {
                continue;
            }

            const double dir_x = std::cos(angle);
            const double dir_y = std::sin(angle);

            const float range_value = scan->ranges[i];
            if (std::isnan(range_value) || range_value < scan->range_min)
            {
                finder->add_unreliable();
            }
            else if (std::isinf(range_value) || range_value > scan->range_max)
            {
                finder->add_open(footprint_free_distance(lidar_x, lidar_y, dir_x, dir_y, scan->range_max));
            }
            else
            {
                const double open_depth = footprint_free_distance(lidar_x, lidar_y, dir_x, dir_y, range_value);
                if (open_depth >= min_open_depth_)
                {
                    finder->add_open(open_depth);
                }
                else
                {
                    finder->add_blocked();
                }
            }
        }

        const double now_s = now().seconds();
        const GapResult front_gap = front.finish();
        const GapResult left_gap = left.finish();
        const GapResult right_gap = right.finish();

        amr_message::msg::Intersection msg;
        msg.front_path = debounce(front_gap.open, front_state_, now_s);
        msg.left_path = debounce(left_gap.open, left_state_, now_s);
        msg.right_path = debounce(right_gap.open, right_state_, now_s);
        msg.front_gap_distance = static_cast<float>(front_gap.distance);
        msg.left_gap_distance = static_cast<float>(left_gap.distance);
        msg.right_gap_distance = static_cast<float>(right_gap.distance);
        msg.front_gap_width = static_cast<float>(front_gap.width);
        msg.left_gap_width = static_cast<float>(left_gap.width);
        msg.right_gap_width = static_cast<float>(right_gap.width);
        intersection_pub_->publish(msg);
    }

}