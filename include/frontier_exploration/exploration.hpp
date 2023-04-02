/**
 * @file exploration.hpp
 * @author
 * @brief 
 * @version 0.1
 * @date 2023-03-11
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef FRONTIER_EXPLORER
#define FRONTIER_EXPLORER

#include <memory>
#include <mutex>

#include "frontier_exploration/frontier_utils.hpp"
#include "frontier_interfaces/srv/frontier_goal.hpp"
 
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

class FrontierExplorer : public rclcpp::Node
{
    public:
        FrontierExplorer();

    private:
        int region_size_thresh_ {1};
        float robot_width_ {0.5};
        std::string occupancy_map_topic_ {"map"}; //"global_costmap/costmap"
        std::string map_frame_ {"map"};

        std::mutex mutex_;

        std::vector<cell> frontierCellGrid_;
        std::vector<frontierRegion> frontierRegions_;

        nav_msgs::msg::OccupancyGrid map_;

        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
        rclcpp::Service<frontier_interfaces::srv::FrontierGoal>::SharedPtr service_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr frontier_map_publisher_;

        void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

        void get_frontiers(const std::shared_ptr<frontier_interfaces::srv::FrontierGoal::Request> request,
            std::shared_ptr<frontier_interfaces::srv::FrontierGoal::Response> response);

        void publishFrontiers();
};

#endif