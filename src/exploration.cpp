/**
 * @file exploration.cpp
 * @author 
 * @brief Implementation of frontier-based exploration
 * @version 0.1
 * @date 2023-03-11
 * 
 * @todo
 *      - serve goal requests from navigation node
 *      - implement frontier based exploration algorithm
 * 
 * @copyright Copyright (c) 2023
 *
 */
#include "frontier_exploration/exploration.hpp"

FrontierExplorer::FrontierExplorer()
: Node("frontier_explorer")
{
    map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", 1, std::bind(&FrontierExplorer::map_callback, this, _1));

    service_ = this->create_service<frontier_interfaces::srv::FrontierGoal>(
        "frontier_pose", std::bind(&FrontierExplorer::get_frontiers, this, _1, _2));
}

void FrontierExplorer::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr recent_map)
{
    auto width = recent_map->info.width;
    auto height = recent_map->info.height;

    RCLCPP_INFO(this->get_logger(),"Map recieved w: %d h: %d.\n.", width, height);
}

void FrontierExplorer::get_frontiers(const std::shared_ptr<frontier_interfaces::srv::FrontierGoal::Request> request,
          std::shared_ptr<frontier_interfaces::srv::FrontierGoal::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received request, %d", request->goal_rank);
    

    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.stamp = this->get_clock()->now();
    response->goal_pose;

   
    // RCLCPP_INFO(this->get_logger(), "sending back response");
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    // Start processing data from the node as well as the callbacks and the timer
    rclcpp::spin(std::make_shared<FrontierExplorer>());
    
    // Shutdown the node when finished
    rclcpp::shutdown();
    return 0;
}