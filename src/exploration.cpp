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

    std::lock_guard<std::mutex> guard(mutex_);
    map_ = *recent_map;
}

void FrontierExplorer::get_frontiers(const std::shared_ptr<frontier_interfaces::srv::FrontierGoal::Request> request,
          std::shared_ptr<frontier_interfaces::srv::FrontierGoal::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received request, %d", request->goal_rank);

    // Copy the map
    std::unique_lock<std::mutex> lck(mutex_);
        nav_msgs::msg::OccupancyGrid map = map_;
    lck.unlock();

    frontierCellGrid.clear(); 																						// Optional
	frontierCellGrid = computeFrontierCellGrid(map.data, map.info.width, map.info.height);
	
	// Print the Frontier Cell Grid
	//printGrid(frontierCellGrid, map.info.width, map.info.height);


	// 2. Compute the Frontier Regions
	frontierRegions.clear(); 																						// Optional
	frontierRegions = computeFrontierRegions(frontierCellGrid, map.info.width, map.info.height);
	
    /**
     * @todo Create a publisher for the frontier map
     * @todo Create a publisher for frontier regions as a marker in /map frame
     */
	// Print the Frontier Regions
	// printFrontierRegions(frontierRegions);

    // Create and init message
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.stamp = this->get_clock()->now();
    goal_pose.pose.position.x = 0.0;
    goal_pose.pose.position.y = 0.0;
    
    // Set the response
    response->goal_pose;

    RCLCPP_INFO(this->get_logger(), "Sending goal x: %f y: %f.",
        goal_pose.pose.position.x, goal_pose.pose.position.y);
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