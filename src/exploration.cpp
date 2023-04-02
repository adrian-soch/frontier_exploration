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
    // Get parameters
    region_size_thresh_ = this->declare_parameter("region_size_thresh", 12);
    robot_width_ = this->declare_parameter("robot_width", 0.5);
    occupancy_map_topic_ = this->declare_parameter("occupancy_map_msg", "map");

    // Subscribers/Publichers/Service setup
    map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        occupancy_map_topic_, 1, std::bind(&FrontierExplorer::map_callback, this, _1));

    service_ = this->create_service<frontier_interfaces::srv::FrontierGoal>(
        "frontier_pose", std::bind(&FrontierExplorer::get_frontiers, this, _1, _2));

    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("f_markers", 1);
    frontier_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("f_map", 1);
}

void FrontierExplorer::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr recent_map)
{
    auto width = recent_map->info.width;
    auto height = recent_map->info.height;
    RCLCPP_INFO(this->get_logger(),"Map recieved w: %d h: %d.", width, height);

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

    frontierCellGrid_.clear();
	frontierCellGrid_ = computeFrontierCellGrid(map.data, map.info.width, map.info.height);
	
	// 2. Compute the Frontier Regions
	frontierRegions_.clear();
	frontierRegions_ = computeFrontierRegions(frontierCellGrid_, map.info.width, map.info.height, 
        map.info.resolution, map.info.origin.position.x, map.info.origin.position.y, region_size_thresh_);

    nav_msgs::msg::OccupancyGrid f_map = map;

    std::transform(frontierCellGrid_.begin(), frontierCellGrid_.end(), frontierCellGrid_.begin(),
               std::bind(std::multiplies<cell>(), std::placeholders::_1, 255));
    f_map.data = frontierCellGrid_;
    frontier_map_publisher_->publish(f_map);

    publishFrontiers();

    /**
     * @todo Get robot pose and pass to `selectFrontier`
     * 
     */
    frontierRegion goal = selectFrontier(frontierRegions_, 0,
    0.0 ,0.0);

    // Create and init message
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.stamp = this->get_clock()->now();
    goal_pose.pose.position.x = goal.x;
    goal_pose.pose.position.y = goal.y;
    
    // Set the response
    response->goal_pose;
    RCLCPP_INFO(this->get_logger(), "Sending goal x: %f y: %f.",
        goal_pose.pose.position.x, goal_pose.pose.position.y);
}

void FrontierExplorer::publishFrontiers()
{
    visualization_msgs::msg::Marker::SharedPtr sphere_list(new visualization_msgs::msg::Marker);
    sphere_list->header.frame_id = map_frame_;
    sphere_list->header.stamp = this->get_clock()->now();
    sphere_list->type = visualization_msgs::msg::Marker::SPHERE_LIST;
    sphere_list->action = visualization_msgs::msg::Marker::ADD;
    sphere_list->scale.x = 0.1; // in meters
    sphere_list->scale.y = 0.1;
    sphere_list->scale.z = 0.1;
    // Set green and alpha(opacity)
    sphere_list->color.g = 1.0;
    sphere_list->color.a = 1.0;

    for(auto reg : frontierRegions_) {
        geometry_msgs::msg::Point p;
        p.x = reg.x;
        p.y = reg.y;
        p.z = 0.05;
        sphere_list->points.push_back(p);
    }
    marker_publisher_->publish(*sphere_list);
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