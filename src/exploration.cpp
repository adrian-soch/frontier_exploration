/**
 * @file exploration.cpp
 * @author 
 * @brief Implementation of frontier-based exploration
 * @version 0.1
 * @date 2023-03-11
 * 
 * @todo
 *      - Publish on goal topic for robot to navigate
 *      - sub to occupancy grid map
 *      - implement frontier based exploration algorithm
 * 
 * @copyright Copyright (c) 2023
 *
 */
#include "frontier_exploration/exploration.hpp"

FrontierExplorer::FrontierExplorer()
: Node("frontier_explorer")
{
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    
    // The timer_callback function will execute every 500ms
    timer_ = this->create_wall_timer(
    500ms, std::bind(&FrontierExplorer::timer_callback, this));
}

void FrontierExplorer::timer_callback()
{
    auto message = std_msgs::msg::String();
    message.data = "test";

    // Print every message to the terminal window      
    RCLCPP_INFO(this->get_logger(),"Publishing: '%s'", message.data.c_str());
    
    publisher_->publish(message);
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