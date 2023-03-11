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

#include <chrono>
#include <string>
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
 
using namespace std::chrono_literals;

class FrontierExplorer : public rclcpp::Node
{
  public:
    FrontierExplorer();
 
  private:

    void timer_callback();
     
    // Declaration of the timer_ attribute
    rclcpp::TimerBase::SharedPtr timer_;
  
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

#endif