/**
 * @file training_data_collector.cpp
 * @brief Saves occupancy grid maps as image files
 *      `ros2 run frontier_exploration collection_node --ros-args -p filename:="/workspace/src/text.png"`
 * @version 0.1
 * @date 2023-04-04
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "frontier_exploration/frontier_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <iostream>
#include <sstream>
#include <fstream>

#include <filesystem>

using std::placeholders::_1;
namespace fs = std::filesystem;

class OccupancyGridSubscriber : public rclcpp::Node
{
public:
    OccupancyGridSubscriber()
        : Node("occupancy_grid_subscriber_node")
    {
        subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 10, std::bind(&OccupancyGridSubscriber::callback, this, _1));

        this->declare_parameter<std::string>("path", "/workspace/");
        this->get_parameter("path", path_);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        last_pose_ = Eigen::Affine3d::Identity();

        auto const now = std::chrono::system_clock::now();
        auto const in_time_t = std::chrono::system_clock::to_time_t(now);

        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%d_%m_%Y_%H_%M_%S");

        path_ = path_ + ss.str();
        fs::create_directory(path_);
    }


    void callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {

        bool moved_enough = getDistTravelled(0.1);
        if(!moved_enough) {
            return;
        }

        std::vector<cell> processed = preprocessMap(msg->data, msg->info.width, msg->info.height);

        // Convert the occupancy grid message to an OpenCV Mat
        cv::Mat occupancy_grid(msg->info.height, msg->info.width, CV_8SC1, (void*) processed.data());

        cv::Mat grayscale_image(occupancy_grid.size(), CV_8UC1);
        grayscale_image.setTo(255);  // Set all pixels to white
        grayscale_image.setTo(128, occupancy_grid == -1);  // Set pixels with value -1 to grey
        grayscale_image.setTo(0, occupancy_grid >= 1);  // 

        // Create file name
        std::string fullpath = path_ + "/" + std::to_string(saved_image_count) + "_" +
            std::to_string(msg->info.height) + "_" +
            std::to_string(msg->info.width) + "_" + 
            std::to_string(msg->info.resolution) + ".png";
        ++saved_image_count;

        // Save the grayscale image to a file
        cv::imwrite(fullpath, grayscale_image);
        RCLCPP_INFO(this->get_logger(), "Saving Image %s", fullpath.c_str());
    }

    bool getDistTravelled(double thresh) {
        geometry_msgs::msg::TransformStamped stransform;
        try {
            stransform = tf_buffer_->lookupTransform(odom_frame_, base_frame_,
                tf2::TimePointZero, tf2::durationFromSec(3));
        }
        catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        }
        
        Eigen::Affine3d T; //
        T = tf2::transformToEigen(stransform);

        // Compute relative transform
        Eigen::Affine3d movement; //
        movement = last_pose_.inverse() * T;
        
        // Threshold for enough motion
        double transl = movement.translation().norm();
        RCLCPP_INFO(this->get_logger(), "dist: %f", transl);

        if(transl < thresh){
            return false;
        }

        // Set new last pose
        last_pose_ = T;

        return true;
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;

    std::string path_;
    Eigen::Affine3d last_pose_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    std::string odom_frame_ {"odom"};
    std::string base_frame_ {"base_link"};

    size_t saved_image_count {0};

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OccupancyGridSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
