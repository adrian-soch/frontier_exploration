#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>

#include <iostream>

using std::placeholders::_1;

class OccupancyGridSubscriber : public rclcpp::Node
{
public:
    OccupancyGridSubscriber()
        : Node("occupancy_grid_subscriber_node")
    {
        subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 10, std::bind(&OccupancyGridSubscriber::callback, this, _1));
    }

    void callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {

                // Convert the occupancy grid message to an OpenCV Mat
                cv::Mat occupancy_grid(msg->info.height, msg->info.width, CV_8SC1, (void*) msg->data.data());


                cv::Mat grayscale_image(occupancy_grid.size(), CV_8UC1);
                grayscale_image.setTo(255);  // Set all pixels to white
                grayscale_image.setTo(128, occupancy_grid == -1);  // Set pixels with value -1 to grey
                grayscale_image.setTo(0, occupancy_grid >= 1);  // 

                // Save the grayscale image to a file
                cv::imwrite("/workspace/src/occupancy_grid.png", grayscale_image);
                RCLCPP_INFO(this->get_logger(), "Saving Image");
            }
private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OccupancyGridSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
