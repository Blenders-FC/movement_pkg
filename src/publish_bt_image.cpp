#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sys/stat.h>  // For file checking

bool fileExists(const std::string& filename)
{
    struct stat buffer;
    return (stat(filename.c_str(), &buffer) == 0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bt_tree_image_publisher");
    ros::NodeHandle nh;

    ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>("/bt_tree/image_raw", 1, true);  // latched

    std::string image_path = "/home/robotis/catkin_ws/src/movement_pkg/config/bt_tree.png";

    // Wait for the image file to be created
    ROS_INFO_STREAM("[publisher] Waiting for PNG file: " << image_path);
    while (ros::ok() && !fileExists(image_path))
    {
        ros::Duration(0.1).sleep();  // sleep for 100ms
    }

    ROS_INFO_STREAM("[publisher] Found PNG file. Loading...");
    ros::Duration(0.5).sleep();  // 500ms

    cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);

    if (image.empty())
    {
        ROS_ERROR_STREAM("[publisher] Failed to load BT tree image from " << image_path);
        return 1;
    }

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    ros::Rate loop_rate(0.5);  // publish twice per second

    while (ros::ok())
    {
        img_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
