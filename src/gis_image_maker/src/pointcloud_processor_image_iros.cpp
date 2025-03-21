#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>

#define IMAGE_SIZE 384
#define RECT_SIZE 15  // 사각형 한 변의 길이

ros::Publisher image_pub;
double max_distance = 45.0; // 기본값

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Create an empty white image
    cv::Mat image(IMAGE_SIZE, IMAGE_SIZE, CV_8UC1, cv::Scalar(255));

    // Compute scaling factor
    double scale = (IMAGE_SIZE / 2.0) / max_distance;

    for (const auto& point : cloud->points) {
        // Filter points within max_distance
        if (std::sqrt(point.x * point.x + point.y * point.y) <= max_distance) {
            // Transform x, y coordinates to image indices
            int i = static_cast<int>((point.x * scale) + IMAGE_SIZE / 2);
            int j = static_cast<int>((point.y * scale) + IMAGE_SIZE / 2);

            // Ensure indices are within image bounds
            if (i >= 0 && i < IMAGE_SIZE && j >= 0 && j < IMAGE_SIZE) {
                cv::Point top_left(i - RECT_SIZE / 2, j - RECT_SIZE / 2);
                cv::Point bottom_right(i + RECT_SIZE / 2, j + RECT_SIZE / 2);

                // Draw a filled rectangle
                cv::rectangle(image, top_left, bottom_right, cv::Scalar(0), -1);  // -1 to fill the rectangle
            }
        }
    }

    // Flip the image horizontally
    cv::Mat flipped_image;
    cv::flip(image, flipped_image, 1);

    // Convert OpenCV image to ROS Image message
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = cloud_msg->header.frame_id;
    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(header, "mono8", flipped_image).toImageMsg();

    // Publish the image
    image_pub.publish(image_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_image_converter");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Get the max_distance parameter
    private_nh.param("max_distance", max_distance, 45.0); // 기본값 45.0

    ROS_INFO("Using max_distance: %f", max_distance);

    // Subscribe to the PointCloud2 topic
    ros::Subscriber pcl_sub = nh.subscribe("/rotated_points", 1, pointCloudCallback);

    // Advertise the image topic
    image_pub = nh.advertise<sensor_msgs::Image>("/processed/image", 1);

    ros::spin();

    return 0;
}
