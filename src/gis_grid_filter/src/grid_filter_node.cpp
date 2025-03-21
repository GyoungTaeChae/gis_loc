#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <unordered_map>
#include <tuple>

// Grid thresholds
const float GRID_RESOLUTION = 2.0; // Grid cell size (0.5m x 0.5m)
const int MIN_POINTS_IN_CELL = 5; // Minimum number of points required in a cell

ros::Publisher filtered_pub;

// Hash function for grid cells
struct GridHash {
    size_t operator()(const std::tuple<int, int>& key) const {
        return std::hash<int>()(std::get<0>(key)) ^ std::hash<int>()(std::get<1>(key));
    }
};

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // Convert ROS PointCloud2 message to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    // Map to store point counts per grid cell
    std::unordered_map<std::tuple<int, int>, int, GridHash> grid_count;

    // First pass: Count points in each grid cell
    for (const auto& point : cloud->points) {
        int grid_x = static_cast<int>(std::floor(point.x / GRID_RESOLUTION));
        int grid_y = static_cast<int>(std::floor(point.y / GRID_RESOLUTION));
        grid_count[std::make_tuple(grid_x, grid_y)]++;
    }

    // Second pass: Keep points in cells with sufficient points
    for (const auto& point : cloud->points) {
        int grid_x = static_cast<int>(std::floor(point.x / GRID_RESOLUTION));
        int grid_y = static_cast<int>(std::floor(point.y / GRID_RESOLUTION));
        if (grid_count[std::make_tuple(grid_x, grid_y)] >= MIN_POINTS_IN_CELL) {
            filtered_cloud->points.push_back(point);
        }
    }

    // Set the filtered cloud's metadata
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;

    // Convert the filtered PCL PointCloud back to ROS PointCloud2 message
    sensor_msgs::PointCloud2 filtered_msg;
    pcl::toROSMsg(*filtered_cloud, filtered_msg);
    filtered_msg.header = msg->header;

    // Publish the filtered point cloud
    filtered_pub.publish(filtered_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "grid_filter_node");
    ros::NodeHandle nh;

    // Subscriber to the /building/points topic
    ros::Subscriber sub = nh.subscribe("/normal/points", 1, pointCloudCallback);

    // Publisher for the filtered point cloud
    filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("/building/points", 1);

    ros::spin();
    return 0;
}
