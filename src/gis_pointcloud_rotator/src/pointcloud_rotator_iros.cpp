#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <cmath>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

double yaw_degrees = 0;  // Global yaw value
double yaw_bias = 270.58; // Bias correction
ros::Publisher rotated_cloud_pub;

void calculateYawFromMagneticField(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
    static ros::Time last_imu_time = ros::Time::now();
    double imu_update_rate = 0.1; // 10 Hz

    ros::Time current_time = ros::Time::now();
    if ((current_time - last_imu_time).toSec() < imu_update_rate) return;
    last_imu_time = current_time;

    // Extract magnetic field data
    double mag_x = msg->vector.x;
    double mag_y = msg->vector.y;

    // Calculate yaw in degrees
    yaw_degrees = std::atan2(mag_y, mag_x) * 180.0 / M_PI;

    // Normalize yaw to [0, 360]
    if (yaw_degrees < 0) yaw_degrees += 360;

    // Apply bias correction
    yaw_degrees -= yaw_bias;
    if (yaw_degrees < 0) yaw_degrees += 360;
    if (yaw_degrees >= 360) yaw_degrees -= 360;

    // ROS_INFO("Updated Yaw (degrees from true north after bias correction): %.2f", yaw_degrees);
}

void rotatePointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // Convert yaw to radians and reverse for rotation
    double yaw_radians = -yaw_degrees * M_PI / 180.0;

    // Create rotation matrix for Z-axis
    tf2::Matrix3x3 rotation_matrix;
    rotation_matrix.setIdentity();
    rotation_matrix[0][0] = std::cos(yaw_radians);
    rotation_matrix[0][1] = -std::sin(yaw_radians);
    rotation_matrix[1][0] = std::sin(yaw_radians);
    rotation_matrix[1][1] = std::cos(yaw_radians);

    // Create a new PointCloud2 for rotated points
    sensor_msgs::PointCloud2 rotated_cloud;
    rotated_cloud.header = msg->header;
    rotated_cloud.height = msg->height;
    rotated_cloud.width = msg->width;
    rotated_cloud.fields = msg->fields;
    rotated_cloud.is_bigendian = msg->is_bigendian;
    rotated_cloud.point_step = msg->point_step;
    rotated_cloud.row_step = msg->row_step;
    rotated_cloud.is_dense = msg->is_dense;
    rotated_cloud.data.resize(msg->data.size());

    // Iterate through points
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    sensor_msgs::PointCloud2Iterator<float> out_iter_x(rotated_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> out_iter_y(rotated_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> out_iter_z(rotated_cloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++out_iter_x, ++out_iter_y, ++out_iter_z) {
        tf2::Vector3 point(*iter_x, *iter_y, *iter_z);
        tf2::Vector3 rotated_point = rotation_matrix * point;

        *out_iter_x = rotated_point.x();
        *out_iter_y = rotated_point.y();
        *out_iter_z = rotated_point.z();
    }

    // Publish the rotated cloud
    rotated_cloud_pub.publish(rotated_cloud);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_rotator");
    ros::NodeHandle nh;

    // Subscriber for IMU magnetic field data
    ros::Subscriber imu_sub = nh.subscribe("/imu/mag", 10, calculateYawFromMagneticField);

    // Subscriber for PointCloud2 data
    ros::Subscriber pointcloud_sub = nh.subscribe("/building/points", 10, rotatePointCloud);
    // ros::Subscriber pointcloud_sub = nh.subscribe("/normal/points", 10, rotatePointCloud);
    // ros::Subscriber pointcloud_sub = nh.subscribe("/rangenet/points", 10, rotatePointCloud);

    // Publisher for rotated PointCloud2
    rotated_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/rotated_points", 10);
    
    ROS_INFO("PointCloud rotator node started. Waiting for messages...");
    ros::spin();

    return 0;
}
