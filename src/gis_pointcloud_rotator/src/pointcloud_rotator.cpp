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

double yaw_degrees = 0;  // 전역 yaw 값
double yaw_bias = 270.58; // 보정 값
ros::Publisher rotated_cloud_pub;
ros::Publisher rotated_road_cloud_pub;

void calculateYawFromMagneticField(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    static ros::Time last_imu_time = ros::Time::now();
    double imu_update_rate = 0.1; // 10 Hz

    ros::Time current_time = ros::Time::now();
    if ((current_time - last_imu_time).toSec() < imu_update_rate)
        return;
    last_imu_time = current_time;

    // 자기장 데이터 추출
    double mag_x = msg->vector.x;
    double mag_y = msg->vector.y;

    // yaw(회전각)를 도 단위로 계산
    yaw_degrees = std::atan2(mag_y, mag_x) * 180.0 / M_PI;

    // yaw를 [0, 360] 범위로 변환
    if (yaw_degrees < 0)
        yaw_degrees += 360;

    // 보정 값 적용
    yaw_degrees -= yaw_bias;
    if (yaw_degrees < 0)
        yaw_degrees += 360;
    if (yaw_degrees >= 360)
        yaw_degrees -= 360;

    // ROS_INFO("Updated Yaw (degrees from true north after bias correction): %.2f", yaw_degrees);
}

sensor_msgs::PointCloud2 rotatePointCloudHelper(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // yaw를 라디안으로 변환하고 부호를 반전하여 회전 행렬 생성에 사용
    double yaw_radians = -yaw_degrees * M_PI / 180.0;

    // Z축 기준 회전 행렬 생성
    tf2::Matrix3x3 rotation_matrix;
    rotation_matrix.setIdentity();
    rotation_matrix[0][0] = std::cos(yaw_radians);
    rotation_matrix[0][1] = -std::sin(yaw_radians);
    rotation_matrix[1][0] = std::sin(yaw_radians);
    rotation_matrix[1][1] = std::cos(yaw_radians);

    // 회전된 포인트를 저장할 PointCloud2 생성
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

    // 각 점에 대해 순회하며 회전 연산 적용
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    sensor_msgs::PointCloud2Iterator<float> out_iter_x(rotated_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> out_iter_y(rotated_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> out_iter_z(rotated_cloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++out_iter_x, ++out_iter_y, ++out_iter_z)
    {
        tf2::Vector3 point(*iter_x, *iter_y, *iter_z);
        tf2::Vector3 rotated_point = rotation_matrix * point;

        *out_iter_x = rotated_point.x();
        *out_iter_y = rotated_point.y();
        *out_iter_z = rotated_point.z();
    }

    return rotated_cloud;
}

void buildingPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    sensor_msgs::PointCloud2 rotated = rotatePointCloudHelper(msg);
    rotated_cloud_pub.publish(rotated);
}

void roadPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    sensor_msgs::PointCloud2 rotated = rotatePointCloudHelper(msg);
    rotated_road_cloud_pub.publish(rotated);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_rotator");
    ros::NodeHandle nh;

    // 자기장 데이터 수신용 서브스크라이버
    ros::Subscriber imu_sub = nh.subscribe("/imu/mag", 10, calculateYawFromMagneticField);

    // PointCloud2 데이터 수신용 서브스크라이버들 생성
    ros::Subscriber building_pointcloud_sub = nh.subscribe("/building/points", 10, buildingPointCloudCallback);
    ros::Subscriber road_pointcloud_sub = nh.subscribe("/road/filtered", 10, roadPointCloudCallback);

    // 회전된 PointCloud2 데이터를 내보내는 퍼블리셔들 생성
    rotated_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/rotated_points", 10);
    rotated_road_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/rotated_road_points", 10);

    ROS_INFO("PointCloud rotator 노드가 시작되었음. 메시지 대기 중...");
    ros::spin();

    return 0;
}
