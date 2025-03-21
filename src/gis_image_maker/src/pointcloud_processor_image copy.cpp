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
#include <mutex>
#include <cmath>

#define IMAGE_SIZE 384
#define RECT_SIZE 15  // 사각형 한 변의 길이
#define RECT_SIZE2 10  // 사각형 한 변의 길이


ros::Publisher image_pub;
double max_distance = 45.0; // 기본값

// 최신 포인트 클라우드 데이터를 저장할 전역 변수
pcl::PointCloud<pcl::PointXYZ>::Ptr latest_building_cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr latest_road_cloud(new pcl::PointCloud<pcl::PointXYZ>());
std::mutex cloud_mutex;

// 두 토픽의 데이터를 한 캔버스에 합쳐 이미지를 작성하고 발행하는 함수
void drawAndPublishImage() {
    cv::Mat image(IMAGE_SIZE, IMAGE_SIZE, CV_8UC3, cv::Scalar(255, 255, 255)); // 흰색 캔버스

    // 스케일 계산
    double scale = (IMAGE_SIZE / 2.0) / max_distance;

    std::lock_guard<std::mutex> lock(cloud_mutex);


    // /rotated_road_points의 점들을 빨간색(BGR: 0,0,255) 사각형으로 표시
    for (const auto& point : latest_road_cloud->points) {
        if (std::sqrt(point.x * point.x + point.y * point.y) <= max_distance) {
            int i = static_cast<int>((point.x * scale) + IMAGE_SIZE / 2);
            int j = static_cast<int>((point.y * scale) + IMAGE_SIZE / 2);
            if (i >= 0 && i < IMAGE_SIZE && j >= 0 && j < IMAGE_SIZE) {
                cv::Point top_left(i - RECT_SIZE / 2, j - RECT_SIZE / 2);
                cv::Point bottom_right(i + RECT_SIZE / 2, j + RECT_SIZE / 2);
                cv::rectangle(image, top_left, bottom_right, cv::Scalar(0, 0, 255), -1);
            }
        }
    }

    // /rotated_points의 점들을 검은색(BGR: 0,0,0) 사각형으로 표시
    for (const auto& point : latest_building_cloud->points) {
        if (std::sqrt(point.x * point.x + point.y * point.y) <= max_distance) {
            int i = static_cast<int>((point.x * scale) + IMAGE_SIZE / 2);
            int j = static_cast<int>((point.y * scale) + IMAGE_SIZE / 2);
            if (i >= 0 && i < IMAGE_SIZE && j >= 0 && j < IMAGE_SIZE) {
                cv::Point top_left(i - RECT_SIZE2 / 2, j - RECT_SIZE2 / 2);
                cv::Point bottom_right(i + RECT_SIZE2 / 2, j + RECT_SIZE2 / 2);
                cv::rectangle(image, top_left, bottom_right, cv::Scalar(0, 0, 0), -1);
            }
        }
    }
    // 좌우 반전 처리
    cv::Mat flipped_image;
    cv::flip(image, flipped_image, 1);

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(header, "bgr8", flipped_image).toImageMsg();
    image_pub.publish(image_msg);
}

// /rotated_points 토픽의 포인트 클라우드 데이터를 처리하는 콜백 함수
void buildingPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    {
        std::lock_guard<std::mutex> lock(cloud_mutex);
        *latest_building_cloud = *cloud;
    }

    drawAndPublishImage();
}

// /rotated_road_points 토픽의 포인트 클라우드 데이터를 처리하는 콜백 함수
void roadPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    {
        std::lock_guard<std::mutex> lock(cloud_mutex);
        *latest_road_cloud = *cloud;
    }

    drawAndPublishImage();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_image_converter");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // max_distance 파라미터 확인
    private_nh.param("max_distance", max_distance, 45.0);
    ROS_INFO("Using max_distance: %f", max_distance);

    // 두 토픽의 포인트 클라우드 데이터를 구독
    ros::Subscriber building_sub = nh.subscribe("/rotated_points", 1, buildingPointCloudCallback);
    ros::Subscriber road_sub = nh.subscribe("/rotated_road_points", 1, roadPointCloudCallback);

    // 합성된 이미지를 발행할 토픽 설정
    image_pub = nh.advertise<sensor_msgs::Image>("/processed/image", 1);

    ros::spin();
    return 0;
}
