#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

class RoadPointsFilter
{
public:
    RoadPointsFilter()
    {
        // rangenet/roadpoints 토픽을 구독함
        sub_ = nh_.subscribe("/rangenet/roadpoints", 1, &RoadPointsFilter::pointCloudCallback, this);
        // road/filtered 토픽에 결과를 발행함
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/road/filtered", 1);
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        // ROS 메시지를 PCL 형식으로 변환함
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);

        // z 값이 0 이상인 점을 제외함
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            if (cloud->points[i].z < 0)
                cloud_filtered->points.push_back(cloud->points[i]);
        }

        cloud_filtered->header = cloud->header;
        cloud_filtered->width = cloud_filtered->points.size();
        cloud_filtered->height = 1;
        cloud_filtered->is_dense = cloud->is_dense;

        // PCL 형식의 데이터를 ROS 메시지로 변환함
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud_filtered, output);
        output.header = cloud_msg->header;

        // 결과 메시지를 발행함
        pub_.publish(output);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "road_points_filter");
    RoadPointsFilter rpf;
    ros::spin();
    return 0;
}
