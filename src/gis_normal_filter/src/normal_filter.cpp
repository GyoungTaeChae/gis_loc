#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h> // VoxelGrid 필터를 위한 헤더 추가

class NormalFilterNode {
public:
    NormalFilterNode() {
        // Initialize subscriber and publisher
        point_sub_ = nh_.subscribe("/rangenet/points", 1, &NormalFilterNode::pointCloudCallback, this);
        point_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/normal/points", 1);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber point_sub_;
    ros::Publisher point_pub_;

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input) {
        // Convert ROS message to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*input, *cloud);

        // Downsampling using VoxelGrid filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(cloud);
        voxel_grid.setLeafSize(0.5f, 0.5f, 0.3f); // 0.3m voxel size
        voxel_grid.filter(*downsampled_cloud);

        // ROS_INFO("Original cloud size: %lu, Downsampled cloud size: %lu", cloud->size(), downsampled_cloud->size());

        // Normal estimation on downsampled cloud
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(downsampled_cloud);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setSearchMethod(tree);
        ne.setRadiusSearch(3.0); // Adjust radius

        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());
        ne.compute(*cloud_normals);

        // Filter points based on elevation angles
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        for (size_t i = 0; i < cloud_normals->size(); ++i) {
            const auto& normal = cloud_normals->points[i];
            double magnitude = std::sqrt(normal.normal_x * normal.normal_x +
                                         normal.normal_y * normal.normal_y +
                                         normal.normal_z * normal.normal_z);
            if (magnitude == 0) continue;

            double elevation = std::asin(normal.normal_z / magnitude); // radians
            if (elevation >= -0.1 && elevation <= 0.1) {
                inliers->indices.push_back(i);
            }
        }

        // Extract filtered points
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(downsampled_cloud); // 다운샘플링된 클라우드 사용
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*filtered_cloud);

        // Convert to ROS message and publish
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*filtered_cloud, output);
        output.header = input->header; // Preserve the original header
        point_pub_.publish(output);

        // ROS_INFO("Filtered %lu points from elevation range -0.1 to 0.1 radians.", filtered_cloud->size());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "normal_filter_node");
    ROS_INFO("normal filter ready");
    NormalFilterNode node;
    ros::spin();
    return 0;
}
