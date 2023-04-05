#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

class ObjectSegmenter {
public:
    ObjectSegmenter() {
        // Subscribe to the PointCloud2 topic
        point_cloud_sub_ = nh_.subscribe("/xtion/depth_registered/points", 1, &ObjectSegmenter::PointCloud2Callback, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber point_cloud_sub_;

    void PointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg) {
        // Convert the PointCloud2 message to a PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*point_cloud_msg, *cloud);

        // Segment objects from the point cloud using RANSAC plane fitting
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        seg.setInputCloud(cloud);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);
        seg.segment(*inliers, *coefficients);

        // Extract the inlier points (points on the plane) from the original point cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.filter(*inlier_cloud);

        // Extract the outlier points (points not on the plane) from the original point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setNegative(true);
        extract.filter(*outlier_cloud);

        // Publish the segmented point clouds
        sensor_msgs::PointCloud2 inlier_cloud_msg;
        pcl::toROSMsg(*inlier_cloud, inlier_cloud_msg);
        inlier_cloud_msg.header = point_cloud_msg->header;
        inlier_cloud_pub_.publish(inlier_cloud_msg);

        sensor_msgs::PointCloud2 outlier_cloud_msg;
        pcl::toROSMsg(*outlier_cloud, outlier_cloud_msg);
        outlier_cloud_msg.header = point_cloud_msg->header;
        outlier_cloud_pub_.publish(outlier_cloud_msg);
    }

    ros::Publisher inlier_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("inlier_cloud", 1);
    ros::Publisher outlier_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("outlier_cloud", 1);
};

int main(int argc, char** argv)
{
  //cout << "TEST1";
  ros::init(argc, argv, "RANSAC_segment");
  ObjectSegmenter node;

  ros::spin();
}
