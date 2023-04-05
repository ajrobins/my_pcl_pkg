#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection2DArray.h>

class Yolov7Segmenter
{
public:
  Yolov7Segmenter()
  {
    // Initialize ROS node, publishers/subscribers
    nh_ = ros::NodeHandle("~");
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("segmented_cloud", 1);
    pub_pc_ = nh_.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);
    sub_pointcloud_ = nh_.subscribe("/xtion/depth_registered/points", 1, &Yolov7Segmenter::pointcloudCallback, this);
    sub_detection_ = nh_.subscribe("Filtered_detection", 10, &Yolov7Segmenter::detectionCallback, this);

  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Publisher pub_pc_;
  ros::Subscriber sub_detection_;
  ros::Subscriber sub_pointcloud_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;

  void detectionCallback(const vision_msgs::Detection2DArray::ConstPtr& detection_msg)
  {
    // Check that point cloud data is available
    if (!cloud_)
    {
      ROS_WARN("No point cloud data available");
      return;
    }

    // Iterate through detections in the message
    for (const auto& detection : detection_msg->detections)
    {
      // Extract the coordinates of the bounding box
      double x_min = detection.bbox.center.x - detection.bbox.size_x / 2;
      double y_min = detection.bbox.center.y - detection.bbox.size_y / 2;
      double x_max = detection.bbox.center.x + detection.bbox.size_x / 2;
      double y_max = detection.bbox.center.y + detection.bbox.size_y / 2;

      // Create a crop box filter to extract points within the bounding box
      pcl::CropBox<pcl::PointXYZRGB> crop_filter;
      crop_filter.setInputCloud(cloud_);
      Eigen::Vector4f min_pt(x_min, y_min, -1, 1);
      Eigen::Vector4f max_pt(x_max, y_max, 1, 1);
      crop_filter.setMin(min_pt);
      crop_filter.setMax(max_pt);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_bbox(new pcl::PointCloud<pcl::PointXYZRGB>);
      crop_filter.filter(*cloud_bbox);

      // Publish point cloud in bounding box for visualization/debugging
      sensor_msgs::PointCloud2 output_cloud;
      pcl::toROSMsg(*cloud_bbox, output_cloud);
      pub_.publish(output_cloud);
    }
  }

  void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
  {
    // Convert point cloud message to PCL format
    cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *cloud_);

    // Publish the original point cloud
    pub_pc_.publish(*cloud_msg);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "yolov7_segmenter");
  Yolov7Segmenter node;

  ros::spin();
}
``
