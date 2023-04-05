#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <visualization_msgs/Marker.h>
#include <pcl/visualization/pcl_visualizer.h>
// #include <tf2/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
#include <std_msgs/Int32MultiArray.h>
using namespace std;

// Function to transform a point from the "xtion_rgb_optical_frame" frame to the "base_link" frame
bool transform_point(const tf2_ros::Buffer& tfBuffer, const geometry_msgs::PointStamped& point_in, geometry_msgs::PointStamped& point_out)
{
    try
    {
        // Create a new stamped point with the correct frame ID
        geometry_msgs::PointStamped point_transformed;
        point_transformed.header.frame_id = "base_link";

        // Transform the point from xtion_rgb_optical_frame to base_link
        tfBuffer.transform(point_in, point_transformed, "base_link");

        // Copy the transformed point to the output parameter and return success
        point_out = point_transformed;
        return true;
    }
    catch(tf2::TransformException& ex)
    {
        // Handle the exception and return failure
        ROS_WARN("Failed to transform point from xtion_rgb_optical_frame to base_link: %s", ex.what());
        return false;
    }
}

class Yolov7Segmenter
{
public:
  Yolov7Segmenter()
  {
    std::printf("TEST2");

    // Initialize ROS node, publishers/subscribers
    nh_ = ros::NodeHandle("~");
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("segmented_cloud", 1);
    pub_pc_ = nh_.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);
    pub_vis = nh_.advertise<sensor_msgs::PointCloud2>("point_cloud_vis", 1);
    sub_pointcloud_ = nh_.subscribe("/xtion/depth_registered/points", 1, &Yolov7Segmenter::pointcloudCallback, this);
    sub_detection_ = nh_.subscribe("/choose_object/Filtered_detection", 10, &Yolov7Segmenter::detectionCallback, this);
    pub_marker = nh_.advertise<visualization_msgs::Marker>("markers", 1);
    pub_bboxArray = nh_.advertise<std_msgs::Int32MultiArray>("bboxArray", 1);
    
    // Create a transform buffer and listener for transforming bbox point to base_link frame
    tf2_ros::TransformListener tfListener(tfBuffer);
   

  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Publisher pub_pc_;
  ros::Publisher pub_vis;
  ros::Publisher pub_marker; 
  ros::Publisher pub_bboxArray; 
  ros::Subscriber sub_detection_;
  ros::Subscriber sub_pointcloud_;
  const std::string from_frame = "xtion_rgb_optical_frame";
  const std::string to_frame = "base_link";
  tf2_ros::Buffer tfBuffer;

  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
  void createBoundingBoxMarker(double xmin, double xmax, double ymin, double ymax, int id, std::string ns)
  {
     visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link"; // set the target frame
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05; // line width
    marker.color.r = 1.0;
    marker.color.a = 1.0;

    geometry_msgs::PointStamped point_in, point_out;
    point_in.header.frame_id = "xtion_rgb_optical_frame";
    point_in.header.stamp = ros::Time::now();
    point_in.point.x = xmin;
    point_in.point.y = ymin;
    point_in.point.z = 0.0;

    // Get the transform from "xtion_rgb_optical_frame" to "base_link"
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    geometry_msgs::TransformStamped transform;
    try {
        transform = tf_buffer.lookupTransform("base_link", "xtion_rgb_optical_frame", ros::Time(0), ros::Duration(1.0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return;
    }

    // Apply the transform to the point
    tf2::doTransform(point_in, point_out, transform);

    // Use the transformed point to create the marker
    geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8;
    p1.x = point_out.point.x; p1.y = point_out.point.y; p1.z = point_out.point.z;
    p2.x = xmax; p2.y = ymin; p2.z = 0.0;
    p3.x = xmax; p3.y = ymin; p3.z = 0.0;
    p4.x = xmax; p4.y = ymax; p4.z = 0.0;
    p5.x = xmax; p5.y = ymax; p5.z = 0.0;
    p6.x = xmin; p6.y = ymax; p6.z = 0.0;
    p7.x = xmin; p7.y = ymax; p7.z = 0.0;
    p8.x = xmin; p8.y = ymin; p8.z = 0.0;

    marker.points.push_back(p1); marker.points.push_back(p2);
    marker.points.push_back(p3); marker.points.push_back(p4);
    marker.points.push_back(p5); marker.points.push_back(p6);

    pub_marker.publish(marker);

  }
  void detectionCallback(const vision_msgs::Detection2DArray::ConstPtr& detection_msg)
  {
    ROS_INFO("TEST detectioncb");
    // Check that point cloud data is available
    if (!cloud_)
    {
      ROS_WARN("No point cloud data available");
      return;
    }
    //std::printf("TEST3");

   

    // Iterate through detections in the message
    for (const auto& detection : detection_msg->detections)
    {
     // Create a point representing the center of the bounding box in xtion_rgb_optical_frame
      geometry_msgs::PointStamped point_in;
      point_in.header.frame_id = from_frame;
      point_in.point.x = detection.bbox.center.x; // Replace with your actual coordinate values
      point_in.point.y = detection.bbox.center.y;
      point_in.point.z = 0.0;


      // Transform the point to the base_link frame
      geometry_msgs::PointStamped point_out;
      if (transform_point(tfBuffer, point_in, point_out))
      {
          pub_marker.publish(point_out);

          // Print the transformed point
          ROS_INFO("Transformed point: (%f, %f, %f)", point_out.point.x, point_out.point.y, point_out.point.z);
      }

      std_msgs::Int32MultiArray array;

      //clear array
      array.data.clear();

      double x_min = detection.bbox.center.x - detection.bbox.size_x / 2;
      double y_min = detection.bbox.center.y - detection.bbox.size_y / 2;
      double x_max = detection.bbox.center.x + detection.bbox.size_x / 2;
      double y_max = detection.bbox.center.y + detection.bbox.size_y / 2;
      cout << "x_min = " << x_min << ", y_min = " << y_min << ", x_max = " << x_max << ", y_max = " << y_max << "\n";
      
      array.data.push_back(x_min);
      array.data.push_back(y_min);
      array.data.push_back(x_max);
      array.data.push_back(y_max);

      pub_bboxArray.publish(array);

      
      //vision_msgs::msg::ObjectHypothesisWithPose result = detection.results[0];
      //std::printf("TEST");
      //int id = std::stoi(results.id);
      //ROS_INFO("Detected object with id: %d", id);
      // Create a crop box filter to extract points within the bounding box
      pcl::CropBox<pcl::PointXYZRGB> crop_filter;
      crop_filter.setInputCloud(cloud_);
      Eigen::Vector4f min_pt(x_min, y_min, -1, 1);
      Eigen::Vector4f max_pt(x_max, y_max, 1, 1);
      crop_filter.setMin(min_pt);
      crop_filter.setMax(max_pt);
      
    //createBoundingBoxMarker(x_min, x_max, y_min, y_max, 1 ,"BOX");
    

      //       // Create a visualization marker for the crop box
      // visualization_msgs::Marker crop_box_marker;
      // crop_filter.getCropBox(crop_box_marker);
      // crop_box_marker.header.frame_id = cloud_->header.frame_id;
      // crop_box_marker.header.stamp = ros::Time::now();
      // crop_box_marker.ns = "crop_box";
      // crop_box_marker.action = visualization_msgs::Marker::ADD;
      // crop_box_marker.pose.orientation.w = 1.0;
      // crop_box_marker.color.r = 1.0;
      // crop_box_marker.color.a = 0.5;

      // // Publish the marker for visualization
      // pub_vis.publish(crop_box_marker);

   

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_bbox(new pcl::PointCloud<pcl::PointXYZRGB>);
      crop_filter.filter(*cloud_bbox);
      ROS_INFO("Number of points in filtered cloud: %lu", cloud_bbox->size());

      // Publish point cloud in bounding box for visualization/debugging
      sensor_msgs::PointCloud2 output_cloud;
      pcl::toROSMsg(*cloud_bbox, output_cloud);
      pub_.publish(output_cloud);

      //          // Visualize the filtered points
      // pcl::visualization::PCLVisualizer viewer("Crop Box Viewer");
      // viewer.addPointCloud(cloud_, "original_cloud");
      // viewer.addPointCloud(cloud_bbox, "filtered_cloud");
      // viewer.spin();



      //       // Create a PointXYZRGB object for visualizing the min/max points
      // pcl::PointCloud<pcl::PointXYZRGB>::Ptr min_max_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      // pcl::PointXYZRGB min_pt(255, 0, 0); // Red color for minimum point
      // min_pt.x = x_min;
      // min_pt.y = y_min;
      // min_max_cloud->push_back(min_pt);
      // pcl::PointXYZRGB max_pt(0, 0, 255); // Blue color for maximum point
      // max_pt.x = x_max;
      // max_pt.y = y_max;
      // min_max_cloud->push_back(max_pt);

      // // Publish the point cloud for visualizing the min/max points
      // sensor_msgs::PointCloud2 min_max_cloud_msg;
      // pcl::toROSMsg(*min_max_cloud, min_max_cloud_msg);
      // pub_.publish(min_max_cloud_msg);

            
    }
  }

  void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
  {
    //ROS_INFO("PCCB TEST");
    // Convert point cloud message to PCL format
    cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *cloud_);

    // Publish the original point cloud
    pub_pc_.publish(*cloud_msg);
  }
};

int main(int argc, char** argv)
{
  cout << "TEST1";
  ros::init(argc, argv, "yolov7_segmenter");
  Yolov7Segmenter node;
  
  ros::spin();
}

