#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/synchronizer.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>




Eigen::Vector3f operator*(const tf2::Stamped<tf2::Transform>& tf, const Eigen::Vector3f& vec)
{
  tf2::Vector3 tf_vec(vec(0), vec(1), vec(2));
  tf2::Vector3 tf_res = tf * tf_vec;
  Eigen::Vector3f res(tf_res.x(), tf_res.y(), tf_res.z());
  return res;
}

using namespace std;
// using namespace sensor_msgs;
// using namespace message_filters;
using sync_pol_ = message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, vision_msgs::Detection2DArray>;
class objectSegmenter
{
public:
    objectSegmenter()//:sync(sub_cylinder_, sub_pointcloud_, 1)
    {
 // Initialize ROS node, publishers/subscribers
        nh_ = ros::NodeHandle("~");

        // Subscribe to pointcloud and filtered object detection 2D bounding boxes
        // sub_detection_ = nh_.subscribe<vision_msgs::Detection2DArray>("/choose_object/Filtered_detection", 1);
        // sub_pointcloud_ = nh_.subscribe<sensor_msgs::PointCloud2>("/xtion/depth_registered/points", 1);
        // sub_cylinder_.subscribe(nh_, "/ros_3d_bb/bb_3d", 1);
 // Create the subscribers for the pointcloud and detection messages
        // sub_pointcloud_.subscribe(nh_, "/xtion/depth_registered/points", 1);
        // sub_detection_.subscribe(nh_, "/Filtered_detection", 1);
        // marker_pub = nh_.advertise<visualization_msgs::Marker>("markers", 1);      // Create the synchronizer object using the ApproximateTime policy and register the callback function
        // message_filters::Synchronizer<sync_pol_> sync(sync_pol_(10), sub_pointcloud, sub_detection);
        // sync.registerCallback(boost::bind(&objectSegmenter::detectionCallback, this, _1, _2));
        // message_filters::Subscriber<vision_msgs::Detection2DArray> sub_detection(nh_, "/choose_object/Filtered_detection", 1);
        // message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pointcloud(nh_, "/xtion/depth_registered/points", 1);
        
        // Set up sync policy and register callback
        // Synchronizer<sync_pol_> sync(sync_pol_(10), sub_detection, sub_pointcloud);
        // sync_.reset(new Sync(ApproximateTimePolicy(10), sub_pointcloud_, sub_detection_));
        // sync.registerCallback(boost::bind(&objectSegmenter::detectionCallback, this, _1, _2));
        
        // typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, vision_msgs::Detection2DArray> MySyncPolicy;
        //   // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
        // Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_detection, sub_pointcloud);
        // sync.registerCallback(boost::bind(&objectSegmenter::detectionCallback, _1, _2));
        // Advertise segmented object topic
        pub_segmented_object_ = nh_.advertise<sensor_msgs::PointCloud2>("segmented_cloud", 1);

        // Get camera intrinsics
        camera_info_ = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/xtion/rgb/camera_info", nh_);
        
        if (camera_info_ == nullptr) {
            ROS_ERROR("Failed to get camera intrinsics....");
        }
        else {
            //focal point
            fx_ = camera_info_->K[0];
            fy_ = camera_info_->K[4];
            //image center
            cx_ = camera_info_->K[2];
            cy_ = camera_info_->K[5];
        }
    }
  
  private:
    ros::NodeHandle nh_;
    // ros::Subscriber sub_centroid_;
    // ros::Subscriber sub_detection_;
    ros::Publisher marker_pub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pointcloud_;
    message_filters::Subscriber<visualization_msgs::MarkerArray> sub_cylinder_;
    ros::Publisher pub_segmented_object_;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener{tfBuffer};
    // message_filters::TimeSynchronizer<visualization_msgs::MarkerArray, sensor_msgs::PointCloud2> sync;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
    sensor_msgs::CameraInfoConstPtr camera_info_;
    double fx_, fy_, cx_, cy_;

   
    void detectionCallback(const visualization_msgs::MarkerArray detection_msg, const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
    {
       // Convert point cloud message to PCL format
        cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*cloud_msg, *cloud_);
                float cylinder_x_min_;
                float cylinder_x_max_;
                float cylinder_y_min_;
                float cylinder_y_max_;
                float cylinder_z_min_;
                float cylinder_z_max_;

        for (const auto& marker : detection_msg.markers) {
            if (marker.type == visualization_msgs::Marker::CYLINDER) {
                if (marker.ns == "0") {
                cylinder_x_min_ = marker.pose.position.x - marker.scale.x / 2.0;
                cylinder_x_max_ = marker.pose.position.x + marker.scale.x / 2.0;
                cylinder_y_min_ = marker.pose.position.y - marker.scale.y / 2.0;
                cylinder_y_max_ = marker.pose.position.y + marker.scale.y / 2.0;
                cylinder_z_min_ = marker.pose.position.z - marker.scale.z / 2.0;
                cylinder_z_max_ = marker.pose.position.z + marker.scale.z / 2.0;
                }
            }
        }
            // Filter the point cloud by only including points inside the cylinders defined by the markers
        pcl::PassThrough<pcl::PointXYZRGB> pass_filter;
        pass_filter.setInputCloud(cloud_);
        pass_filter.setFilterFieldName("x");
        pass_filter.setFilterLimits(cylinder_x_min_, cylinder_x_max_);
        pass_filter.filter(*cloud_);
        pass_filter.setInputCloud(cloud_);
        pass_filter.setFilterFieldName("y");
        pass_filter.setFilterLimits(cylinder_y_min_, cylinder_y_max_);
        pass_filter.filter(*cloud_);
        pass_filter.setInputCloud(cloud_);
        pass_filter.setFilterFieldName("z");
        pass_filter.setFilterLimits(cylinder_z_min_, cylinder_z_max_);
        pass_filter.filter(*cloud_);

        // Convert the filtered point cloud back to a message and publish it
        sensor_msgs::PointCloud2 filtered_msg;
        pcl::toROSMsg(*cloud_, filtered_msg);
        filtered_msg.header = detection_msg.markers[0].header;
        pub_segmented_object_.publish(filtered_msg);
        // // Get the transformation matrix from the camera frame to the robot's base frame
        // tf2::Stamped<tf2::Transform> transform;
        // try {
        //     geometry_msgs::TransformStamped transformStampedMsg = tfBuffer.lookupTransform("xtion_rgb_optical_frame", "xtion_rgb_optical_frame", ros::Time(0));
        //     tf2::fromMsg(transformStampedMsg, transform);

        //      // Print out the transformation matrix
        //     ROS_INFO("Transformation matrix:");
        //     ROS_INFO("%f %f %f %f", transform.getBasis().getRow(0).getX(), transform.getBasis().getRow(0).getY(), transform.getBasis().getRow(0).getZ(), transform.getOrigin().getX());
        //     ROS_INFO("%f %f %f %f", transform.getBasis().getRow(1).getX(), transform.getBasis().getRow(1).getY(), transform.getBasis().getRow(1).getZ(), transform.getOrigin().getY());
        //     ROS_INFO("%f %f %f %f", transform.getBasis().getRow(2).getX(), transform.getBasis().getRow(2).getY(), transform.getBasis().getRow(2).getZ(), transform.getOrigin().getZ());
        //     ROS_INFO("%f %f %f %f", 0.0, 0.0, 0.0, 1.0);
        // }
        // catch (tf2::TransformException& ex) {
        //     ROS_WARN("FAILED: transform point from /xtion_rgb_optical_frame to base_link: %s", ex.what());
        //     return;
        // }
        // for (const auto& marker : msg->markers) {
        //     if (marker.type == visualization_msgs::Marker::CYLINDER) {
        //         if (marker.ns == "object") {

        // Iterate through detections in the message
        // for (const auto& detection : detection_msg->detections)
        // {
        //     // Create a point representing the center of the bounding box in xtion_rgb_optical_frame
        //     float cx = detection.bbox.center.x;
        //     float cy = detection.bbox.center.y;

        //     ROS_INFO("bbox cx = %f, cy = %f", cx, cy);
        //     // Get the depth value at the center of the bounding box
        //     pcl::PointXYZRGB point;
        //     pcl::PointXYZRGB nan_point;
        //     //nan_points used as placeholder point so centroid doesnt have random values
        //     nan_point.x = std::numeric_limits<float>::quiet_NaN();
        //     nan_point.y = std::numeric_limits<float>::quiet_NaN();
        //     nan_point.z = std::numeric_limits<float>::quiet_NaN();

        //     if (cx >= 0 && cx < cloud_->width && cy >= 0 && cy < cloud_->height)
        //         point = cloud_->at(cx, cy);
        //     else
        //         point = nan_point;


        //     float z = point.z;


        //     // Convert pixel coordinates to 3D coordinates in the camera frame
        //     float x = (cx_ - cx) * z / fx_;
        //     float y = (cy_ - cy) * z / fy_;

        //      // Transform 3D coordinates to the robot's coordinate frame
        //     Eigen::Vector3f point_cam_frame(x, y, z);
        //     Eigen::Vector3f point_robot_frame = transform * point_cam_frame;
        //     publish_marker(point_cam_frame, "xtion_rgb_optical_frame", marker_pub);
        //     ROS_INFO("Point in robot frame: x=%f, y=%f, z=%f", point_robot_frame.x(), point_robot_frame.y(), point_robot_frame.z());            // Segment the object from its surroundings in the point cloud
        //     pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        //     for (const auto& p : cloud_->points)
        //     {
        //         if (std::abs(p.x - point_robot_frame.x()) < 0.1 && std::abs(p.y - point_robot_frame.y()) < 0.1 && std::abs(p.z - point_robot_frame.z()) < 0.1)
        //         {
        //             ROS_INFO("Adding point to segmented cloud");
        //             object_cloud->push_back(p);
        //         }
        //     }

        //     // Publish the segmented object as a point cloud
        //     sensor_msgs::PointCloud2 object_cloud_msg;
        //     pcl::toROSMsg(*object_cloud, object_cloud_msg);
        //     object_cloud_msg.header.frame_id = "base_link";
        //     object_cloud_msg.header.stamp = ros::Time::now();
        //     pub_segmented_object_.publish(object_cloud_msg);

        // }
    }
    void publish_marker(const Eigen::Vector3f& point_robot_frame, const std::string& frame_id, const ros::Publisher& marker_pub)
    {
        // Create a marker message
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "my_namespace";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        // Set marker pose and scale
        marker.pose.position.x = point_robot_frame(0);
        marker.pose.position.y = point_robot_frame(1);
        marker.pose.position.z = point_robot_frame(2);
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        // Set marker color
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        // Publish marker
        marker_pub.publish(marker);
    }

};
int main(int argc, char** argv)
{
  cout << "TEST1";
  ros::init(argc, argv, "objectSegmenter_node");
  objectSegmenter node;

  ros::spin();
}
