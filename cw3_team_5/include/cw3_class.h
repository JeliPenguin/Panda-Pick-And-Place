/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw3_team_<your_team_number> package */

// include guards, prevent .h file being defined multiple times (linker error)
#ifndef CW3_CLASS_H_
#define CW3_CLASS_H_

// system includes
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// PCL specific includes
#include <pcl/filters/conditional_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <tf/transform_listener.h>
#include <pcl/features/moment_of_inertia_estimation.h>

// Set default pointcloud types
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

// include services from the spawner package - we will be responding to these
#include "cw3_world_spawner/Task1Service.h"
#include "cw3_world_spawner/Task2Service.h"
#include "cw3_world_spawner/Task3Service.h"

// // include any services created in this package
// #include "cw3_team_x/example.h"

class cw3
{
public:

  /* ----- class member functions ----- */
  
  //ROS subscriber for the input point cloud
  ros::Subscriber color_cloud_;
  ros::Subscriber point_cloud_;
  
  /** \brief Node handle. */
  ros::NodeHandle g_nh;
  
  /** \brief ROS publishers. */
  ros::Publisher g_pub_cloud;
  
  /** \brief ROS pose publishers. */
  ros::Publisher g_pub_pose;

  // constructor
  cw3(ros::NodeHandle nh);

  // service callbacks for tasks 1, 2, and 3
  bool 
  t1_callback(cw3_world_spawner::Task1Service::Request &request,
    cw3_world_spawner::Task1Service::Response &response);
  bool 
  t2_callback(cw3_world_spawner::Task2Service::Request &request,
    cw3_world_spawner::Task2Service::Response &response);
  bool 
  t3_callback(cw3_world_spawner::Task3Service::Request &request,
    cw3_world_spawner::Task3Service::Response &response);

  /* ----- class member variables ----- */
  
  geometry_msgs::Pose
  moveAbove(geometry_msgs::Point point, float angle);
  
  geometry_msgs::Pose
  moveAbovePose(geometry_msgs::Point point);
  
  bool 
  moveArm(geometry_msgs::Pose target_pose);
  
  bool 
  moveGripper(float width);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
  cloudCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& msg);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud;
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
  filterPointCloudByColor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud);
  
  bool
  pick(geometry_msgs::Point object, 
                  geometry_msgs::Point Goal,
                  float angle);
  bool 
  t1(geometry_msgs::Point object, 
            geometry_msgs::Point target, 
            std::string shape_type, 
            double size=0.04);

  void
  addCollision(std::string object_name,
                geometry_msgs::Point centre, 
                geometry_msgs::Vector3 dimensions,
                geometry_msgs::Quaternion orientation);

  void
  removeCollision(std::string object_name);

  void
  addGroundCollision();

  ros::NodeHandle nh_;
  ros::ServiceServer t1_service_;
  ros::ServiceServer t2_service_;
  ros::ServiceServer t3_service_;

  std::string base_frame_ = "panda_link0";
  
   /** \brief MoveIt interface to move groups to seperate the arm and the gripper,
  * these are defined in urdf. */
  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
  moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};

  /** \brief MoveIt interface to interact with the moveit planning scene 
    * (eg collision objects). */
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  
  
};

// Angle offset to align gripper with cube
  double angle_offset_ = 3.14159 / 4.0;
  double gripper_open_ = 80e-3;
  double gripper_closed_ = 0.0;
  double camera_offset_ = -0.04;

#endif // end of include guard for cw3_CLASS_H_
