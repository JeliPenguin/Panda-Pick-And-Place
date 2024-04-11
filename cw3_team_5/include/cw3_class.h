/*
Copyright (c) [2024] [Zeyu Chen, Yuzhou Chen, Jeffrey Li]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

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
#include <pcl/kdtree/kdtree_flann.h>
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
#include <pcl/registration/icp.h>

#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

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
  
   /** \brief Node handle. */
  ros::NodeHandle g_nh;
  
  /** \brief ROS publishers. */
  ros::Publisher g_pub_cloud;

  /** \brief ROS publisher for publishing the entire scene's point cloud data. */
  ros::Publisher full_scene_pub_cloud;

  /** \brief ROS publisher for publishing the OctoMap. */
  ros::Publisher octomap_publisher;

  /** \brief ROS publisher for publishing visualization markers. */
  ros::Publisher marker_pub;
  
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
  
  /** \brief Gripper pose calculation. */
  geometry_msgs::Pose
  moveAbove(geometry_msgs::Point point, float angle);
  
  /** \brief Gripper pose calculation. */
  geometry_msgs::Pose
  moveAbovePose(geometry_msgs::Point point);
  
  /** \brief Move to the target pose. */
  bool 
  moveArm(geometry_msgs::Pose target_pose);

  /** \brief Path planning. */
  bool 
  moveArmVertical(geometry_msgs::Pose target_pose, float angle_tol=0.1, float ori_weight=0.25, float radius=0.02, float pos_weight=0.25);
  
  /** \brief Moves the gripper. */
  bool 
  moveGripper(float width);
  
  /** \brief Computes the optimal angle. */
  float 
  computeOptimalAngle(const PointCPtr& input_cloud, double length, float radius, std::string type);
  
  /** \brief Angle between the principal axis. */
  float 
  computeOptimalAnglePCA(const PointCPtr& input_cloud);
  
  /** \brief PointCloud call back function. */
  void
  cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
  
  /** \brief Filtered PointCloud message. */
  void
  pubFilteredPCMsg (ros::Publisher &pc_pub,
                               PointC &pc);
                               
  /** \brief Pick function. */
  bool
  pick(geometry_msgs::Point object, 
                  geometry_msgs::Point Goal,
                  float angle);
                  
  /** \brief Grasp and place. */
  void
  graspAndPlace(geometry_msgs::Point object, geometry_msgs::Point target, std::string shape_type, const PointCPtr& input_cloud, double size=0.04);
  
  /** \brief task 1 main function. */
  bool 
  t1(geometry_msgs::Point object, 
            geometry_msgs::Point target, 
            std::string shape_type);

  /** \brief Determines the shape detected in the point cloud. */
  std::string
  determineShape();

  /** \brief Determines the size in the point cloud. */ 
  float 
  determineSize(std::string type, float max_dist);

  /** \brief Euclidean distance between two 3D points. */ 
  float
  euclidDistance(geometry_msgs::Point p1,geometry_msgs::Point p2);

  /** \brief task 2 main function. */ 
  int64_t
  t2(std::vector<geometry_msgs::PointStamped>& ref_object_points, geometry_msgs::PointStamped mystery_object_point);

  /** \brief task 3 main function. */ 
  std::array<int64_t, 2>
  t3();
  
  /** \brief Transforms the point cloud. */ 
  void
  transformGraspAndPlace(geometry_msgs::Point object, geometry_msgs::Point target, std::string shape_type, const PointCPtr& input_cloud, double size=0.04);

  /** \brief Adds a collision object. */ 
  void
  addCollision(std::string object_name,
                geometry_msgs::Point centre, 
                geometry_msgs::Vector3 dimensions,
                geometry_msgs::Quaternion orientation);

  /** \brief Adds a cylinder collision. */ 
  void 
  addCylinderCollision(std::string object_name, geometry_msgs::Point centre, float height, float radius);

  /** \brief Remove a collision object. */ 
  void
  removeCollision(std::string object_name);

  /** \brief Remove obstacles. */ 
  void
  removeObstacles(int obstacle_count);

  /** \brief Adds a ground collision object. */ 
  void
  addGroundCollision(float ground_height=0.02);

  /** \brief Adds Obstacle Collision. */ 
  void
  addObstacleCollision(geometry_msgs::Point obstacle_centroid,PointCPtr &output_cloud,std::string obj_name);

  /** \brief Applies a ground filter. */ 
  void 
  applyGroundFilter(PointCPtr &input_cloud, PointCPtr &output_cloud);

  /** \brief Filters out black. */ 
  void 
  applyBlackFilter(PointCPtr &input_cloud, PointCPtr &output_cloud);

  /** \brief Applies a pass-through filter. */ 
  void
  applyPassthrough(PointCPtr &in_cloud_ptr,
                      PointCPtr &out_cloud_ptr,
                      std::string axis,
                      float threshold_upper = 0.027,
                      float threshold_lower = -0.027
                    );

  /** \brief Apply Voxel Grid filtering.
    * 
    * \input[in] in_cloud_ptr the input PointCloud2 pointer
    * \input[out] out_cloud_ptr the output PointCloud2 pointer
    */
  void
  applyVX (PointCPtr &in_cloud_ptr,
            PointCPtr &out_cloud_ptr);

  /** \brief Apply Pass Through filtering.
    * 
    * \input[in] in_cloud_ptr the input PointCloud2 pointer
    * \input[out] out_cloud_ptr the output PointCloud2 pointer
    */
  void
  applyPT (PointCPtr &in_cloud_ptr,
            PointCPtr &out_cloud_ptr);
  
  /** \brief Normal estimation.
    * 
    * \input[in] in_cloud_ptr the input PointCloud2 pointer
    */
  void
  findNormals (PointCPtr &in_cloud_ptr);
  
  /** \brief Segment Plane from point cloud.
    * 
    * \input[in] in_cloud_ptr the input PointCloud2 pointer
    */
  void
  segPlane (PointCPtr &in_cloud_ptr);
  
  /** \brief Segment Cylinder from point cloud.
    * 
    * \input[in] in_cloud_ptr the input PointCloud2 pointer
    */
  void
  segCylind (PointCPtr &in_cloud_ptr);

  std::vector<pcl::PointIndices> 
  dbscanClustering(PointCPtr &cloud);

  void
  cartesianPathPlan(geometry_msgs::Pose end_pose);

  PointCPtr
  scanEnvironment();

  void 
  publishMarker(float x, float y,float z,int id);

  std::string
  colorDetermination(float r, float g, float b);


  ros::NodeHandle nh_;
  ros::ServiceServer t1_service_;
  ros::ServiceServer t2_service_;
  ros::ServiceServer t3_service_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud_;
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

  const std::string BASE_FRAME_ = "panda_link0";
  const std::string CAMERA_FRAME_ = "color";
  const std::string GROUND_COLLISION_ = "ground";
  
   /** \brief MoveIt interface to move groups to seperate the arm and the gripper,
  * these are defined in urdf. */
  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
  moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};

  /** \brief MoveIt interface to interact with the moveit planning scene 
    * (eg collision objects). */
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  
  tf::TransformListener listener_;

  /** \brief The input point cloud frame id. */
  std::string g_input_pc_frame_id_;
  
  /** \brief ROS geometry message point. */
  geometry_msgs::PointStamped g_cyl_pt_msg;
  
  /** \brief Voxel Grid filter's leaf size. */
  double g_vg_leaf_sz;
  
  /** \brief Point Cloud (input) pointer. */
  PointCPtr g_cloud_ptr;

  /** \brief Point Cloud (filtered) pointer. */
  PointCPtr g_cloud_filtered, g_cloud_filtered2, g_cloud_filtered_color;
  
  /** \brief Point Cloud (filtered) sensros_msg for publ. */
  sensor_msgs::PointCloud2 g_cloud_filtered_msg;
  
  /** \brief Point Cloud (input). */
  pcl::PCLPointCloud2 g_pcl_pc;
  
  /** \brief Voxel Grid filter. */
  pcl::VoxelGrid<PointT> g_vx;
  
  /** \brief Pass Through filter. */
  pcl::PassThrough<PointT> g_pt;
  
  /** \brief Pass Through min and max threshold sizes. */
  double g_pt_thrs_min, g_pt_thrs_max;
  
  /** \brief KDTree for nearest neighborhood search. */
  pcl::search::KdTree<PointT>::Ptr g_tree_ptr;
  
  /** \brief Normal estimation. */
  pcl::NormalEstimation<PointT, pcl::Normal> g_ne;
  
  /** \brief Cloud of normals. */
  pcl::PointCloud<pcl::Normal>::Ptr g_cloud_normals, g_cloud_normals2;
  
  /** \brief Nearest neighborhooh size for normal estimation. */
  double g_k_nn;
  
  /** \brief SAC segmentation. */
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> g_seg; 
  
  /** \brief Extract point cloud indices. */
  pcl::ExtractIndices<PointT> g_extract_pc;

  /** \brief Extract point cloud normal indices. */
  pcl::ExtractIndices<pcl::Normal> g_extract_normals;
  
  /** \brief Point indices for plane. */
  pcl::PointIndices::Ptr g_inliers_plane;
    
  /** \brief Point indices for cylinder. */
  pcl::PointIndices::Ptr g_inliers_cylinder;
  
  /** \brief Model coefficients for the plane segmentation. */
  pcl::ModelCoefficients::Ptr g_coeff_plane;
  
  /** \brief Model coefficients for the culinder segmentation. */
  pcl::ModelCoefficients::Ptr g_coeff_cylinder;
  
  /** \brief Point cloud to hold plane and cylinder points. */
  PointCPtr g_cloud_plane, g_cloud_cylinder;

  geometry_msgs::Point crt_ee_position;

  // Angle offset to align gripper with cube
  double angle_offset_ = 3.14159 / 4.0;
  double gripper_open_ = 80e-3;
  double gripper_closed_ = 0.0;
  double camera_offset_ = -0.04;

  bool add_scene_ = false;
  
};

#endif // end of include guard for cw3_CLASS_H_
