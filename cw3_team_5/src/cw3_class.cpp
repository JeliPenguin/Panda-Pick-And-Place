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

#include <cw3_class.h> // Team 5

///////////////////////////////////////////////////////////////////////////////

cw3::cw3(ros::NodeHandle nh):
  g_cloud_ptr (new PointC), // input point cloud
  g_cloud_filtered (new PointC), // filtered point cloud
  g_cloud_filtered2 (new PointC), // filtered point cloud
  g_cloud_plane (new PointC), // plane point cloud
  g_cloud_cylinder (new PointC), // cylinder point cloud
  g_tree_ptr (new pcl::search::KdTree<PointT> ()), // KdTree
  g_cloud_normals (new pcl::PointCloud<pcl::Normal>), // segmentation
  g_cloud_normals2 (new pcl::PointCloud<pcl::Normal>), // segmentation
  g_inliers_plane (new pcl::PointIndices), // plane seg
  g_inliers_cylinder (new pcl::PointIndices), // cylidenr seg
  g_coeff_plane (new pcl::ModelCoefficients), // plane coeff
  g_coeff_cylinder (new pcl::ModelCoefficients) // cylinder coeff
{
  /* class constructor */

  nh_ = nh;

  // advertise solutions for coursework tasks
  t1_service_  = nh_.advertiseService("/task1_start", 
    &cw3::t1_callback, this);
  t2_service_  = nh_.advertiseService("/task2_start", 
    &cw3::t2_callback, this);
  t3_service_  = nh_.advertiseService("/task3_start",
    &cw3::t3_callback, this);
    
  // Define the publishers
  g_pub_cloud = nh_.advertise<sensor_msgs::PointCloud2> ("filtered_cloud", 1, true);
  full_scene_pub_cloud = nh_.advertise<sensor_msgs::PointCloud2> ("full_scene", 1, true);

  g_pub_pose = nh_.advertise<geometry_msgs::PointStamped> ("cyld_pt", 1, true);
  octomap_publisher = nh.advertise<octomap_msgs::Octomap>("/octomap", 1);

  marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  
  // Define public variables
  g_vg_leaf_sz = 0.01; // VoxelGrid leaf size: Better in a config file
  g_pt_thrs_min = 0.0; // PassThrough min thres: Better in a config file
  g_pt_thrs_max = 0.7; // PassThrough max thres: Better in a config file
  g_k_nn = 50; // Normals nn size: Better in a config file

  crt_ee_position.x = -9999;

  ROS_INFO("cw3 class initialised");
}

///////////////////////////////////////////////////////////////////////////////

bool
cw3::t1_callback(cw3_world_spawner::Task1Service::Request &request,
  cw3_world_spawner::Task1Service::Response &response) 
{
  /* function which should solve task 1 */

  ROS_INFO("The coursework solving callback for task 1 has been triggered");
  
  bool success = t1(request.object_point.point, request.goal_point.point, request.shape_type);

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw3::t2_callback(cw3_world_spawner::Task2Service::Request &request,
  cw3_world_spawner::Task2Service::Response &response)
{
  /* function which should solve task 2 */

  ROS_INFO("The coursework solving callback for task 2 has been triggered");

  int64_t number = t2(request.ref_object_points,request.mystery_object_point);

  ROS_INFO("The mystery object num is: %ld",number);

  response.mystery_object_num = number;

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw3::t3_callback(cw3_world_spawner::Task3Service::Request &request,
  cw3_world_spawner::Task3Service::Response &response)
{
  /* function which should solve task 3 */

  ROS_INFO("The coursework solving callback for task 3 has been triggered");

  std::array<int64_t, 2> res = t3();

  response.total_num_shapes = res[0];
  response.num_most_common_shape = res[1];

  ROS_INFO("Total number of shapes: %ld",res[0]);
  ROS_INFO("Number of most common shape: %ld",res[1]);

  return true;
}

///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Callback function for handling PointCloud2 messages.
 * 
 * This function is called whenever a PointCloud2 message is received.
 * It performs the necessary conversions and processing on the received message.
 * 
 * @param msg Pointer to the received PointCloud2 message.
 */
void 
cw3::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
  // ROS_INFO("PointCloud2 message received");

  // Convert from ROS sensor_msgs/PointCloud2 to PCL PCLPointCloud2
  pcl::PCLPointCloud2 pcl_cloud;
  pcl_conversions::toPCL(*msg, pcl_cloud);

  // Reset the pointer to the PointCloud<pcl::PointXYZRGBA> to store the converted point cloud
  pcl::fromPCLPointCloud2(pcl_cloud, *g_cloud_ptr);

}

////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Publishes a filtered PointCloud message.
 * 
 * This function converts the input PointCloud data to a ROS PointCloud2 message
 * and publishes it using the provided ROS publisher.
 * 
 * @param pc_pub Reference to the ROS publisher for the PointCloud message.
 * @param pc Reference to the filtered PointCloud data to be published.
 */
void 
cw3::pubFilteredPCMsg(ros::Publisher &pc_pub, PointC &pc) {
  // Convert PointCloud data to ROS PointCloud2 message
  sensor_msgs::PointCloud2 g_cloud_msg;
  pcl::toROSMsg(pc, g_cloud_msg);

  // Publish the PointCloud2 message
  pc_pub.publish(g_cloud_msg);

  return;
}

///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Generates a geometry_msgs::Pose for placing the gripper above a specified point.
 * 
 * This function calculates a gripper pose with the gripper facing downwards, positioned above the specified point.
 * 
 * @param point The XYZ coordinates of the point below which the gripper should be positioned.
 * @param angle The angle offset from the default orientation of the gripper.
 * @return A geometry_msgs::Pose representing the gripper pose.
 */
geometry_msgs::Pose 
cw3::moveAbove(geometry_msgs::Point point, float angle) {
  // This function produces a "gripper facing down" pose given an xyz point
  
  // Position gripper above point
  tf2::Quaternion q_x180deg(1, 0, 0, 0);
  
  // Calculate gripper orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_ + angle);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion orientation = tf2::toMsg(q_result);

  // Set the desired Pose
  geometry_msgs::Pose pose;
  pose.position = point;
  pose.orientation = orientation;

  return pose;
}


///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Moves the robotic arm to a specified target pose.
 * 
 * This function sets up the target pose for the arm, plans a movement path, and executes the planned path.
 * 
 * @param target_pose The target pose to which the arm should move.
 * @return True if the arm successfully moves to the target pose, false otherwise.
 */
bool 
cw3::moveArm(geometry_msgs::Pose target_pose) {
  // Setup the target pose
  ROS_INFO("Setting pose target");
  arm_group_.setPoseTarget(target_pose);

  // Create a movement plan for the arm
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Output plan visualization status
  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // Update current end-effector position if planning successful
  crt_ee_position = target_pose.position;

  // Execute the planned path
  arm_group_.move();

  return success;
}

///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Moves the robotic arm to a specified target pose vertically while maintaining its current orientation.
 * 
 * This function configures and applies orientation and position constraints to ensure that the arm moves vertically 
 * to the target pose, maintaining its current orientation throughout the motion. The function sets up the target pose 
 * with these constraints, plans a movement path within these constraints, and executes the planned path if successful.
 * 
 * @param target_pose The target pose to which the arm should move vertically. The orientation of the target pose 
 * is used to align the position constraint region, and the position defines the final location of the end-effector.
 * @return True if the arm successfully moves to the target pose within the defined constraints, false otherwise.
 */
 
bool cw3::moveArmVertical(geometry_msgs::Pose target_pose, float angle_tol, float ori_weight, float radius, float pos_weight) {
    // Setup the target pose with orientation and position constraints
    ROS_INFO("Setting vertical pose target with orientation and position constraints");

    int total_steps = 50;
    float step_size = 1/static_cast<float>(total_steps);
    float x = target_pose.position.x - crt_ee_position.x;
    float y = target_pose.position.y - crt_ee_position.y;
    float z = target_pose.position.z - crt_ee_position.z;

    std::cout<<"Step size: "<<step_size<<std::endl;
    std::cout<<"Vector: "<<"x: "<<x<<"y: "<<y<<"z: "<<z<<std::endl;


    std::vector<geometry_msgs::Pose> waypoints;  

    waypoints.push_back(target_pose);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 6;
    const double eef_step = 0.005;
    double fraction = arm_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    if (fraction > 0.9){
      // Execute trajectory
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      my_plan.trajectory_ = trajectory;
      arm_group_.execute(my_plan);

      crt_ee_position = target_pose.position;

      return true;
    }

    else{

      // Define the orientation constraint to maintain the current orientation during vertical movement
      moveit_msgs::OrientationConstraint ocm;
      ocm.link_name = arm_group_.getEndEffectorLink();
      ocm.header.frame_id = arm_group_.getPlanningFrame();
      ocm.orientation = arm_group_.getCurrentPose().pose.orientation; // Use current orientation to maintain it
      ocm.absolute_x_axis_tolerance = angle_tol;
      ocm.absolute_y_axis_tolerance = angle_tol;
      ocm.absolute_z_axis_tolerance = angle_tol; // Allow rotation around the Z-axis if needed
      ocm.weight = ori_weight;

      // Define the position constraint using a cylinder for minimal x and y movement
      moveit_msgs::PositionConstraint pcm;
      pcm.link_name = arm_group_.getEndEffectorLink();
      pcm.header.frame_id = arm_group_.getPlanningFrame();
      shape_msgs::SolidPrimitive primitive;
      primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
      primitive.dimensions.resize(2); // CYLINDER has 2 dimensions: height (H) and radius (R)
      primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = 1.0; // Large height for vertical movement
      primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = radius; // Radius for minimal lateral movement

      geometry_msgs::Pose cylinder_pose; // Position constraint centered on target_pose for x and y, allows vertical movement
      cylinder_pose.position.x = target_pose.position.x;
      cylinder_pose.position.y = target_pose.position.y;
      cylinder_pose.position.z = arm_group_.getCurrentPose().pose.position.z; // Use current z position as reference
      cylinder_pose.orientation = target_pose.orientation; // Use target orientation for alignment

      pcm.constraint_region.primitives.push_back(primitive);
      pcm.constraint_region.primitive_poses.push_back(cylinder_pose);
      pcm.weight = pos_weight;

      // Add both the orientation and position constraints to the move group
      moveit_msgs::Constraints constraints;
      constraints.orientation_constraints.push_back(ocm);
      constraints.position_constraints.push_back(pcm);
      arm_group_.setPathConstraints(constraints);

      // Set the target pose for vertical movement
      arm_group_.setPoseTarget(target_pose);

      // Attempt to plan and execute the path within the defined constraints
      ROS_INFO("Attempting to plan the vertical path with combined constraints");
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = (arm_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      // Output the planning status
      ROS_INFO("Visualising vertical plan %s", success ? "with combined constraints" : "FAILED with combined constraints");

      // Execute the planned path if successful
      if (success) {
          arm_group_.move();
          crt_ee_position = target_pose.position; // Update the current end-effector position
          // Clear the path constraints after planning and execution   
          arm_group_.clearPathConstraints();
      }

      return success;
    }
}


///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Generates a geometry_msgs::Pose for positioning the gripper above a specified point.
 * 
 * This function calculates a gripper pose with the gripper facing downwards and positioned above the specified point.
 * 
 * @param point The XYZ coordinates of the point below which the gripper should be positioned.
 * @return A geometry_msgs::Pose representing the gripper pose.
 */
geometry_msgs::Pose 
cw3::moveAbovePose(geometry_msgs::Point point) {
  // Create a quaternion representing a 180-degree rotation along the X-axis for "gripper facing down" orientation.
  tf2::Quaternion q_x180deg(1, 0, 0, 0); // Gripper Position

  // Apply additional rotation based on angle_offset_ to adjust the gripper's orientation.
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_); // Additional orientation adjustment
  tf2::Quaternion q_result = q_x180deg * q_object; // Combine rotations
  geometry_msgs::Quaternion orientation = tf2::toMsg(q_result); // Convert to message

  // Set and return the desired pose with the calculated orientation.
  geometry_msgs::Pose pose;
  pose.position = point; // Target position
  pose.orientation = orientation; // Set orientation

  return pose;
}


///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Moves the gripper to a specified width.
 * 
 * This function adjusts the width of the gripper to the specified value by moving its joints.
 * 
 * @param width The desired width of the gripper.
 * @return True if the gripper successfully moves to the specified width, false otherwise.
 */
bool 
cw3::moveGripper(float width) {
  // Safety checks in case width exceeds safe values
  if (width > gripper_open_) 
    width = gripper_open_;
  if (width < gripper_closed_) 
    width = gripper_closed_;

  // Calculate the joint targets as half each of the requested distance
  double eachJoint = width / 2.0;

  // Create a vector to hold the joint target for each joint
  std::vector<double> gripperJointTargets(2);
  gripperJointTargets[0] = eachJoint;
  gripperJointTargets[1] = eachJoint;

  // Apply the joint target
  hand_group_.setJointValueTarget(gripperJointTargets);

  // Move the robot hand
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (hand_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Output plan visualization status
  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // Move the gripper joints
  hand_group_.move();

  return success;
}


///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Picks up an object from a specified position and places it at a goal position.
 * 
 * This function performs a pick-and-place operation for an object. It calculates the grasp pose above the object,
 * adjusts the pose for picking up and releasing the object, executes the pick-and-place operation, and opens the gripper.
 * 
 * @param object The position of the object to be picked up.
 * @param Goal The goal position where the object should be placed.
 * @param angle The angle offset for adjusting the gripper orientation during picking.
 * @return True if the pick-and-place operation is successful, false otherwise.
 */
bool 
cw3::pick(geometry_msgs::Point object, geometry_msgs::Point Goal, float angle) {
  // Calculate the grasp pose above the object with the specified angle
  geometry_msgs::Pose grasp_pose = moveAbove(object, angle);
  grasp_pose.position.z += 0.14; // Adjust the height for grasping
  
  // Create offset pose for picking up the object
  geometry_msgs::Pose offset_pose = grasp_pose;
  offset_pose.position.z += 0.2;
  
  // Calculate the release pose above the goal position
  geometry_msgs::Pose release_pose = moveAbovePose(Goal);
  release_pose.position.z += 0.5; // Adjust the height for releasing


  geometry_msgs::Pose approach_pose = moveAbove(object, angle);
  approach_pose.position.z = 0.5; 

  // Move above the object
  addGroundCollision(0.06f);
  moveArm(offset_pose);
  // cartesianPathPlan(offset_pose);
  removeCollision(GROUND_COLLISION_);

  addGroundCollision();
  // Close gripper to pick up the object
  moveGripper(gripper_open_);
  moveArmVertical(grasp_pose, 0.1f, 0.35f, 0.02f, 0.35f);
  //cartesianPathPlan(grasp_pose);
  moveGripper(gripper_closed_);

  // Move the arm to take away the object
  offset_pose.position.z += 0.26;
  moveArmVertical(offset_pose, 0.15f, 0.3f, 0.03f, 0.3f);
  // cartesianPathPlan(offset_pose);
  removeCollision(GROUND_COLLISION_);
  
  // Move the arm to place the object at the goal position
  addGroundCollision(0.26f);
  moveArm(release_pose);
  removeCollision(GROUND_COLLISION_);

  addGroundCollision();
  release_pose.position.z -= 0.15;
  moveArmVertical(release_pose, 0.3f, 0.3f, 0.1f, 0.3f);
  //cartesianPathPlan(release_pose);

  // Open gripper to release the object
  moveGripper(80e-3);

  release_pose.position.z += 0.25;
  moveArmVertical(release_pose, 0.3f, 0.3f, 0.1f, 0.3f);
  //cartesianPathPlan(release_pose);
  
  return true;
}


///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Computes the optimal angle for placing a cross or maximizing points within a specified radius.
 * 
 * This function analyzes a given point cloud to determine the optimal angle at which to either place a cross 
 * or maximize the number of points within a specified radius. The decision is based on the type specified by 
 * the user ("cross" for placing a cross with minimal overlap with points, "max" for maximizing overlap). 
 * It first filters the z-coordinate to target a specific layer of the point cloud. Then, it iterates over 
 * possible angles, performing a radius search around a point projected at each angle from the origin, 
 * to either minimize or maximize the count of points within the radius, depending on the operation type.
 * 
 * @param input_cloud A shared pointer to the input point cloud.
 * @param length The length from the origin to the search point along the specified angle.
 * @param radius The radius within which to search for points around the search point.
 * @param type A string specifying the operation type: "cross" to find an angle minimizing point count within the radius, "max" for maximizing it.
 * @return The optimal angle (in radians) for the specified operation.
 */

float 
cw3::computeOptimalAngle(const PointCPtr& input_cloud, double length, float radius, std::string type) {
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(input_cloud);

    std::vector<float> z_values;
    float z;
    z_values.reserve(input_cloud->points.size());

    // cllect all z coordinates
    for (const auto& point : input_cloud->points) {
        z_values.push_back(point.z);
    }

    // sort accoording to z
    std::sort(z_values.begin(), z_values.end());

    size_t n = z_values.size();

    // calculate the index od 1/10*n z 
    size_t index = (1 * n / 10); 
    z = z_values[index];

    float optimal_angle = 0.0f;

    if (type == "cross"){
      int min_count = 1000;

      for (int alpha = 0; alpha < 90; alpha += 1) {
          float rad = alpha * M_PI / 180.0;
          PointT searchPoint; // Use PointT, i.e. pcl::PointXYZRGBA
          searchPoint.x = length * std::cos(rad);
          searchPoint.y = length * std::sin(rad);
          searchPoint.z = z; 

          std::vector<int> pointIdxRadiusSearch;
          std::vector<float> pointRadiusSquaredDistance;
          int count = kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

          if (count < min_count) {
              min_count = count;
              optimal_angle = rad;
          }
          else if(count == min_count) {
              optimal_angle = (optimal_angle + rad)/2;
          }
        }
    }
    else{
      int max_count = 0;

      for (int alpha = 0; alpha < 90; alpha += 1) {
          float rad = alpha * M_PI / 180.0;
          PointT searchPoint; // Use PointT, i.e. pcl::PointXYZRGBA
          searchPoint.x = length * std::cos(rad);
          searchPoint.y = length * std::sin(rad);
          searchPoint.z = z; 

          std::vector<int> pointIdxRadiusSearch;
          std::vector<float> pointRadiusSquaredDistance;
          int count = kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

          if (count > max_count) {
              max_count = count;
              optimal_angle = rad;
          }
          else if(count == max_count) {
              optimal_angle = (optimal_angle + rad)/2;
          }
        }
    }


    return optimal_angle;
}

///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Computes the angle between the principal axis of a point cloud and the global Y-axis using PCA.
 * 
 * This function applies Principal Component Analysis (PCA) to the input point cloud to find its principal axis, 
 * which is the direction of maximum variance. It then computes the angle between this principal axis and the global Y-axis. 
 * This angle can be useful for understanding the orientation of objects within the point cloud or for aligning the point cloud 
 * with a reference axis.
 * 
 * The principal axis is determined by the eigenvector associated with the largest eigenvalue from the PCA. The angle is 
 * calculated as the arccosine of the dot product between the principal axis vector and the global Y-axis, normalized by 
 * their magnitudes.
 * 
 * @param input_cloud A shared pointer to the input point cloud.
 * @return The angle in radians between the point cloud's principal axis and the global Y-axis.
 */
 
float 
cw3::computeOptimalAnglePCA(const PointCPtr& input_cloud)
{
  pcl::PCA<PointT> pca;
  pca.setInputCloud(input_cloud);
  Eigen::Vector3f eigenvalues = pca.getEigenValues();
  Eigen::Matrix3f eigenvectors = pca.getEigenVectors();

  // The principal axis direction is the eigenvector corresponding to the largest eigenvalue
  Eigen::Vector3f principal_axis = eigenvectors.col(0);
  
  // Calculate angle
  Eigen::Vector3f global_y_axis(0, 1, 0);
  float cosine_of_angle = principal_axis.dot(global_y_axis) / (principal_axis.norm() * global_y_axis.norm());

  return acos(cosine_of_angle);
}

///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Grasps an object of a specified shape and size at its location and places it at a target location.
 * 
 * This function calculates the optimal angle for grasping an object based on its shape, using the point cloud data of 
 * the surrounding environment to avoid collisions and ensure a successful grasp. It distinguishes between 'cross' and 
 * 'nought' shapes, applying different strategies for angle calculation and adjusting the grasp position accordingly. 
 * After computing the angle and making necessary adjustments to ensure it is acute and within specified limits, 
 * the function then calculates positional offsets for both the object and the target to optimize the pick-and-place task.
 * Finally, it executes the pick-and-place operation at the calculated angle and with the determined offsets.
 * @param object The geometry_msgs::Point representing the object's starting position.
 * @param target The geometry_msgs::Point representing the target position for placement.
 * @param shape_type A string indicating the shape of the object ('cross' or 'nought').
 * @param input_cloud A shared pointer to the point cloud data, used for calculating the optimal grasp angle.
 * @param size The size parameter of the object, influencing the calculation of the grasp angle and positional offsets.
 */

void
cw3::graspAndPlace(geometry_msgs::Point object, geometry_msgs::Point target, std::string shape_type, const PointCPtr& input_cloud, double size)
{
  float angle_radians;

  // Calculate the angle
  if(shape_type == "cross"){
    angle_radians = computeOptimalAngle(input_cloud, 2.12*size, size, "cross") - M_PI/4; 
    
    std::cout << "Cross, the angle is: " << angle_radians/M_PI*360 << std::endl;
  }else{
    angle_radians = computeOptimalAngle(input_cloud, 3.5*size, 0.5*size, "nought") - M_PI/4;

    std::cout << "Nought, the angle is: " << angle_radians/M_PI*360 << std::endl;
    std::cout << "Original angle radians: "<<angle_radians<<std::endl;

  }

  // Avoid obtuse angles, make angles acute
  if (angle_radians > M_PI / 2) angle_radians -= M_PI;
  else if (angle_radians < -M_PI / 2) angle_radians += M_PI;

  // Restriction angle between -pi/4 and pi/4
  if (angle_radians > M_PI / 4) angle_radians -= M_PI/2;
  else if (angle_radians < -M_PI / 4) angle_radians += M_PI/2;

  std::cout<<"Final angle: "<<angle_radians<<std::endl;

  // Positional offsets for pick and place
  if(shape_type == "cross"){
    object.x += size * cos(angle_radians) ;
    object.y -= size * sin(angle_radians) ;
    target.x += size;
    
  }else{
    object.x += 2 * size * sin(angle_radians) ;
    object.y += 2 * size * cos(angle_radians) ;
    target.y += 2 * size;
  }
  
  
  pick(object, target, angle_radians);
}

///////////////////////////////////////////////////////////////////////////////

bool 
cw3::t1(geometry_msgs::Point object, 
            geometry_msgs::Point target, 
            std::string shape_type)
{
  geometry_msgs::Pose target_pose = moveAbovePose(object);
  target_pose.position.z = 0.5; 
  target_pose.position.x += camera_offset_; 

  addGroundCollision(0.2f); // higher ground collision requirement
  moveArm(target_pose);
  removeCollision(GROUND_COLLISION_);
  addGroundCollision();

  applyGroundFilter(g_cloud_ptr, g_cloud_filtered);
  pubFilteredPCMsg(g_pub_cloud, *g_cloud_filtered);
  
  graspAndPlace(object,target,shape_type,g_cloud_filtered);
  

  removeCollision(GROUND_COLLISION_);

  return true;
}

/////////////////////////////////////////////////////////////////////////

/**
 * @brief Determines the shape detected in the point cloud.
 * 
 * This function applies various filters to the point cloud to isolate the shape of interest.
 * It then analyzes the filtered point cloud to determine the detected shape.
 * 
 * @return A string indicating the determined shape ("cross" or "nought").
 */
std::string 
cw3::determineShape() {
  std::string shape = "nought";

  applyVX(g_cloud_ptr, g_cloud_filtered);

  // Apply ground filter
  applyGroundFilter(g_cloud_filtered, g_cloud_filtered);

  // Apply passthrough filters along X and Y axes
  applyPassthrough(g_cloud_filtered, g_cloud_filtered, "x");
  applyPassthrough(g_cloud_filtered, g_cloud_filtered, "y");

  // Publish the filtered point cloud
  pubFilteredPCMsg(g_pub_cloud, *g_cloud_filtered);

  // Get the number of points in the filtered cloud
  float num_points = g_cloud_filtered->size();

  std::cout<<"==================================="<<std::endl;
  // Determine shape based on the number of points
  std::cout<<"Number of inside points: "<<num_points<<std::endl;
  std::cout<<"==================================="<<std::endl;
  
  if (num_points > 10) {
    return "cross";
  }

  return "nought";
}

///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Determines the size of an object based on its type and the number of points in its point cloud.
 * 
 * This function estimates the size of an object by analyzing the point cloud associated with it. The size estimation 
 * depends on the object's type ('cross' or another specified type) and the total number of points in the point cloud. 
 * Different thresholds of point counts are used to categorize the object into one of several size categories, each associated 
 * with a predefined size value. These size values are determined based on empirical observations or predefined standards 
 * relating the point count to the physical size of the object.
 * 
 * The function provides a simplified mechanism to estimate object size, useful in scenarios where direct measurement 
 * might not be feasible or where a quick estimation is sufficient.
 * 
 * @param type A string representing the type of the object ('cross' or other types).
 * @param point_cloud A shared pointer to the object's point cloud data.
 * @return The estimated size of the object as a float, based on the analysis of its point cloud.
 */

float 
cw3::determineSize(std::string type, float max_dist) {
    double size;
    if (type == "cross") {
      size = max_dist/2.5495097567963;

    } else {
      size = max_dist/3.5355339059327;
    }

    std::cout << "Estimated Size: " << size << std::endl;
    
    // Thresholding based on the estimated size
    if (size > 0.035f){
      return 0.04f;
    }
    else if (size < 0.025f){
      return 0.02f;
    }
    else{
      return 0.03f;
    }
}


///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Calculates the Euclidean distance between two 3D points.
 * 
 * This function computes the Euclidean distance between two points represented by their x, y, and z coordinates.
 * 
 * @param p1 The first point.
 * @param p2 The second point.
 * @return The Euclidean distance between the two points.
 */
float 
cw3::euclidDistance(geometry_msgs::Point p1, geometry_msgs::Point p2) {
  // Calculate differences in coordinates
  double dx = p2.x - p1.x;
  double dy = p2.y - p1.y;
  double dz = p2.z - p1.z;

  // Compute Euclidean distance
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Determines the type of the mystery object relative to reference object points.
 * 
 * This function compares the distance between the end-effector position and the mystery object point
 * with the distance between the end-effector position and the first reference object point.
 * Based on the comparison, it selects two points for further processing.
 * It then moves the robot arm to positions above these points, determines the shapes of the objects,
 * and removes ground collision constraints before returning the result.
 * 
 * @param ref_object_points Vector containing reference object points.
 * @param mystery_object_point The mystery object point to be compared with the reference points.
 * @return 1 if the mystery object is of the same type as the reference object 1, 2 otherwise.
 */
int64_t
cw3::t2(std::vector<geometry_msgs::PointStamped>& ref_object_points, geometry_msgs::PointStamped mystery_object_point) {
  // Add ground collision constraint
  addGroundCollision();

  // Initialize vector to hold two points
  std::vector<geometry_msgs::Point> togo(2);

  // Determine which point to select based on end-effector position
  if (crt_ee_position.x != -9999) {
    if (euclidDistance(crt_ee_position, mystery_object_point.point) < euclidDistance(crt_ee_position, ref_object_points[0].point)) {
       togo[0] = mystery_object_point.point;
       togo[1] = ref_object_points[0].point;
    } else {
      togo[0] = ref_object_points[0].point;
      togo[1] = mystery_object_point.point;
    }
  } else {
    togo[0] = mystery_object_point.point;
    togo[1] = ref_object_points[0].point;
  }

  // Initialize target pose and object types array
  geometry_msgs::Pose target_pose;
  std::array<std::string, 2> object_types;

  // Move the robot arm to positions above the selected points and determine object shapes
  for (size_t i = 0; i < 2; ++i) {
      const auto& point = togo[i];
      target_pose = moveAbovePose(point);
      target_pose.position.z = 0.7; // Adjust height
      target_pose.position.x += camera_offset_; // Apply camera offset
      moveArm(target_pose);
      object_types[i] = determineShape();
  }

  // Remove ground collision constraint
  removeCollision(GROUND_COLLISION_);

  std::cout<<"Object One shape: "<<object_types[0]<<std::endl;
  std::cout<<"Object Two shape: "<<object_types[1]<<std::endl;

  // Compare the object types and return the result
  if (object_types[0] == object_types[1]) {
    return 1;
  }
  return 2;
}

///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Plans and executes a Cartesian path for the robot arm to reach a specified end pose.
 * 
 * This function computes a Cartesian path for the robot arm to move smoothly from its current end-effector position to the specified end pose.
 * The path is planned in steps to ensure precision, with the motion divided into 50 segments. The function first calculates the distance vector
 * between the current and the desired end-effector positions, and then uses this vector to generate waypoints. The Cartesian path is generated using
 * these waypoints. Once the path is successfully computed, the function executes this trajectory to move the robot arm.
 * The current position of the end-effector is updated after executing the trajectory.
 * 
 * @param end_pose The target pose for the end-effector, consisting of position and orientation coordinates.
 */

void
cw3::cartesianPathPlan(geometry_msgs::Pose end_pose)
{
  int total_steps = 50;
  float step_size = 1/static_cast<float>(total_steps);
  float x = end_pose.position.x - crt_ee_position.x;
  float y = end_pose.position.y - crt_ee_position.y;
  float z = end_pose.position.z - crt_ee_position.z;

  std::cout<<"Step size: "<<step_size<<std::endl;
  std::cout<<"Vector: "<<"x: "<<x<<"y: "<<y<<"z: "<<z<<std::endl;


  std::vector<geometry_msgs::Pose> waypoints;  

  waypoints.push_back(end_pose);

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.005;
  double fraction = arm_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  // Execute trajectory
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  my_plan.trajectory_ = trajectory;
  arm_group_.execute(my_plan);

  crt_ee_position = end_pose.position;
}

///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Scans the environment by capturing point clouds from different perspectives.
 * 
 * This function captures point clouds from multiple perspectives by moving the robot arm to different positions.
 * It constructs a point cloud representing the entire scene by merging the point clouds obtained from each perspective.
 * 
 * @return A shared pointer to a PointCloud containing the merged point cloud of the entire scene.
 */
PointCPtr 
cw3::scanEnvironment() {
  // Define initial position
  double x = 0.31;
  double y = 0.2;
  double z = 0.88;
  
  // Define corner points of the scanning area
  geometry_msgs::Point point1;
  point1.x = x;
  point1.y = y;
  point1.z = z;

  geometry_msgs::Point point2;
  point2.x = x;
  point2.y = -y;
  point2.z = z;

  geometry_msgs::Point point3;
  point3.x = -x - 0.02;
  point3.y = -y;
  point3.z = z;

  geometry_msgs::Point point4;
  point4.x = -x - 0.02;
  point4.y = y;
  point4.z = z;

  // Store corner points in a vector
  std::vector<geometry_msgs::Point> corners(4);
  corners[0] = point1;
  corners[1] = point2;
  corners[2] = point3;
  corners[3] = point4;

  geometry_msgs::Pose target_pose;

  // Set initial point based on current end-effector position or default values
  if (crt_ee_position.x == -9999) {
    geometry_msgs::Point init_point;
    init_point.x = x;
    init_point.y = 0;
    init_point.z = z;
    // Move arm to initial position
    target_pose = moveAbovePose(init_point);
    cartesianPathPlan(target_pose);
  }

  std::cout<<"Crt EE X: "<< crt_ee_position.x <<"Crt EE Y: "<< crt_ee_position.y <<"Crt EE Z: "<< crt_ee_position.z<<std::endl; 

  // Calculate starting corner index
  int corner_index = 0;
  float min_dist = 9999;
  float dist;
  for (int i = 0; i < corners.size(); ++i) {
    dist = euclidDistance(corners[i],crt_ee_position);
    std::cout<<"========================"<<std::endl;
    std::cout<<"Distance: "<<dist<<std::endl;
    std::cout<<"========================"<<std::endl;
    if (dist < min_dist)
    {
      corner_index = i;
      min_dist = dist;
    }
  }

  std::cout<<"========================"<<std::endl;
  std::cout<<"Starting corner index: "<<corner_index<<std::endl;
  std::cout<<"========================"<<std::endl;

  PointCPtr full_scene_cloud (new PointC);

  // geometry_msgs::Point start_point = crt_ee_position;

  // Iterate through each corner point
  for (size_t i = 0; i < corners.size(); ++i) {

    target_pose = moveAbovePose(corners[corner_index]);
    moveArm(target_pose);
    // cartesianPathPlan(target_pose);

    PointCPtr world_cloud (new PointC);

    // Apply ground filter and transform point cloud to world frame
    applyGroundFilter(g_cloud_ptr, g_cloud_filtered);

    pcl_ros::transformPointCloud(BASE_FRAME_, *g_cloud_filtered, *world_cloud, listener_);

    pubFilteredPCMsg(g_pub_cloud, *world_cloud);

    // Merge point clouds
    if (i == 0) {
      *full_scene_cloud = *world_cloud;
    } else {
      *full_scene_cloud += *world_cloud;
    }

    corner_index++;
    corner_index = corner_index%corners.size();
    // start_point = crt_ee_position;
  }

  // Publish the merged point cloud of the entire scene
  pubFilteredPCMsg(full_scene_pub_cloud, *full_scene_cloud);

  return full_scene_cloud;
}

///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Performs color determination for each basket location.
 *
 * This function moves the arm to a predefined position, filters a point cloud by color,
 * calculates cluster centroids, and determines the color of each basket based on the closest centroid.
 *
 * @param basket_locs A vector containing the locations of the baskets.
 * @return A vector of strings representing the determined colors for each basket.
 */
std::string 
cw3::colorDetermination(float r, float g, float b) {

    int colorThreshold = 50; 

    int upper = 160;
    int lower = 26;

    if (std::fabs(r - upper) < colorThreshold && std::fabs(g - lower) < colorThreshold && std::fabs(b - lower) < colorThreshold) {
        return "red";
    }
    else if (std::fabs(r  - upper) < colorThreshold && std::fabs(g - lower) < colorThreshold && std::fabs(b - upper) < colorThreshold) {
        return "purple";
    }
    else if (std::fabs(r  - lower) < colorThreshold && std::fabs(g - lower) < colorThreshold && std::fabs(b - upper) < colorThreshold) {
        return "blue";
    } 
    else {
        return "none";
    }

}

///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Publishes a marker at a specified position.
 * 
 * This function publishes a marker in RViz at the specified position with the given ID.
 * 
 * @param x The x-coordinate of the marker position.
 * @param y The y-coordinate of the marker position.
 * @param z The z-coordinate of the marker position.
 * @param id The ID of the marker.
 */
void 
cw3::publishMarker(float x, float y, float z, int id) {
  // Set up the marker
  visualization_msgs::Marker marker;
  marker.header.frame_id = BASE_FRAME_;
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();

  // Publish the marker
  marker_pub.publish(marker);
}

///////////////////////////////////////////////////////////////////////////////

std::array<int64_t, 2>
cw3::t3()
{
   
  // Get the current list of collision object names
  std::vector<std::string> object_ids = planning_scene_interface_.getKnownObjectNames();

  // Remove all known objects
  if (!object_ids.empty()) {
      planning_scene_interface_.removeCollisionObjects(object_ids);
  }

  addGroundCollision();

  std::vector<geometry_msgs::Point> noughts;
  std::vector<PointCPtr> noughts_clouds;
  std::vector<float> noughts_max_distances;

  std::vector<geometry_msgs::Point> crosses;
  std::vector<PointCPtr> crosses_clouds;
  std::vector<float> crosses_max_distances;

  int obstacle_count = 0;

  geometry_msgs::Point basket_point;
  basket_point.z = 0;

  std::array<int64_t, 2> res;

  PointCPtr full_scene_cloud = scanEnvironment();

  std::vector<pcl::PointIndices> cluster_indices = dbscanClustering(full_scene_cloud);

  std::cout<<"Number of clusters: "<<cluster_indices.size()<<std::endl;

  int obs_id = 0;

  int nought_index = 0;
  int cross_index = 0;

  float closest_nought_distance = 999;
  float closest_cross_distance = 999;

  int closest_nought;
  int closest_cross;

  for (const auto& indices : cluster_indices) {

    float sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;

    unsigned long sum_r = 0, sum_g = 0, sum_b = 0;
    
    for (const auto& idx : indices.indices) {
            const auto& point = full_scene_cloud->points[idx];
            sum_x += point.x;
            sum_y += point.y;
            sum_z += point.z;
            sum_r += point.r;
            sum_g += point.g;
            sum_b += point.b;
    }

    size_t num_points = indices.indices.size();
    float avg_x = sum_x / num_points;
    float avg_y = sum_y / num_points;
    float avg_z = sum_z / num_points;

    float avg_r = static_cast<float>(sum_r / num_points);
    float avg_g = static_cast<float>(sum_g / num_points);
    float avg_b = static_cast<float>(sum_b / num_points);


    geometry_msgs::Point centroid;
    centroid.x = avg_x;
    centroid.y = avg_y;
    centroid.z = 0;

    publishMarker(avg_x,avg_y,0,obs_id);
    obs_id++;

    std::cout<<"==================================="<<std::endl;
    std::cout<<"Cluster Size: "<<num_points<<std::endl;
    // std::cout<<"Centroid X: "<<avg_x<<std::endl;
    // std::cout<<"Centroid Y: "<<avg_y<<std::endl;
    std::cout<<"Average R: "<<avg_r<<std::endl;
    std::cout<<"Average G: "<<avg_g<<std::endl;
    std::cout<<"Average B: "<<avg_b<<std::endl;

    bool cluster_not_black = (avg_r + avg_g + avg_b) > 90;

    // std::cout<<"Color: "<<cluster_color<<std::endl;

    PointCPtr cluster_cloud_ptr (new PointC);
    pcl::PointIndices::Ptr cluster_indices (new pcl::PointIndices (indices));
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(full_scene_cloud);
    extract.setIndices(cluster_indices);
    extract.filter(*cluster_cloud_ptr);

    if (cluster_not_black)
    {
      float min_dist = 9999;
      float max_dist = 0;
      float distance_from_centroid;
      geometry_msgs::Point cloud_point;
      cloud_point.z = 0;
      for (const auto& idx : indices.indices) {
              const auto& point = full_scene_cloud->points[idx];
              cloud_point.x = point.x;
              cloud_point.y = point.y;
              distance_from_centroid = euclidDistance(centroid,cloud_point);
              min_dist = std::min(min_dist,distance_from_centroid);
              max_dist = std::max(max_dist,distance_from_centroid);
      }

      std::cout<<"Min Distance: "<<min_dist<<std::endl;
      std::cout<<"Max Distance: "<<max_dist<<std::endl;

      float centroid_to_ee_distance = euclidDistance(centroid,crt_ee_position);


      if (max_dist > 0.2)
      {
        std::cout<<"This is a basket"<<std::endl;
        basket_point.x = avg_x;
        basket_point.y = avg_y;
      }
      else if (min_dist > 0.01){
        std::cout<<"This is a nought"<<std::endl;
        noughts.push_back(centroid);
        noughts_clouds.push_back(cluster_cloud_ptr);
        noughts_max_distances.push_back(max_dist);

        if (centroid_to_ee_distance < closest_nought_distance)
        {
          closest_nought_distance = centroid_to_ee_distance;
          closest_nought = nought_index;
        }

        nought_index++;
      }else{
        std::cout<<"This is a cross"<<std::endl;
        crosses.push_back(centroid);
        crosses_clouds.push_back(cluster_cloud_ptr);
        crosses_max_distances.push_back(max_dist);

        if (centroid_to_ee_distance < closest_cross_distance)
        {
          closest_cross_distance = centroid_to_ee_distance;
          closest_cross = cross_index;
        }

        cross_index++;
      }
    }else{
      // add obstacle collision
      addObstacleCollision(centroid,cluster_cloud_ptr,std::to_string(obstacle_count));

      obstacle_count++;
    }
  }

  std::cout<<"There are "<<noughts.size()<<" noughts"<<std::endl;
  std::cout<<"There are "<<crosses.size()<<" crosses"<<std::endl;

  std::cout<<"Noughts:"<<std::endl;
  for (const auto& nought : noughts) {
    std::cout<<"X: "<<nought.x<<" Y: "<<nought.y<<" Z: "<<nought.z<<std::endl;
  }

  std::cout<<"Crosses:"<<std::endl;
  for (const auto& cross : crosses) {
    std::cout<<"X: "<<cross.x<<" Y: "<<cross.y<<" Z: "<<cross.z<<std::endl;
  }


  if (noughts.size() > crosses.size())
  {
    pubFilteredPCMsg(g_pub_cloud,*noughts_clouds[closest_nought]);
    float size = determineSize("nought", noughts_max_distances[closest_nought]);
    gripper_open_ = size+0.04;
    transformGraspAndPlace(noughts[closest_nought], basket_point, "nought", noughts_clouds[closest_nought], size);

  }
  else if (noughts.size() < crosses.size())
  {
    pubFilteredPCMsg(g_pub_cloud,*crosses_clouds[closest_cross]);
    float size = determineSize("cross", crosses_max_distances[closest_cross]);
    gripper_open_ = size+0.04;
    transformGraspAndPlace(crosses[closest_cross], basket_point, "cross", crosses_clouds[closest_cross], size);

  } 
  else
  {
    if (closest_nought_distance < closest_cross_distance)
    {
      pubFilteredPCMsg(g_pub_cloud,*noughts_clouds[closest_nought]);
      float size = determineSize("nought", noughts_max_distances[closest_nought]);
      gripper_open_ = size+0.04;
      transformGraspAndPlace(noughts[closest_nought], basket_point, "nought", noughts_clouds[closest_nought], size);
      // t1(noughts[closest_nought],basket_point,"nought");
    }
    else
    {
      pubFilteredPCMsg(g_pub_cloud,*crosses_clouds[closest_cross]);
      float size = determineSize("cross", crosses_max_distances[closest_cross]);
      gripper_open_ = size+0.04;
      transformGraspAndPlace(crosses[closest_cross], basket_point, "cross", crosses_clouds[closest_cross], size);
      // t1(crosses[closest_nought],basket_point,"cross");
    }
  }

  int64_t total_num_shapes = static_cast<int64_t>(noughts.size() + crosses.size());
  int64_t num_most_common = static_cast<int64_t>(std::max(noughts.size(), crosses.size()));

  res[0] = total_num_shapes;
  res[1] = num_most_common;

  // Remove all collision settings
  removeCollision(GROUND_COLLISION_);

  removeObstacles(obstacle_count);

  return res;
}

///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Transforms the point cloud, then performs a grasp and place operation.
 * 
 * This function manipulates the point cloud data to align with the object's orientation before executing a grasp and place operation.
 * It makes a copy of the input point cloud and applies a transformation to each point. This transformation involves translating the 
 * point cloud so that the object's position becomes the new origin, and then swapping the x and y coordinates of each point. This 
 * operation is intended to align the point cloud with the robot's frame of reference or to correct for orientation differences 
 * between the object and the robot's grasp mechanism. After transforming the point cloud, the function proceeds to grasp the object
 * at its current location and place it at a specified target location, taking into account the object's shape and the size parameter 
 * provided.
 * 
 * This method allows for more accurate positioning and orientation handling during the pick and place process by ensuring the 
 * transformed point cloud is in the desired orientation relative to the robot's manipulator.
 * 
 * @param object The geometry_msgs::Point representing the object's starting position.
 * @param target The geometry_msgs::Point representing the target position for placement.
 * @param shape_type A string indicating the shape of the object ('cross' or 'nought'), which influences the grasp strategy.
 * @param input_cloud A shared pointer to the point cloud data associated with the object.
 * @param size A double value representing the size of the object, which affects grasp planning.
 */

void
cw3::transformGraspAndPlace(geometry_msgs::Point object, geometry_msgs::Point target, std::string shape_type, const PointCPtr& input_cloud, double size)
{

  PointCPtr cloud_copy(new PointC);
  *cloud_copy = *input_cloud;

  for (auto &point : *cloud_copy) {
      point.x -= object.x;
      point.y -= object.y;

      float tmp = point.x;

      point.x = point.y;
      point.y = tmp;
  }

  float angle_radians;

  // Calculate the angle
  if(shape_type == "cross"){
    angle_radians = computeOptimalAngle(cloud_copy, 2.12*size, size, "cross") - M_PI/4; 
    
    std::cout << "Cross, the angle is: " << angle_radians/M_PI*360 << std::endl;
  }else{
    angle_radians = computeOptimalAngle(cloud_copy, 3.5*size, 0.5*size, "nought") - M_PI/4;

    std::cout << "Nought, the angle is: " << angle_radians/M_PI*360 << std::endl;
    std::cout << "Original angle radians: "<<angle_radians<<std::endl;

  }

  // Avoid obtuse angles, make angles acute
  if (angle_radians > M_PI / 2) angle_radians -= M_PI;
  else if (angle_radians < -M_PI / 2) angle_radians += M_PI;

  // Restriction angle between -pi/4 and pi/4
  if (angle_radians > M_PI / 4) angle_radians -= M_PI/2;
  else if (angle_radians < -M_PI / 4) angle_radians += M_PI/2;

  std::cout<<"Final angle: "<<angle_radians<<std::endl;

  // Positional offsets for pick and place
  if(shape_type == "cross"){
    object.x += (0.5*size+0.02) * cos(angle_radians) ;
    object.y -= (0.5*size+0.02) * sin(angle_radians) ;
    target.x += size;
    
  }else{
    object.x += 2 * size * sin(angle_radians) ;
    object.y += 2 * size * cos(angle_radians) ;
    target.y += 2 * size;
  }

  geometry_msgs::Pose target_pose = moveAbove(object, angle_radians);
  target_pose.position.z = 0.5; 

  addGroundCollision(0.2f); // higher ground collision requirement
  moveArm(target_pose);
  removeCollision(GROUND_COLLISION_);
  addGroundCollision();

  pick(object, target, angle_radians);
}

///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Adds a collision object to the MoveIt planning scene and RViz.
 * 
 * This function creates a collision object message and adds it to the MoveIt planning scene and RViz for collision detection.
 * 
 * @param object_name The name of the collision object.
 * @param centre The centre point of the collision object.
 * @param dimensions The dimensions of the collision object.
 * @param orientation The orientation of the collision object.
 */
void 
cw3::addCollision(std::string object_name, geometry_msgs::Point centre, geometry_msgs::Vector3 dimensions, geometry_msgs::Quaternion orientation) {
  // Create a collision object message and a vector of these messages
  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;

  // Set header information
  collision_object.id = object_name;
  collision_object.header.frame_id = BASE_FRAME_;

  // Define the primitive type and its dimensions
  collision_object.primitives.resize(1);
  collision_object.primitives[0].type = collision_object.primitives[0].BOX;
  collision_object.primitives[0].dimensions.resize(3);
  collision_object.primitives[0].dimensions[0] = dimensions.x;
  collision_object.primitives[0].dimensions[1] = dimensions.y;
  collision_object.primitives[0].dimensions[2] = dimensions.z;

  // Define the pose of the collision object
  collision_object.primitive_poses.resize(1);
  collision_object.primitive_poses[0].position.x = centre.x;
  collision_object.primitive_poses[0].position.y = centre.y;
  collision_object.primitive_poses[0].position.z = centre.z;
  collision_object.primitive_poses[0].orientation = orientation;

  // Specify that we are adding this collision object
  collision_object.operation = collision_object.ADD;

  // Add the collision object to the vector and apply to the planning scene
  object_vector.push_back(collision_object);
  planning_scene_interface_.applyCollisionObjects(object_vector);

  return;
}

///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Adds a cylindrical collision object to the planning scene for the robot.
 *
 * This function creates and configures a cylindrical collision object based on specified parameters,
 * then adds this object to the robot's planning scene. The function sets up the collision object with
 * an identifier, frame of reference, and the dimensions (height and radius) of the cylinder.
 * It defines the position of the cylinder using the provided centre coordinates. Once configured,
 * the cylinder is added to a vector of collision objects, which is then applied to the planning scene
 * to update the environment the robot interacts with. This helps in avoiding collisions during robot movements.
 *
 * @param object_name The name used to identify the collision object.
 * @param centre The geometric centre of the cylinder, specifying its position in the space.
 * @param height The height of the cylinder.
 * @param radius The radius of the cylinder.
 */

void 
cw3::addCylinderCollision(std::string object_name, geometry_msgs::Point centre, float height, float radius) {
  // Create a collision object message and a vector of these messages
  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;

  // Set header information
  collision_object.id = object_name;
  collision_object.header.frame_id = BASE_FRAME_;

  // Define the primitive type and its dimensions as cylinder
  collision_object.primitives.resize(1);
  collision_object.primitives[0].type = collision_object.primitives[0].CYLINDER;
  collision_object.primitives[0].dimensions.resize(2); // Cylinder only has height and radius
  collision_object.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = height;
  collision_object.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = radius;

  // Define the pose of the collision object
  collision_object.primitive_poses.resize(1);
  collision_object.primitive_poses[0].position.x = centre.x;
  collision_object.primitive_poses[0].position.y = centre.y;
  collision_object.primitive_poses[0].position.z = centre.z;

  // Specify that we are adding this collision object
  collision_object.operation = collision_object.ADD;

  object_vector.push_back(collision_object);
  planning_scene_interface_.applyCollisionObjects(object_vector);

  return;
}

///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Removes a collision object from the MoveIt planning scene and RViz.
 * 
 * This function creates a collision object message to remove a specified collision object from the MoveIt planning scene and RViz.
 * 
 * @param object_name The name of the collision object to be removed.
 */
void 
cw3::removeCollision(std::string object_name) {
  // Create a collision object message and a vector of these messages
  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;
  
  // Input the name of the object and specify that we want it removed
  collision_object.id = object_name;
  collision_object.operation = collision_object.REMOVE;

  // Apply the collision object removal to the scene
  object_vector.push_back(collision_object);
  planning_scene_interface_.applyCollisionObjects(object_vector);
}

///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Removes obstacles from the MoveIt planning scene and RViz.
 * 
 * This function iterates through a specified number of obstacles and removes them from the MoveIt planning scene and RViz.
 * 
 * @param obstacle_count The number of obstacles to be removed.
 */
void 
cw3::removeObstacles(int obstacle_count) {
  // Iterate through the specified number of obstacles
  for (int i = 0; i < obstacle_count; i++) {
    // Remove collision object with name corresponding to the current iteration index
    removeCollision(std::to_string(i));
  }
}

///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Adds a ground collision object to the MoveIt planning scene and RViz.
 * 
 * This function creates and adds a ground collision object to the MoveIt planning scene and RViz for collision detection.
 * 
 * @param ground_height The height of the ground collision object.
 */
void 
cw3::addGroundCollision(float ground_height) {
  // Define dimensions of the ground collision object
  geometry_msgs::Vector3 ground_dimension;
  ground_dimension.x = 1.4;
  ground_dimension.y = 1.1;
  ground_dimension.z = ground_height;

  // Define position of the ground collision object
  geometry_msgs::Point ground_position;
  ground_position.x = 0;
  ground_position.y = 0;
  ground_position.z = 0.01; // Slightly above the actual ground to prevent collision with objects on the ground

  // Define orientation of the ground collision object
  geometry_msgs::Quaternion ground_orientation;
  ground_orientation.w = 1;
  ground_orientation.x = 0;
  ground_orientation.y = 0;
  ground_orientation.z = 0;

  // Add the ground collision object
  addCollision(GROUND_COLLISION_, ground_position, ground_dimension, ground_orientation);
}

///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Adds a collision object representing an obstacle into the planning scene.
 * 
 * This function creates a collision object with a specified name and dimensions, positioned at a given centroid point in the
 * environment. The collision object is used by the motion planning framework to avoid collisions during robotic arm movements.
 * The obstacle is modeled as a box with predefined dimensions, and its orientation is set to a neutral quaternion (no rotation).
 * This setup is commonly used to simulate obstacles in the environment that the robot must navigate around.
 * 
 * The dimensions and position of the obstacle can be adjusted based on the specific requirements of the environment or the task.
 * This function abstracts the details of collision object creation, making it easier to programmatically add obstacles to the
 * planning scene for simulation or real-world applications.
 * 
 * @param obstacle_centroid The geometry_msgs::Point representing the central position of the obstacle in the environment.
 * @param obj_name A string that uniquely identifies the collision object (obstacle) being added.
 */

void
cw3::addObstacleCollision(geometry_msgs::Point obstacle_centroid,PointCPtr &output_cloud,std::string obj_name)
{

  pcl::PointXYZRGBA min_pt, max_pt;
  pcl::getMinMax3D(*output_cloud, min_pt, max_pt);
  float length = max_pt.x - min_pt.x;
  float width = max_pt.y - min_pt.y;
  float height = max_pt.z - min_pt.z;

  obstacle_centroid.z = (height/2) + 0.05;

  float radius = std::sqrt(width*width + length*length) /2;

  addCylinderCollision(obj_name,obstacle_centroid,height,radius);
}

void 
cw3::applyBlackFilter(PointCPtr &input_cloud, PointCPtr &output_cloud) {
    pcl::ConditionalRemoval<PointT> color_filter;
    color_filter.setInputCloud(input_cloud);
    
    pcl::PackedRGBComparison<PointT>::Ptr red_condition(new pcl::PackedRGBComparison<PointT>("r",pcl::ComparisonOps::GT,10));
    pcl::PackedRGBComparison<PointT>::Ptr green_condition(new pcl::PackedRGBComparison<PointT>("g",pcl::ComparisonOps::GT,10));
    pcl::PackedRGBComparison<PointT>::Ptr blue_condition(new pcl::PackedRGBComparison<PointT>("b",pcl::ComparisonOps::GT,10));
    pcl::ConditionAnd<PointT>::Ptr color_cond (new pcl::ConditionAnd<PointT>());
    color_cond->addComparison(red_condition);
    color_cond->addComparison(green_condition);
    color_cond->addComparison(blue_condition);
    color_filter.setCondition(color_cond);
    color_filter.filter(*output_cloud);
}

///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Applies a ground filter to a point cloud.
 * 
 * This function filters the input point cloud to remove points that meet a specified condition.
 * 
 * @param input_cloud The input point cloud to be filtered.
 * @param output_cloud The output point cloud containing the filtered points.
 */
 
void 
cw3::applyGroundFilter(PointCPtr &input_cloud, PointCPtr &output_cloud) {
    // Create a ConditionalRemoval object for filtering
    pcl::ConditionalRemoval<PointT> color_filter;
    color_filter.setInputCloud(input_cloud);

    // Define a condition based on RGB color
    pcl::PackedRGBComparison<PointT>::Ptr green_condition(new pcl::PackedRGBComparison<PointT>("g", pcl::ComparisonOps::LT, 54));
    pcl::ConditionAnd<PointT>::Ptr color_cond (new pcl::ConditionAnd<PointT>());
    color_cond->addComparison(green_condition);
    color_filter.setCondition(color_cond);

    // Apply the filter to the input cloud and store the result in the output cloud
    color_filter.filter(*output_cloud);
}

///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Applies a pass-through filter to a point cloud along a specified axis.
 * 
 * This function filters the input point cloud to retain points within a specified range along the given axis.
 * 
 * @param in_cloud_ptr The input point cloud to be filtered.
 * @param out_cloud_ptr The output point cloud containing the filtered points.
 * @param axis The axis along which the pass-through filter is applied (e.g., "x", "y", "z").
 * @param threshold The threshold value defining the filtering range along the specified axis.
 */
void 
cw3::applyPassthrough(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr, std::string axis, float threshold_upper,float threshold_lower) {
  // Create a new point cloud pointer for the filtered cloud
  PointCPtr cloud_filtered(new PointC);

  // Apply a pass-through filter along the specified axis
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(in_cloud_ptr); // Set the input point cloud
  pass.setFilterFieldName(axis); // Set the filtering field to the specified axis
  pass.setFilterLimits(threshold_lower, threshold_upper); // Set the filtering range
  pass.filter(*out_cloud_ptr); // Execute the filtering
}

///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Applies voxel grid downsampling to reduce the density of a point cloud.
 * 
 * This function reduces the density of an input point cloud using a voxel grid filter, which partitions the space into a grid of voxels 
 * and then condenses the points within each voxel into a single representative point. The size of the voxels is determined by the 
 * `g_vg_leaf_sz` variable, which defines the length of the sides of the cubic voxel. This downsampling technique is commonly used to 
 * decrease the number of points in a point cloud, thereby reducing computational requirements for subsequent processing steps while 
 * preserving the overall shape and structure of the data.
 * 
 * The voxel grid filter is a critical pre-processing step in many point cloud processing pipelines, particularly in tasks involving 
 * 3D reconstruction, object detection, and robotic perception, where managing the trade-off between data resolution and computational 
 * efficiency is essential.
 * 
 * @param in_cloud_ptr A shared pointer to the input point cloud that will be downsampled.
 * @param out_cloud_ptr A shared pointer to the output point cloud where the downsampled results will be stored.
 */


void
cw3::applyVX (PointCPtr &in_cloud_ptr,
                      PointCPtr &out_cloud_ptr)
{
  g_vx.setInputCloud (in_cloud_ptr);
  g_vx.setLeafSize (g_vg_leaf_sz, g_vg_leaf_sz, g_vg_leaf_sz);
  g_vx.filter (*out_cloud_ptr);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Filters points in a point cloud based on their x-coordinate values.
 * 
 * This function selectively removes points from an input point cloud based on the x-coordinate value of each point. It uses a pass-through 
 * filter configured to retain only the points whose x-coordinates fall within a specified range, defined by `g_pt_thrs_min` and 
 * `g_pt_thrs_max`. Points outside this range are excluded from the output point cloud. The pass-through filter is a common technique used 
 * to crop a point cloud to a region of interest, effectively reducing the processing load and focusing analysis on specific areas.
 * 
 * This filtering process is particularly useful in applications like robotic perception and environment scanning, where points outside a 
 * certain spatial region are irrelevant to the task at hand and can be ignored to improve efficiency and focus.
 * 
 * @param in_cloud_ptr A shared pointer to the input point cloud to be filtered.
 * @param out_cloud_ptr A shared pointer to the output point cloud containing only the points within the specified x-coordinate range.
 */

void
cw3::applyPT (PointCPtr &in_cloud_ptr,
                      PointCPtr &out_cloud_ptr)
{
  g_pt.setInputCloud (in_cloud_ptr);
  g_pt.setFilterFieldName ("x");
  g_pt.setFilterLimits (g_pt_thrs_min, g_pt_thrs_max);
  g_pt.filter (*out_cloud_ptr);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Estimates the normal vectors for each point in a point cloud.
 * 
 * This function calculates the normal vectors for all points in an input point cloud. The normals are estimated using a k-nearest 
 * neighbors approach, where the normal at each point is computed based on the orientation of its nearest neighbors. The number of 
 * neighbors considered for each point is specified by `g_k_nn`. The normals are crucial for various 3D processing tasks, including 
 * surface reconstruction, object segmentation, and feature extraction, as they provide information about the geometry and orientation 
 * of surfaces within the point cloud.
 * 
 * The function uses a kd-tree search method for efficient nearest neighbor search, making the normal estimation process faster and 
 * suitable for real-time applications. The estimated normals are stored in `g_cloud_normals`, making them accessible for subsequent 
 * processing steps that rely on normal information.
 * 
 * Estimating normals is an essential preprocessing step in many point cloud processing pipelines, particularly those involving 
 * the analysis of surface characteristics and geometry.
 */


void
cw3::findNormals (PointCPtr &in_cloud_ptr)
{
  // Estimate point normals
  g_ne.setInputCloud (in_cloud_ptr);
  g_ne.setSearchMethod (g_tree_ptr);
  g_ne.setKSearch (g_k_nn);
  g_ne.compute (*g_cloud_normals);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Segments the largest planar component from the input point cloud.
 * 
 * This function performs plane segmentation on an input point cloud to identify and extract the largest planar component within it. 
 * It uses the RANSAC algorithm alongside normal information to robustly fit a plane model to the point cloud data. Parameters such 
 * as the distance threshold for including points in the plane, the maximum number of iterations, and the normal distance weight are 
 * configured to optimize the segmentation process.
 * 
 * After identifying the plane, the function extracts both the planar inliers (points belonging to the plane) and the remaining points 
 * (non-planar component). The extracted planar inliers are stored in a separate point cloud, which can be used for further analysis 
 * or visualization. Simultaneously, the non-planar component is also retained for additional processing steps. This segmentation 
 * is a crucial step in many point cloud processing pipelines, enabling tasks such as object detection on surfaces, ground removal, 
 * and environment modeling.
 * 
 * Additionally, the function stores the coefficients of the plane model, which can be used to understand the orientation and position 
 * of the plane within the 3D space.
 */

void
cw3::segPlane (PointCPtr &in_cloud_ptr)
{
  // Create the segmentation object for the planar model
  // and set all the params
  g_seg.setOptimizeCoefficients (true);
  g_seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  g_seg.setNormalDistanceWeight (0.1); //bad style
  g_seg.setMethodType (pcl::SAC_RANSAC);
  g_seg.setMaxIterations (100); //bad style
  g_seg.setDistanceThreshold (0.03); //bad style
  g_seg.setInputCloud (in_cloud_ptr);
  g_seg.setInputNormals (g_cloud_normals);
  // Obtain the plane inliers and coefficients
  g_seg.segment (*g_inliers_plane, *g_coeff_plane);
  
  // Extract the planar inliers from the input cloud
  g_extract_pc.setInputCloud (in_cloud_ptr);
  g_extract_pc.setIndices (g_inliers_plane);
  g_extract_pc.setNegative (false);
  
  // Write the planar inliers to disk
  g_extract_pc.filter (*g_cloud_plane);
  
  // Remove the planar inliers, extract the rest
  g_extract_pc.setNegative (true);
  g_extract_pc.filter (*g_cloud_filtered2);
  g_extract_normals.setNegative (true);
  g_extract_normals.setInputCloud (g_cloud_normals);
  g_extract_normals.setIndices (g_inliers_plane);
  g_extract_normals.filter (*g_cloud_normals2);

  //ROS_INFO_STREAM ("Plane coefficients: " << *g_coeff_plane);
  // ROS_INFO_STREAM ("PointCloud representing the planar component: "
  //                  << g_cloud_plane->size ()
  //                  << " data points.");
}
    
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Segments cylindrical components from the filtered point cloud.
 * 
 * This function is dedicated to identifying and extracting cylindrical components within a pre-filtered point cloud. Utilizing the 
 * RANSAC algorithm in combination with normal information from the point cloud, it fits a cylinder model to the data. The segmentation 
 * process involves setting specific parameters such as the normal distance weight, the maximum number of iterations for the RANSAC 
 * algorithm, a distance threshold that points must be within to fit the model, and limits on the radius of the cylinder to be identified.
 * 
 * Once a cylindrical component is identified, its inliers (points that are part of the cylinder) are extracted and stored in a separate 
 * point cloud. This allows for further analysis or visualization of the cylindrical component independently from the rest of the data. 
 * Cylinder segmentation is a key step in processing point clouds for applications involving robotic manipulation, industrial inspection, 
 * or environmental modeling where cylindrical objects are of interest.
 * 
 * The parameters used for segmentation can be fine-tuned based on the specific requirements of the application or the characteristics 
 * of the point cloud data being processed.
 */

void
cw3::segCylind (PointCPtr &in_cloud_ptr)
{
  // Create the segmentation object for cylinder segmentation
  // and set all the parameters
  g_seg.setOptimizeCoefficients (true);
  g_seg.setModelType (pcl::SACMODEL_CYLINDER);
  g_seg.setMethodType (pcl::SAC_RANSAC);
  g_seg.setNormalDistanceWeight (0.1); //bad style
  g_seg.setMaxIterations (10000); //bad style
  g_seg.setDistanceThreshold (0.05); //bad style
  g_seg.setRadiusLimits (0, 0.1); //bad style
  g_seg.setInputCloud (g_cloud_filtered2);
  g_seg.setInputNormals (g_cloud_normals2);

  // Obtain the cylinder inliers and coefficients
  g_seg.segment (*g_inliers_cylinder, *g_coeff_cylinder);
  
  // Write the cylinder inliers to disk
  g_extract_pc.setInputCloud (g_cloud_filtered2);
  g_extract_pc.setIndices (g_inliers_cylinder);
  g_extract_pc.setNegative (false);
  g_extract_pc.filter (*g_cloud_cylinder);
  
  // ROS_INFO_STREAM ("PointCloud representing the cylinder component: "
  //                  << g_cloud_cylinder->size ()
  //                  << " data points.");
  
  return;
}

////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Performs DBSCAN clustering on a given point cloud.
 *
 * Modifies the Z coordinates of all points in the cloud to 0, effectively flattening the cloud
 * for 2D clustering. It uses a KdTree for efficient neighbor search in the clustering process.
 * The DBSCAN clustering parameters cluster tolerance, minimum cluster size, and maximum cluster
 * size are set before extracting the clusters.
 *
 * @param cloud A shared pointer to the input point cloud (pcl::PointCloud<pcl::PointXYZRGB>).
 * This cloud is modified in-place by setting the Z coordinates of all points to 0.
 *
 * @return A vector of pcl::PointIndices, where each pcl::PointIndices object contains the indices
 * of the points belonging to a cluster. The size of the vector indicates the number of clusters found.
 *
 * @note The function modifies the input cloud by flattening it to Z=0 for all points.
 */
std::vector<pcl::PointIndices> 
cw3::dbscanClustering(PointCPtr &cloud) {
    
    PointCPtr cloud_copy(new PointC);
    *cloud_copy = *cloud;

    // Set the Z coordinates of all points to 0
    for (auto &point : *cloud_copy) {
        point.z = 0.0;
    }

    // Creating a KdTree object for finding neighbors
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    tree->setInputCloud(cloud_copy);

    // Create DBSCAN clustering object and set parameters
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.007); // Search radius
    ec.setMinClusterSize(100);   // Minimum Cluster Size
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_copy);
    ec.extract(cluster_indices);

 
    return cluster_indices;
}


