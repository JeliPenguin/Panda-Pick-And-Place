/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw3_team_<your_team_number> package */

#include <cw3_class.h> // change to your team name here!

///////////////////////////////////////////////////////////////////////////////

cw3::cw3(ros::NodeHandle nh)
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
  g_pub_pose = nh_.advertise<geometry_msgs::PointStamped> ("cyld_pt", 1, true);
  
  // Create a ROS subscriber for the input point cloud
  // ros::Subscriber color_cloud_ = nh.subscribe<sensor_msgs::PointCloud2>("/r200/camera/depth_registered/points", 1, 
      // boost::bind(&cw3::cloudCallback, this, _1));

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

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw3::t3_callback(cw3_world_spawner::Task3Service::Request &request,
  cw3_world_spawner::Task3Service::Response &response)
{
  /* function which should solve task 3 */

  ROS_INFO("The coursework solving callback for task 3 has been triggered");

  return true;
}

///////////////////////////////////////////////////////////////////////////////

void
cw3::cloudCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& msg) {
  // ROS_INFO("PointCloud2 message received");
  // convert from ros' sensor_msg/PointCloud2 to pcl/PointCloud2
  pcl::PCLPointCloud2 pcl_cloud;
  pcl_conversions::toPCL(*msg, pcl_cloud);

  // convert pcl/PointCloud2 to PointCloud vector of PointXYZRGB
  temp_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>); 
  pcl::fromPCLPointCloud2(pcl_cloud, *temp_cloud_);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp_cloud_rgba(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::copyPointCloud(*temp_cloud_, *temp_cloud_rgba); // Convert from XYZRGB to XYZRGBA

  pubFilteredPCMsg(g_pub_cloud, *temp_cloud_rgba);

}

void
cw3::pubFilteredPCMsg (ros::Publisher &pc_pub,
                               PointC &pc)
{
  // Publish the data
  sensor_msgs::PointCloud2 g_cloud_filtered_msg;
  pcl::toROSMsg(pc, g_cloud_filtered_msg);
  pc_pub.publish (g_cloud_filtered_msg);
  
  return;
}


geometry_msgs::PointStamped
cw3::frameTransform(geometry_msgs::Point from_p, std::string from_frame, std::string to_frame){
  // Temporary storage for transformation.
  geometry_msgs::PointStamped from_p_stamped;
  geometry_msgs::PointStamped to_p;

  // Prepare for coordinate frame transformation.
  from_p_stamped.header.frame_id = from_frame;
  from_p_stamped.point.x = from_p.x;
  from_p_stamped.point.y = from_p.y;
  from_p_stamped.point.z = from_p.z;

  try {
      // Transform centroid to "panda_link0" frame.
      listener_.transformPoint(to_frame, from_p_stamped, to_p);
  } catch (tf::TransformException& ex) {
      ROS_ERROR("Received a transformation exception: %s", ex.what());
  }

  return to_p;
}


///////////////////////////////////////////////////////////////////////////////
// Task 1
///////////////////////////////////////////////////////////////////////////////

geometry_msgs::Pose
cw3::moveAbove(geometry_msgs::Point point, float angle){
  /* This function produces a "gripper facing down" pose given a xyz point */

  // Position gripper above point
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);
  // Gripper Orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_+angle);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion orientation = tf2::toMsg(q_result);

  // set the desired Pose
  geometry_msgs::Pose pose;
  pose.position = point;
  pose.orientation = orientation;

  return pose;
}

///////////////////////////////////////////////////////////////////////////////

bool 
cw3::moveArm(geometry_msgs::Pose target_pose)
{
  // setup the target pose
  ROS_INFO("Setting pose target");
  arm_group_.setPoseTarget(target_pose);

  // create a movement plan for the arm
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // google 'c++ conditional operator' to understand this line
  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // execute the planned path
  arm_group_.move();

  return success;
}

///////////////////////////////////////////////////////////////////////////////

geometry_msgs::Pose
cw3::moveAbovePose(geometry_msgs::Point point){
  // Create a quaternion representing a 180-degree rotation along the X-axis for "gripper facing down" orientation.
  tf2::Quaternion q_x180deg(-1, 0, 0, 0); // Gripper Position

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

bool 
cw3::moveGripper(float width)
{
  // safety checks in case width exceeds safe values
  if (width > gripper_open_) 
    width = gripper_open_;
  if (width < gripper_closed_) 
    width = gripper_closed_;

  // calculate the joint targets as half each of the requested distance
  double eachJoint = width / 2.0;

  // create a vector to hold the joint target for each joint
  std::vector<double> gripperJointTargets(2);
  gripperJointTargets[0] = eachJoint;
  gripperJointTargets[1] = eachJoint;

  // apply the joint target
  hand_group_.setJointValueTarget(gripperJointTargets);

  // move the robot hand
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (hand_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // move the gripper joints
  hand_group_.move();

  return success;
}

///////////////////////////////////////////////////////////////////////////////

// Filter function with inputs as references to point cloud smart pointers
pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
cw3::filterPointCloudByColor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud) {
    pcl::ConditionalRemoval<pcl::PointXYZRGB> color_filter;
    color_filter.setInputCloud(input_cloud);

    pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr green_condition(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g",pcl::ComparisonOps::LT,54));
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB>());
    color_cond->addComparison(green_condition);
    color_filter.setCondition(color_cond);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    color_filter.filter(*cloud_filtered);

    

    return cloud_filtered;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw3::pick(geometry_msgs::Point object, 
                  geometry_msgs::Point Goal,
                  float angle)
{

  geometry_msgs::Pose grasp_pose = moveAbove(object, angle);
  grasp_pose.position.z += 0.14; 
  geometry_msgs::Pose offset_pose = grasp_pose;
  offset_pose.position.z += 0.125;
  geometry_msgs::Pose release_pose = moveAbovePose(Goal);
  release_pose.position.z += 0.35; 


  // Close
  moveArm(offset_pose);
  moveGripper(gripper_open_);
  moveArm(grasp_pose);
  moveGripper(gripper_closed_);
  
  // Takeaway
  moveArm(offset_pose);
  offset_pose.position.z += 0.125;
  moveArm(offset_pose);
  
  // Place
  moveArm(release_pose);
  release_pose.position.z -= 0.15;
  moveArm(release_pose);

  // Open gripper
  moveGripper(gripper_open_);
  
  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool 
cw3::t1(geometry_msgs::Point object, 
            geometry_msgs::Point target, 
            std::string shape_type, 
            double size)
{
  addGroundCollision();

  geometry_msgs::Pose target_pose = moveAbovePose(object);
  target_pose.position.z = 0.7; 
  target_pose.position.x -= 0.03; 
  moveArm(target_pose);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud = filterPointCloudByColor(temp_cloud_);

  int num_points_before = temp_cloud_->size();

  int num_points = filtered_cloud->size();

  std::cout << "Before: "<< num_points_before << std::endl;
  std::cout << "After: " << num_points << std::endl;
  
  // 计算质心
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*filtered_cloud, centroid);

  // 应用PCA
  pcl::PCA<pcl::PointXYZRGB> pca;
  pca.setInputCloud(filtered_cloud);
  Eigen::Vector3f eigenvalues = pca.getEigenValues();
  Eigen::Matrix3f eigenvectors = pca.getEigenVectors();

  // 主轴方向是最大特征值对应的特征向量
  Eigen::Vector3f principal_axis = eigenvectors.col(0);
  
  // 计算夹角
  Eigen::Vector3f global_y_axis(0, 1, 0);
  float cosine_of_angle = principal_axis.dot(global_y_axis) / (principal_axis.norm() * global_y_axis.norm());
  float angle_radians = acos(cosine_of_angle);
  
  // 避免钝角，使角度为锐角
  if (angle_radians > M_PI / 2) angle_radians -= M_PI;
  else if (angle_radians < -M_PI / 2) angle_radians += M_PI;

  // 限制角度在-π/4到π/4之间
  if (angle_radians > M_PI / 4) angle_radians -= M_PI/2;
  else if (angle_radians < -M_PI / 4) angle_radians += M_PI/2;
  
  // Positional offsets for pick and place
  if(shape_type == "cross"){
    object.x += size * cos(angle_radians);
    object.y += size * sin(angle_radians);
    target.x += size;
    
  }else{
    object.x += 2 * size * sin(angle_radians);
    object.y += 2 * size * cos(angle_radians);
    target.y += 2 * size;

  }
  
  pick(object,target, -angle_radians);
  
  return true;
}


void
cw3::addCollision(std::string object_name,
  geometry_msgs::Point centre, geometry_msgs::Vector3 dimensions,
  geometry_msgs::Quaternion orientation)
{
  /* add a collision object in RViz and the MoveIt planning scene */

  // create a collision object message, and a vector of these messages
  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;
  
  // input header information
  collision_object.id = object_name;
  collision_object.header.frame_id = base_frame_;

  // define the primitive and its dimensions
  collision_object.primitives.resize(1);
  collision_object.primitives[0].type = collision_object.primitives[0].BOX;
  collision_object.primitives[0].dimensions.resize(3);
  collision_object.primitives[0].dimensions[0] = dimensions.x;
  collision_object.primitives[0].dimensions[1] = dimensions.y;
  collision_object.primitives[0].dimensions[2] = dimensions.z;

  // define the pose of the collision object
  collision_object.primitive_poses.resize(1);
  collision_object.primitive_poses[0].position.x = centre.x;
  collision_object.primitive_poses[0].position.y = centre.y;
  collision_object.primitive_poses[0].position.z = centre.z;
  collision_object.primitive_poses[0].orientation = orientation;

  // define that we will be adding this collision object 
  // hint: what about collision_object.REMOVE?
  collision_object.operation = collision_object.ADD;

  // add the collision object to the vector, then apply to planning scene
  object_vector.push_back(collision_object);
  planning_scene_interface_.applyCollisionObjects(object_vector);

  return;
}

void
cw3::removeCollision(std::string object_name)
{
  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;
  
  // input the name and specify we want it removed
  collision_object.id = object_name;
  collision_object.operation = collision_object.REMOVE;

  // apply this collision object removal to the scene
  object_vector.push_back(collision_object);
  planning_scene_interface_.applyCollisionObjects(object_vector);
}

void
cw3::addGroundCollision()
{
  geometry_msgs::Vector3 ground_dimension;
  ground_dimension.x = 5;
  ground_dimension.y = 5;
  ground_dimension.z = 0.02;

  geometry_msgs::Point ground_position;
  ground_position.x = 0;
  ground_position.y = 0;
  ground_position.z = 0.01;

  geometry_msgs::Quaternion ground_orientation;
  ground_orientation.w = 1;
  ground_orientation.x = 0;
  ground_orientation.y = 0;
  ground_orientation.z = 0;

  addCollision("ground",ground_position,ground_dimension,ground_orientation);
}