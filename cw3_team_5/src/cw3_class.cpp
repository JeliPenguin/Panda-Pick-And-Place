/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw3_team_<your_team_number> package */

#include <cw3_class.h> // change to your team name here!

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

void
cw3::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
  // ROS_INFO("PointCloud2 message received");
  // convert from ros' sensor_msg/PointCloud2 to pcl/PointCloud2
  pcl::PCLPointCloud2 pcl_cloud;
  pcl_conversions::toPCL(*msg, pcl_cloud);

  // convert pcl/PointCloud2 to PointCloud vector of PointXYZRGB
  // rgb_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>); 
  // pcl::fromPCLPointCloud2(pcl_cloud, *rgb_cloud_);

  // g_cloud_ptr.reset(new PointC);
  pcl::fromPCLPointCloud2 (pcl_cloud, *g_cloud_ptr);

  // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp_cloud_rgba(new pcl::PointCloud<pcl::PointXYZRGBA>);
  // pcl::copyPointCloud(*cloud_filtered, *temp_cloud_rgba); // Convert from XYZRGB to XYZRGBA
  // pubFilteredPCMsg(g_pub_cloud, *temp_cloud_rgba);

}

void
cw3::pubFilteredPCMsg (ros::Publisher &pc_pub,
                               PointC &pc)
{
  // Publish the data
  sensor_msgs::PointCloud2 g_cloud_msg;
  pcl::toROSMsg(pc, g_cloud_msg);
  pc_pub.publish (g_cloud_msg);
  
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
  tf2::Quaternion q_x180deg(1, 0, 0, 0);
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

  if (success)
  {
    crt_ee_position = target_pose.position;
  }
  // execute the planned path
  arm_group_.move();

  return success;
}

///////////////////////////////////////////////////////////////////////////////

geometry_msgs::Pose
cw3::moveAbovePose(geometry_msgs::Point point){
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
  release_pose.position.z += 0.5; 


  // Close
  moveArm(offset_pose);
  moveGripper(gripper_open_);
  moveArm(grasp_pose);
  moveGripper(gripper_closed_);
  

  // Takeaway
  offset_pose.position.z += 0.125;
  moveArm(offset_pose);
  
  // Place
  moveArm(release_pose);
  release_pose.position.z -= 0.2;
  moveArm(release_pose);

  // Open gripper
  moveGripper(gripper_open_);
  
  return true;
}

///////////////////////////////////////////////////////////////////////////////

float cw3::computeOptimalAngle(const PointCPtr& input_cloud, double length, float radius, std::string type) {
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(input_cloud);

    std::vector<float> z_values;
    float z;
    z_values.reserve(input_cloud->points.size());

    // 收集所有点的z坐标
    for (const auto& point : input_cloud->points) {
        z_values.push_back(point.z);
    }

    // 对z坐标排序
    std::sort(z_values.begin(), z_values.end());

    size_t n = z_values.size();

    // 计算最接近 1/10*n 的点的索引（向下取整）
    size_t index = (1 * n / 10); // 计算 1/10 的位置，直接使用整数除法实现向下取整

    // 由于索引是从0开始的，所以这里不需要减1
    z = z_values[index];

    float optimal_angle = 0.0f;

    if (type == "cross"){
      std::cout << "??????????????????????????????????????????????????????????????????????????????" << std::endl;
      int min_count = 1000;

      for (int alpha = -45; alpha <= 45; alpha += 1) {
          float rad = alpha * M_PI / 180.0;
          PointT searchPoint; // 使用PointT，即pcl::PointXYZRGBA
          searchPoint.x = length * std::cos(rad);
          searchPoint.y = length * std::sin(rad);
          searchPoint.z = z; // 使用pose提供的z值

          std::vector<int> pointIdxRadiusSearch;
          std::vector<float> pointRadiusSquaredDistance;
          int count = kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

          std::cout << count << std::endl;
          //std::cout << z_values[0] << "..." << z_values[n-1] << std::endl;
          if (count < min_count) {
              min_count = count;
              optimal_angle = rad;
          }
        }
    }
    else{
      int max_count = 0;

      for (int alpha = -45; alpha <= 45; alpha += 1) {
          float rad = alpha * M_PI / 180.0;
          PointT searchPoint; // 使用PointT，即pcl::PointXYZRGBA
          searchPoint.x = length * std::cos(rad);
          searchPoint.y = length * std::sin(rad);
          searchPoint.z = z; // 使用pose提供的z值

          std::vector<int> pointIdxRadiusSearch;
          std::vector<float> pointRadiusSquaredDistance;
          int count = kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

          std::cout << count << std::endl;
          //std::cout << z_values[0] << "..." << z_values[n-1] << std::endl;
          if (count > max_count) {
              max_count = count;
              optimal_angle = rad;
          }
        }
    }


    return optimal_angle;
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
  target_pose.position.z = 0.5; 
  target_pose.position.x += camera_offset_; 
  moveArm(target_pose);

  // applyVX(g_cloud_ptr, g_cloud_filtered);
  // findNormals(g_cloud_filtered);
  // segPlane(g_cloud_filtered);    
  // g_cloud_filtered_color = applyGroundFilter(g_cloud_filtered2);

  applyGroundFilter(g_cloud_ptr, g_cloud_filtered);

  pubFilteredPCMsg(g_pub_cloud, *g_cloud_filtered);
  
  // // 计算质心
  // Eigen::Vector4f centroid;
  // pcl::compute3DCentroid(*g_cloud_filtered, centroid);

  /*
  // Applied PCA
  pcl::PCA<PointT> pca;
  pca.setInputCloud(g_cloud_filtered);
  Eigen::Vector3f eigenvalues = pca.getEigenValues();
  Eigen::Matrix3f eigenvectors = pca.getEigenVectors();

  // The principal axis direction is the eigenvector corresponding to the largest eigenvalue
  Eigen::Vector3f principal_axis = eigenvectors.col(0);
  
  // Calculate angle
  Eigen::Vector3f global_y_axis(0, 1, 0);
  float cosine_of_angle = principal_axis.dot(global_y_axis) / (principal_axis.norm() * global_y_axis.norm());
  float angle_radians = acos(cosine_of_angle);
  
  // Avoid obtuse angles, make angles acute
  if (angle_radians > M_PI / 2) angle_radians -= M_PI;
  else if (angle_radians < -M_PI / 2) angle_radians += M_PI;

  // Restriction angle between -π/4 and π/4
  if (angle_radians > M_PI / 4) angle_radians -= M_PI/2;
  else if (angle_radians < -M_PI / 4) angle_radians += M_PI/2;
  
  */

  float angle_radians;

  // Calculate the angle
  if(shape_type == "cross"){
    angle_radians = computeOptimalAngle(g_cloud_filtered, 2.12*size, size, "cross") + M_PI/4; 
    std::cout << "the angle isi is is isi i:" << angle_radians/M_PI*360 << std::endl;
  }else{
    angle_radians = computeOptimalAngle(g_cloud_filtered, 3.5*size, 0.5*size, "nought") - M_PI/4;
    std::cout << "the angle isi is is isi i:" << angle_radians/M_PI*360 << std::endl;
  }

  // Avoid obtuse angles, make angles acute
  if (angle_radians > M_PI / 2) angle_radians -= M_PI;
  else if (angle_radians < -M_PI / 2) angle_radians += M_PI;

  // Restriction angle between -π/4 and π/4
  if (angle_radians > M_PI / 4) angle_radians -= M_PI/2;
  else if (angle_radians < -M_PI / 4) angle_radians += M_PI/2;

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

  removeCollision(GROUND_COLLISION_);

  return true;
}

/////////////////////////////////////////////////////////////////////////

std::string
cw3::determineShape()
{
  std::string shape = "nought";

  applyVX(g_cloud_ptr, g_cloud_filtered);

  applyGroundFilter(g_cloud_filtered,g_cloud_filtered);

  applyPassthrough(g_cloud_filtered,g_cloud_filtered,"x");

  applyPassthrough(g_cloud_filtered,g_cloud_filtered,"y");

  pubFilteredPCMsg(g_pub_cloud, *g_cloud_filtered);

  float num_points = g_cloud_filtered->size();

  if (num_points > 10)
  {
    return "cross";
  }
  return "nought";
}

///////////////////////////////////////////////////////////////////////////////

float
cw3::euclidDistance(geometry_msgs::Point p1,geometry_msgs::Point p2)
{
  double dx = p2.x - p1.x;
  double dy = p2.y - p1.y;
  double dz = p2.z - p1.z;
  return std::sqrt(dx*dx + dy*dy + dz*dz);
}

int64_t
cw3::t2(std::vector<geometry_msgs::PointStamped>& ref_object_points, geometry_msgs::PointStamped mystery_object_point)
{
  addGroundCollision();

  std::vector<geometry_msgs::Point> togo(2);

  if (crt_ee_position.x != -9999)
  {
    if (euclidDistance(crt_ee_position,mystery_object_point.point)<euclidDistance(crt_ee_position,ref_object_points[0].point))
    {
       togo[0] = mystery_object_point.point;
       togo[1] = ref_object_points[0].point;
    }else{
      togo[0] = ref_object_points[0].point;
      togo[1] = mystery_object_point.point;
    }
  }else{
    togo[0] = mystery_object_point.point;
    togo[1] = ref_object_points[0].point;
  }

  geometry_msgs::Pose target_pose;
  std::array<std::string, 2> object_types;

  for (size_t i = 0; i < 2; ++i) {
      const auto& point = togo[i];
      target_pose = moveAbovePose(point);
      target_pose.position.z = 0.7; 
      target_pose.position.x += camera_offset_; 
      moveArm(target_pose);
      object_types[i] = determineShape();
  }

  removeCollision(GROUND_COLLISION_);

  if (object_types[0] == object_types[1]){
    return 1;
  }
  return 2;
}

PointCPtr
cw3::scanEnvironment()
{

  double x = 0.31;
  double y = 0.2;
  double z = 0.88;

  geometry_msgs::Point initPoint;
  initPoint.x = x;
  initPoint.y = 0;
  initPoint.z = z;

  geometry_msgs::Point point1;
  point1.x = x;
  point1.y = y;
  point1.z = z;

  geometry_msgs::Point point2;
  point2.x = x;
  point2.y = -y;
  point2.z = z;

  geometry_msgs::Point point3;
  point3.x = -x-0.02;
  point3.y = -y;
  point3.z = z;

  geometry_msgs::Point point4;
  point4.x = -x-0.02;
  point4.y = y;
  point4.z = z;

    // geometry_msgs::Point point1;
  // point1.x = 0.29;
  // point1.y = 0;
  // point1.z = 0.91;

  // geometry_msgs::Point point2;
  // point2.x = -0.29;
  // point2.y = 0;
  // point2.z = 0.91;

  std::vector<geometry_msgs::Point> corners(4);
  corners[0] = point1;
  corners[1] = point2;
  corners[2] = point3;
  corners[3] = point4;

  // Move arm to initial position
  geometry_msgs::Pose target_pose = moveAbovePose(initPoint);
  moveArm(target_pose);

  geometry_msgs::Point point;

  PointCPtr full_scene_cloud (new PointC);

  for (size_t i = 0; i < corners.size(); ++i) {
      const auto& point = corners[i];
      target_pose = moveAbovePose(point);
      moveArm(target_pose);
      // octomap::OcTree tree(0.01); // Adjust resolution as needed
      // applyGroundFilter(g_cloud_ptr,g_cloud_filtered);

      // for (const auto& point : *g_cloud_filtered) {
      //     tree.updateNode(octomap::point3d(point.x, point.y, point.z), true); // Mark the cell as occupied
      // }
      // // Publish OctoMap
      // octomap_msgs::Octomap octomap_msg;
      // octomap_msgs::fullMapToMsg(tree, octomap_msg);

      // octomap_msg.header.frame_id = g_cloud_filtered->header.frame_id;
      // octomap_msg.header.stamp = pcl_conversions::fromPCL(g_cloud_filtered->header.stamp);
      // octomap_publisher.publish(octomap_msg);

      PointCPtr world_cloud (new PointC);

      applyGroundFilter(g_cloud_ptr,g_cloud_filtered);

      // applyPassthrough(g_cloud_filtered,g_cloud_filtered,"z",0.8);

      pubFilteredPCMsg(g_pub_cloud,*g_cloud_filtered);

      pcl_ros::transformPointCloud(BASE_FRAME_, *g_cloud_filtered, *world_cloud, listener_);

      if (i==0)
      {
        *full_scene_cloud = *world_cloud;
      }else{
        *full_scene_cloud += *world_cloud;
      }

      
  }

  pubFilteredPCMsg(full_scene_pub_cloud,*full_scene_cloud);

  return full_scene_cloud;
}


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

void 
cw3::publishMarker(float x, float y,float z,int id)
{

  // Set up the marker
  visualization_msgs::Marker marker;
  marker.header.frame_id = BASE_FRAME_;
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE; // Set the marker type
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x; // Set the position
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.05; // Set the scale
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.color.r = 1.0; // Set the color (in RGB)
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(); // Set the lifetime

  // Publish the marker
  marker_pub.publish(marker);
}

std::array<int64_t, 2>
cw3::t3()
{
  addGroundCollision();

  std::vector<geometry_msgs::Point> noughts;
  std::vector<geometry_msgs::Point> crosses;

  int obstacle_count = 0;

  geometry_msgs::Point basket_point;
  basket_point.z = 0;

  std::array<int64_t, 2> res;

  PointCPtr full_scene_cloud = scanEnvironment();

  std::vector<pcl::PointIndices> cluster_indices = dbscanClustering(full_scene_cloud);

  std::cout<<"Number of clusters: "<<cluster_indices.size()<<std::endl;

  int id = 0;

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

    float avg_r = static_cast<float>(sum_r / num_points);
    float avg_g = static_cast<float>(sum_g / num_points);
    float avg_b = static_cast<float>(sum_b / num_points);

    geometry_msgs::Point centroid;
    centroid.x = avg_x;
    centroid.y = avg_y;
    centroid.z = 0;

    publishMarker(avg_x,avg_y,0,id);
    id++;

    std::cout<<"==================================="<<std::endl;
    std::cout<<"Cluster Size: "<<num_points<<std::endl;
    // std::cout<<"Centroid X: "<<avg_x<<std::endl;
    // std::cout<<"Centroid Y: "<<avg_y<<std::endl;
    std::cout<<"Average R: "<<avg_r<<std::endl;
    std::cout<<"Average G: "<<avg_g<<std::endl;
    std::cout<<"Average B: "<<avg_b<<std::endl;

    bool cluster_not_black = (avg_r + avg_g + avg_b) > 90;

    // std::cout<<"Color: "<<cluster_color<<std::endl;

    if (cluster_not_black)
    {
      float min_dist = 9999;
      float max_dist = 0;
      float distance;
      geometry_msgs::Point cloud_point;
      cloud_point.z = 0;
      for (const auto& idx : indices.indices) {
              const auto& point = full_scene_cloud->points[idx];
              cloud_point.x = point.x;
              cloud_point.y = point.y;
              distance = euclidDistance(centroid,cloud_point);
              min_dist = std::min(min_dist,distance);
              max_dist = std::max(max_dist,distance);
      }

      std::cout<<"Min Distance: "<<min_dist<<std::endl;
      std::cout<<"Max Distance: "<<max_dist<<std::endl;

      if (max_dist > 0.2)
      {
        std::cout<<"This is a basket"<<std::endl;
        basket_point.x = avg_x;
        basket_point.y = avg_y;
      }
      else if (min_dist > 0.01){
        std::cout<<"This is a nought"<<std::endl;
        noughts.push_back(centroid);
      }else{
        std::cout<<"This is a cross"<<std::endl;
        crosses.push_back(centroid);
      }
    }else{
      // add obstacle collision
      geometry_msgs::Point obstacle_point = centroid;
      addObstacleCollision(obstacle_point,std::to_string(obstacle_count));

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
    t1(noughts[0], basket_point, "noughts");
  }else{
    t1(crosses[0], basket_point, "crosses");
  } 

  int64_t total_num_shapes = static_cast<int>(noughts.size() + crosses.size());
  int64_t num_most_common = static_cast<int>(std::max(noughts.size(), crosses.size()));

  res[0] = total_num_shapes;
  res[1] = num_most_common;

  removeCollision(GROUND_COLLISION_);

  removeObstacles(obstacle_count);

  return res;
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
  collision_object.header.frame_id = BASE_FRAME_;

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
cw3::removeObstacles(int obstacle_count)
{
  for (int i=0;i<obstacle_count;i++)
  {
    removeCollision(std::to_string(i));
  }
}


void
cw3::addGroundCollision(float ground_height)
{
  geometry_msgs::Vector3 ground_dimension;
  ground_dimension.x = 5;
  ground_dimension.y = 5;
  ground_dimension.z = ground_height;

  geometry_msgs::Point ground_position;
  ground_position.x = 0;
  ground_position.y = 0;
  ground_position.z = 0.01;

  geometry_msgs::Quaternion ground_orientation;
  ground_orientation.w = 1;
  ground_orientation.x = 0;
  ground_orientation.y = 0;
  ground_orientation.z = 0;

  addCollision(GROUND_COLLISION_,ground_position,ground_dimension,ground_orientation);
}

void
cw3::addObstacleCollision(geometry_msgs::Point obstacle_centroid,std::string obj_name)
{
  geometry_msgs::Quaternion orientation;
  orientation.w = 1;
  orientation.x = 0;
  orientation.y = 0;
  orientation.z = 0;

  geometry_msgs::Vector3 dimension;
  dimension.x = 0.1;
  dimension.y = 0.1;
  dimension.z = 0.5;

  addCollision(obj_name,obstacle_centroid,dimension,orientation);
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

// Filter function with inputs as references to point cloud smart pointers
void 
cw3::applyGroundFilter(PointCPtr &input_cloud, PointCPtr &output_cloud) {
    pcl::ConditionalRemoval<PointT> color_filter;
    color_filter.setInputCloud(input_cloud);

    pcl::PackedRGBComparison<PointT>::Ptr green_condition(new pcl::PackedRGBComparison<PointT>("g",pcl::ComparisonOps::LT,54));
    pcl::ConditionAnd<PointT>::Ptr color_cond (new pcl::ConditionAnd<PointT>());
    color_cond->addComparison(green_condition);
    color_filter.setCondition(color_cond);
    color_filter.filter(*output_cloud);
}


void
cw3::applyPassthrough(PointCPtr &in_cloud_ptr,
                      PointCPtr &out_cloud_ptr,
                      std::string axis,
                      float threshold)
{
  PointCPtr cloud_filtered_y(new PointC);
  // Apply a pass-through filter on the y-axis
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(in_cloud_ptr); // Set the input point cloud
  pass.setFilterFieldName(axis); // Set the filtering field to the axis
  pass.setFilterLimits(-threshold, threshold); // Set the filtering range
  pass.filter(*out_cloud_ptr); // Execute the filtering
}

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

/**
 * @brief Performs DBSCAN clustering on a given point cloud.
 *
 * Modifies the Z coordinates of all points in the cloud to 0, effectively flattening the cloud
 * for 2D clustering. It uses a KdTree for efficient neighbor search in the clustering process.
 * The DBSCAN clustering parameters—cluster tolerance, minimum cluster size, and maximum cluster
 * size—are set before extracting the clusters.
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
    // Set the Z coordinates of all points to 0
    for (auto &point : *cloud) {
        point.z = 0.0;
    }

    // Creating a KdTree object for finding neighbors
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    tree->setInputCloud(cloud);

    // Create DBSCAN clustering object and set parameters
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.007); // 2cm search radius
    ec.setMinClusterSize(100);   // Minimum Cluster Size
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

 
    return cluster_indices;
}