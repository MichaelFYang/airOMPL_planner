/*
 * OMPL-based 3D Route Planner
 * Copyright (C) 2022 Fan Yang - All rights reserved
 * fanyang2@alumni.cmu.edu  
 */


#include "air_ompl_planner/air_planner.h"

/***************************************************************************************/

void PlannerMaster::Init() {
  /* initialize subscriber and publisher */
  odom_sub_            = nh.subscribe("/odom_world", 5, &PlannerMaster::OdomCallBack, this);
  scan_sub_            = nh.subscribe("/scan_cloud", 1, &PlannerMaster::ScanCallBack, this);
  waypoint_sub_        = nh.subscribe("/goal_point", 1, &PlannerMaster::WaypointCallBack, this);
  goal_pub_            = nh.advertise<geometry_msgs::PointStamped>("/way_point",5);
  surround_obs_debug_  = nh.advertise<sensor_msgs::PointCloud2>("/OMPL_obs_debug",1);
  this->LoadROSParams();
  /* init Dynamic Planner Processing Objects */
  map_handler_.Init(map_params_);
  ompl_planner_.Init(ompl_params_);
  planner_viz_.Init(nh);
  scan_handler_.Init(scan_params_);

  /* init internal params */
  is_cloud_init_      = false;
  is_odom_init_       = false;
  is_goal_activity_   = false;
  goal_pos_           = Point3D(0,0,0);

  // allocate memory to pointers
  temp_cloud_ptr_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());

  // init TF listener
  tf_listener_ = new tf::TransformListener();

  // clear temp vectors and memory
  AOMPLUtil::robot_pos = Point3D(0,0,0);
  execute_path.clear(), current_path.clear();

  robot_pos_   = Point3D(0,0,0);
  goal_waypoint_stamped_.header.frame_id = master_params_.world_frame;
  // printf("\033[2J"), printf("\033[0;0H"); // cleanup screen
  // std::cout<<std::endl;
  std::cout<<"\033[1;33m **************** OMPL PLANNING **************** \033[0m\n"<<std::endl;
  std::cout<<"\n"<<std::endl;
}

void PlannerMaster::Loop() {
  ros::Rate loop_rate(master_params_.main_run_freq);
  while (ros::ok()) {
    /* Process callback functions */
    ros::spinOnce(); 
    if (!this->PreconditionCheck()) {
      loop_rate.sleep();
      continue;
    }
    if (is_goal_activity_) {
      Point3D waypoint_msg;
      PointStack execute_path, current_path;
      ompl_planner_.PlanPath(robot_pos_, goal_pos_, waypoint_msg, execute_path, current_path);
      // planning status determine
      if (execute_path.size() > 1) { 
        // planning undergoing
        PublishWaypointMsg(waypoint_msg);
        std::cout<<"Executing the path ..."<<std::endl;
      } else if (execute_path.size() == 1) {
        // goal reached
        ROS_INFO("********************* Goal Reached ********************");
        execute_path.clear(), current_path.clear();
        is_goal_activity_ = false;
      } else {
        // planning fails
        ROS_INFO("********************* Planning Timeout: Failed ********************");
        execute_path.clear(), current_path.clear();
        is_goal_activity_ = false;
      }
      planner_viz_.VizPath(execute_path, current_path);
    }   
    loop_rate.sleep();
  }
}


void PlannerMaster::LoadROSParams() {
  const std::string master_prefix   = "/air_ompl_planner/";
  const std::string map_prefix      = master_prefix + "MapHandler/";
  const std::string planner_prefix  = master_prefix + "OMPLPlanner/";

  // master params
  nh.param<float>(master_prefix + "voxel_dim",         master_params_.voxel_dim, 0.2);
  nh.param<float>(master_prefix + "robot_dim",         master_params_.robot_dim, 0.8);
  nh.param<float>(master_prefix + "sensor_range",      master_params_.sensor_range, 10.0);
  nh.param<float>(master_prefix + "visualize_ratio",   master_params_.viz_ratio, 1.0);
  nh.param<float>(master_prefix + "main_run_freq",     master_params_.main_run_freq, 2.0);
  nh.param<float>(master_prefix + "ceil_height",       master_params_.ceil_height, 2.0);
  nh.param<bool>(master_prefix +  "is_static_env",     master_params_.is_static_env, true);
  nh.param<std::string>(master_prefix + "world_frame", master_params_.world_frame, "map");

    // ompl planner params
  nh.param<float>(planner_prefix + "planner_step", ompl_params_.dstep, 1.0f);
  nh.param<float>(planner_prefix + "collision_radius", ompl_params_.collision_r, 1.0f);
  nh.param<float>(planner_prefix + "converage_dist", ompl_params_.converage_d, 2.0f);
  nh.param<float>(planner_prefix + "project_dist", ompl_params_.project_d, 2.0f);

  // map handler params
  nh.param<float>(map_prefix + "cell_height", map_params_.cell_z, 2.0);
  nh.param<float>(map_prefix + "cell_length", map_params_.cell_x, 5.0);
  nh.param<float>(map_prefix + "max_length",  map_params_.max_length, 5000.0);
  map_params_.sensor_range     = master_params_.sensor_range;
  map_params_.ceil_height      = master_params_.ceil_height;
  map_params_.margin_d         = ompl_params_.collision_r * 2.0f;

  // utility params
  nh.param<int>(map_prefix + "collide_counter", AOMPLUtil::kCollideN, 2.0);
  AOMPLUtil::robot_dim = master_params_.robot_dim;
  AOMPLUtil::kLeafSize = master_params_.voxel_dim;
  AOMPLUtil::kSensorRange = master_params_.sensor_range;
  AOMPLUtil::kVizRatio = master_params_.viz_ratio;
  AOMPLUtil::worldFrameId = master_params_.world_frame;

  // scan handler params
  scan_params_.terrain_range = master_params_.sensor_range;
  scan_params_.voxel_size    = master_params_.voxel_dim;
  scan_params_.ceil_height   = map_params_.ceil_height;
}

void PlannerMaster::OdomCallBack(const nav_msgs::OdometryConstPtr& msg) {
  // transform from odom frame to mapping frame
  std::string odom_frame = msg->header.frame_id;
  tf::Pose tf_odom_pose;
  tf::poseMsgToTF(msg->pose.pose, tf_odom_pose);
  if (!AOMPLUtil::IsSameFrameID(odom_frame, master_params_.world_frame)) {
    tf::StampedTransform odom_to_world_tf_stamp;
    try
    {
      tf_listener_->waitForTransform(master_params_.world_frame, odom_frame, msg->header.stamp, ros::Duration(2.0));
      tf_listener_->lookupTransform(master_params_.world_frame, odom_frame, msg->header.stamp, odom_to_world_tf_stamp);
      tf_odom_pose = odom_to_world_tf_stamp * tf_odom_pose;
    }
    catch (tf::TransformException ex){
      ROS_ERROR("Tracking odom TF lookup: %s",ex.what());
      return;
    }
  }
  robot_pos_.x = tf_odom_pose.getOrigin().getX(); 
  robot_pos_.y = tf_odom_pose.getOrigin().getY();
  robot_pos_.z = tf_odom_pose.getOrigin().getZ();
  // extract robot heading
  AOMPLUtil::robot_pos = robot_pos_;
  double roll, pitch, yaw;
  tf_odom_pose.getBasis().getRPY(roll, pitch, yaw);
  robot_heading_ = Point3D(cos(yaw), sin(yaw), 0);

  if (!is_odom_init_) {
    map_handler_.UpdateRobotPosition(robot_pos_);
  }

  is_odom_init_ = true;
}

bool PlannerMaster::ProcessCloud(const sensor_msgs::PointCloud2ConstPtr& pc,
                              const PointCloudPtr& cloudOut) 
{

  pcl::PointCloud<PCLPoint> temp_cloud;
  pcl::PointCloud<pcl::PointXYZ> temp_msg_cloud;
  pcl::fromROSMsg(*pc, temp_msg_cloud);
                                  
  AOMPLUtil::FilterCloud(temp_msg_cloud, master_params_.voxel_dim);
  pcl::copyPointCloud(temp_msg_cloud, temp_cloud);
  cloudOut->clear(), *cloudOut = temp_cloud;
  if (cloudOut->empty()) return false;
  // transform cloud frame
  std::string cloud_frame = pc->header.frame_id;
  AOMPLUtil::RemoveNanInfPoints(cloudOut);
  if (!AOMPLUtil::IsSameFrameID(cloud_frame, master_params_.world_frame)) {
    try
    {
      AOMPLUtil::TransformPCLFrame(cloud_frame, 
                                  master_params_.world_frame,
                                  pc->header.stamp, 
                                  tf_listener_,
                                  cloudOut);
    }
    catch(tf::TransformException ex)
    {
      ROS_ERROR("Tracking cloud TF lookup: %s",ex.what());
      return false;
    }
  }
  AOMPLUtil::CropBoxCloud(*cloudOut, robot_pos_, Point3D(master_params_.sensor_range,
                                                        master_params_.sensor_range,
                                                        master_params_.sensor_range));
  return true;
}

void PlannerMaster::ExtractDynamicObsFromScan(const PointCloudPtr& scanCloudIn, 
                                              const PointCloudPtr& obsCloudIn,
                                              const PointCloudPtr& dyObsCloudOut)
{
  scan_handler_.ReInitGrids();
  scan_handler_.SetCurrentScanCloud(scanCloudIn);
  scan_handler_.ExtractDyObsCloud(obsCloudIn, dyObsCloudOut);
}

void PlannerMaster::ScanCallBack(const sensor_msgs::PointCloud2ConstPtr& pc) {
  if (!is_odom_init_) return;
  // update map grid robot center
  map_handler_.UpdateRobotPosition(robot_pos_);
  if (!this->ProcessCloud(pc, temp_cloud_ptr_)) {
    return;
  }
  // extract new points
  AOMPLUtil::ExtractNewObsPointCloud(temp_cloud_ptr_,                  // curnent scan cloud
                                     AOMPLUtil::surround_obs_cloud_,
                                     AOMPLUtil::cur_new_cloud_);
  
  map_handler_.UpdateObsCloudGrid(AOMPLUtil::cur_new_cloud_);

  // update surround obs cloud
  map_handler_.GetSurroundObsCloud(AOMPLUtil::surround_obs_cloud_);
  ROS_INFO_STREAM("size of cloud: "<<AOMPLUtil::surround_obs_cloud_->size());

  if (!AOMPLUtil::surround_obs_cloud_->empty()) is_cloud_init_ = true;

  /* Extract Dynamic Obs in Cloud */
  if (!master_params_.is_static_env) {
    this->ExtractDynamicObsFromScan(temp_cloud_ptr_, 
                                    AOMPLUtil::surround_obs_cloud_,
                                    AOMPLUtil::cur_dyobs_cloud_);

    if (AOMPLUtil::cur_dyobs_cloud_->size() > 5) {
      ROS_WARN("Dynamic Obstacle Detected, removing from map...");
      AOMPLUtil::InflateCloud(AOMPLUtil::cur_dyobs_cloud_, master_params_.voxel_dim, 1, true);
      map_handler_.RemoveObsCloudFromGrid(AOMPLUtil::cur_dyobs_cloud_);

      AOMPLUtil::RemoveOverlapCloud(AOMPLUtil::surround_obs_cloud_, AOMPLUtil::cur_dyobs_cloud_);
      AOMPLUtil::FilterCloud(AOMPLUtil::cur_dyobs_cloud_, master_params_.voxel_dim);
      // update new cloud
      *AOMPLUtil::cur_new_cloud_ += *AOMPLUtil::cur_dyobs_cloud_;
      AOMPLUtil::FilterCloud(AOMPLUtil::cur_new_cloud_, master_params_.voxel_dim);
    }
  }

  /* visualize clouds */
  planner_viz_.VizPointCloud(surround_obs_debug_,  AOMPLUtil::surround_obs_cloud_);
  // visualize map grid
  PointStack neighbor_centers, occupancy_centers;
  map_handler_.GetNeighborCeilsCenters(neighbor_centers);
  map_handler_.GetOccupancyCeilsCenters(occupancy_centers);
  planner_viz_.VizMapGrids(neighbor_centers, occupancy_centers, map_params_.cell_x, map_params_.cell_z);
  ROS_INFO_STREAM("neighbor_centers: "<<neighbor_centers.size()<<"; occupancy_centers: "<<occupancy_centers.size());
}

void PlannerMaster::WaypointCallBack(const geometry_msgs::PointStamped& route_goal) {
  Point3D goal_p(route_goal.point.x, route_goal.point.y, route_goal.point.z);
  const std::string goal_frame = route_goal.header.frame_id;
  if (!AOMPLUtil::IsSameFrameID(goal_frame, master_params_.world_frame)) {
    AOMPLUtil::TransformPoint3DFrame(goal_frame, master_params_.world_frame, tf_listener_, goal_p); 
  }
  goal_pos_ = goal_p;
  is_goal_activity_ = true;
  ompl_planner_.ResetExecutePath();
  ROS_INFO("PlannerMaster: goal received.");
}

void PlannerMaster::PublishWaypointMsg(const Point3D& goal_pos_) {
  geometry_msgs::PointStamped wp_ros_msg;
  wp_ros_msg.header.frame_id = master_params_.world_frame;
  wp_ros_msg.header.stamp = ros::Time::now();
  wp_ros_msg.point.x = goal_pos_.x, wp_ros_msg.point.y = goal_pos_.y, wp_ros_msg.point.z = goal_pos_.z;
  goal_pub_.publish(wp_ros_msg);
}

/* allocate static utility PointCloud pointer memory */
PointCloudPtr  AOMPLUtil::surround_obs_cloud_  = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  AOMPLUtil::cur_new_cloud_       = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  AOMPLUtil::cur_dyobs_cloud_     = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
/* init static utility values */
const float AOMPLUtil::kEpsilon = 1e-7;
const float AOMPLUtil::kINF     = std::numeric_limits<float>::max();
Point3D AOMPLUtil::robot_pos;
Point3D AOMPLUtil::robot_head;
float   AOMPLUtil::robot_dim;
float   AOMPLUtil::kLeafSize;
float   AOMPLUtil::kSensorRange;
float   AOMPLUtil::kVizRatio;
int     AOMPLUtil::kCollideN;
std::string AOMPLUtil::worldFrameId;

/* init map static values */
std::unique_ptr<grid_ns::Grid<CloudType>> MapHandler::world_obs_cloud_grid_;
std::vector<int> MapHandler::global_visited_induces_;
MapHandlerParams MapHandler::map_params_;

/* init planner static values */
OMPLPlannerParams OMPLPlanner::ompl_params_;

int main(int argc, char** argv){
  ros::init(argc, argv, "air_ompl_planner");
  PlannerMaster air_ompl_planner_node;
  air_ompl_planner_node.Init();
  air_ompl_planner_node.Loop();
}
