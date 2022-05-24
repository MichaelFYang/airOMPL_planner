/*
 * OMPL-based 3D Route Planner
 * Copyright (C) 2022 Fan Yang - All rights reserved
 * fanyang2@alumni.cmu.edu  
 */


#include "air_ompl_planner/utility.h"

/***************************************************************************************/

void AOMPLUtil::ExtractNewObsPointCloud(const PointCloudPtr& cloudIn,
                                       const PointCloudPtr& cloudRefer,
                                       const PointCloudPtr& cloudNew)
{
  PointCloudPtr temp_new_cloud(new pcl::PointCloud<PCLPoint>());
  AOMPLUtil::ResetCloudIntensity(cloudIn, false);
  AOMPLUtil::ResetCloudIntensity(cloudRefer, true);
  cloudNew->clear(), temp_new_cloud->clear();
  *temp_new_cloud = *cloudIn + *cloudRefer;
  AOMPLUtil::FilterCloud(temp_new_cloud, AOMPLUtil::kLeafSize);
  for (const auto& p : temp_new_cloud->points) {
    if (p.intensity < 5.0f) {
      cloudNew->points.push_back(p);
    } 
  }
}

void AOMPLUtil::ResetCloudIntensity(const PointCloudPtr& cloudIn, const bool isHigh) {
  const float kValue = isHigh? 255.0 : 0.0;
  for (std::size_t i=0; i<cloudIn->size(); i++) {
    cloudIn->points[i].intensity = kValue;
  }
}

geometry_msgs::Point AOMPLUtil::Point3DToGeoMsgPoint(const Point3D& point) {
  geometry_msgs::Point p;
  p.x = point.x;
  p.y = point.y;
  p.z = point.z;
  return p;
}

void AOMPLUtil::FilterCloud(const PointCloudPtr& point_cloud, const float& leaf_size) {
  const Eigen::Vector3d leaf(leaf_size, leaf_size, leaf_size);
  AOMPLUtil::FilterCloud(*point_cloud, leaf);
}

void AOMPLUtil::ClearKdTree(const PointKdTreePtr& kdTree_ptr) {
  PCLPoint temp_p;
  temp_p.x = temp_p.y = temp_p.z = 0.0f;
  PointCloudPtr temp_cloud(new pcl::PointCloud<PCLPoint>());
  temp_cloud->resize(1), temp_cloud->points[0] = temp_p;
  kdTree_ptr->setInputCloud(temp_cloud);
}

void AOMPLUtil::RemoveOverlapCloud(const PointCloudPtr& cloudInOut,
                                  const PointCloudPtr& cloudRef,
                                  const bool& is_copy_cloud) 
{
  if (cloudRef->empty() || cloudInOut->empty()) return;
  PointCloudPtr temp_cloud(new pcl::PointCloud<PCLPoint>());
  PointCloudPtr ref_cloud = cloudRef;
  if (is_copy_cloud) {
    PointCloudPtr copyRefCloud(new pcl::PointCloud<PCLPoint>());
    pcl::copyPointCloud(*cloudRef, *copyRefCloud);
    ref_cloud = copyRefCloud;
  }
  AOMPLUtil::ResetCloudIntensity(cloudInOut, true);
  AOMPLUtil::ResetCloudIntensity(ref_cloud, false);
  *temp_cloud = *cloudInOut + *ref_cloud;
  const float leaf_size = AOMPLUtil::kLeafSize * 1.2;
  AOMPLUtil::FilterCloud(temp_cloud, leaf_size);
  cloudInOut->clear(), cloudInOut->resize(temp_cloud->size());
  std::size_t idx = 0;
  for (const auto& p : temp_cloud->points) {
    if (p.intensity < 255.0) continue;
    cloudInOut->points[idx] = p;
    idx++;
  }
  cloudInOut->resize(idx);
}

void AOMPLUtil::TransformPoint3DFrame(const std::string& from_frame_id,
                                    const std::string& to_frame_id,
                                    const tf::TransformListener* tf_listener,
                                    Point3D& point)
{
  tf::Vector3 point_vec(point.x, point.y, point.z);
  tf::StampedTransform transform_tf_stamp;
  try {
    tf_listener->waitForTransform(to_frame_id, from_frame_id, ros::Time(0), ros::Duration(2.0));
    tf_listener->lookupTransform(to_frame_id, from_frame_id, ros::Time(0), transform_tf_stamp);
    point_vec = transform_tf_stamp * point_vec;
  } catch (tf::TransformException ex){
    ROS_ERROR("Tracking Point3D TF lookup: %s",ex.what());
    return;
  }
  point.x = point_vec.x();
  point.y = point_vec.y();
  point.z = point_vec.z();
}

bool AOMPLUtil::IsSameFrameID(const std::string& cur_frame, const std::string& ref_frame) {
  std::string str1 = cur_frame;
  std::string str2 = ref_frame;
  if (cur_frame[0] == '/') str1 = cur_frame.substr(1);
  if (ref_frame[0] == '/') str2 = ref_frame.substr(1);
  return str1 == str2;
}

void AOMPLUtil::TransformPCLFrame(const std::string& from_frame_id,
                                 const std::string& to_frame_id,
                                 const ros::Time& time_stamp,
                                 const tf::TransformListener* tf_listener,
                                 const PointCloudPtr& cloudInOut) 
{
  if (cloudInOut->empty()) return;
  pcl::PointCloud<PCLPoint> aft_tf_cloud;
  tf::StampedTransform cloud_to_map_tf;
  try {
    tf_listener->waitForTransform(to_frame_id, from_frame_id, time_stamp, ros::Duration(2.0));
    tf_listener->lookupTransform(to_frame_id, from_frame_id, time_stamp, cloud_to_map_tf);
  } catch (tf::TransformException ex){
    throw ex;
    return;
  }
  pcl_ros::transformPointCloud(*cloudInOut, aft_tf_cloud, cloud_to_map_tf);
  *cloudInOut = aft_tf_cloud;
}

void AOMPLUtil::RemoveNanInfPoints(const PointCloudPtr& cloudInOut) {
  pcl::PointCloud<PCLPoint> temp_cloud;
  temp_cloud.resize(cloudInOut->points.size());
  std::size_t idx = 0;
  for (const auto& p : cloudInOut->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      ROS_WARN_ONCE("OMPLUtil: nan or inf point detected.");
      continue;
    }
    temp_cloud.points[idx] = p;
    idx ++;
  }
  temp_cloud.points.resize(idx);
  *cloudInOut = temp_cloud;
}
