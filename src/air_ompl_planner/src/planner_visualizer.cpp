/*
 * OMPL-based 3D Route Planner
 * Copyright (C) 2022 Fan Yang - All rights reserved
 * fanyang2@alumni.cmu.edu  
 */


#include "air_ompl_planner/planner_visualizer.h"

/***************************************************************************************/


void DPVisualizer::Init(const ros::NodeHandle& nh) {
    nh_ = nh;
    point_cloud_ptr_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
    // Rviz Publisher
    viz_path_pub_ = nh_.advertise<MarkerArray>("/viz_path_topic", 5);
    viz_map_pub_  = nh_.advertise<MarkerArray>("/viz_grid_map_topic", 5);
}

void DPVisualizer::VizPoint3D(const Point3D& point, 
                             const std::string& ns,
                             const VizColor& color,
                             const float scale,
                             const float alpha)
{
    Marker node_marker;
    node_marker.type = Marker::SPHERE;
    this->SetMarker(color, ns, scale, alpha, node_marker);
    std::size_t idx = 0;
    node_marker.pose.position.x = point.x;
    node_marker.pose.position.y = point.y;
    node_marker.pose.position.z = point.z;
    viz_node_pub_.publish(node_marker);
}

void DPVisualizer::VizPath(const PointStack& execute_path, const PointStack& current_path) {
    Marker ect_path_marker, cur_path_marker;
    ect_path_marker.type = Marker::LINE_STRIP;
    cur_path_marker.type = Marker::LINE_STRIP;
    MarkerArray path_marker_array;
    this->SetMarker(VizColor::BLUE, "execute_path", 0.5f, 0.9f, ect_path_marker);
    this->SetMarker(VizColor::GREEN, "current_path", 0.5f, 0.9f, cur_path_marker);
    geometry_msgs::Point geo_p;
    for (const auto& p : execute_path) {
        geo_p = AOMPLUtil::Point3DToGeoMsgPoint(p);
        ect_path_marker.points.push_back(geo_p);
    }
    for (const auto& p : current_path) {
        geo_p = AOMPLUtil::Point3DToGeoMsgPoint(p);
        cur_path_marker.points.push_back(geo_p);
    } 
    path_marker_array.markers.push_back(ect_path_marker);
    path_marker_array.markers.push_back(cur_path_marker);
    viz_path_pub_.publish(path_marker_array);
}

void DPVisualizer::VizMapGrids(const PointStack& neighbor_centers, const PointStack& occupancy_centers,
                               const float& ceil_length, const float& ceil_height)
{
    MarkerArray map_grid_marker_array;
    Marker neighbor_marker, occupancy_marker;
    neighbor_marker.type = Marker::CUBE_LIST;
    occupancy_marker.type = Marker::CUBE_LIST;
    this->SetMarker(VizColor::GREEN, "neighbor_grids",  ceil_length / AOMPLUtil::kVizRatio, 0.3f,  neighbor_marker);
    this->SetMarker(VizColor::RED,   "occupancy_grids", ceil_length / AOMPLUtil::kVizRatio, 0.2f, occupancy_marker);
    neighbor_marker.scale.z = occupancy_marker.scale.z = ceil_height;
    const std::size_t N1 = neighbor_centers.size();
    const std::size_t N2 = occupancy_centers.size();
    neighbor_marker.points.resize(N1), occupancy_marker.points.resize(N2);
    for (std::size_t i=0; i<N1; i++) {
        geometry_msgs::Point p = AOMPLUtil::Point3DToGeoMsgPoint(neighbor_centers[i]);
        neighbor_marker.points[i] = p;
    }
    for (std::size_t i=0; i<N2; i++) {
        geometry_msgs::Point p = AOMPLUtil::Point3DToGeoMsgPoint(occupancy_centers[i]);
        occupancy_marker.points[i] = p;
    }
    map_grid_marker_array.markers.push_back(neighbor_marker);
    map_grid_marker_array.markers.push_back(occupancy_marker);
    viz_map_pub_.publish(map_grid_marker_array);
}

void DPVisualizer::SetMarker(const VizColor& color, 
                             const std::string& ns,
                             const float& scale,
                             const float& alpha,  
                             Marker& scan_marker, 
                             const float& scale_ratio) 
{
    scan_marker.header.frame_id = AOMPLUtil::worldFrameId;
    scan_marker.header.stamp = ros::Time::now();
    scan_marker.id = 0;
    scan_marker.ns = ns;
    scan_marker.action = Marker::ADD;
    scan_marker.scale.x = scan_marker.scale.y = scan_marker.scale.z = scale * scale_ratio;
    scan_marker.pose.orientation.x = 0.0;
    scan_marker.pose.orientation.y = 0.0;
    scan_marker.pose.orientation.z = 0.0;
    scan_marker.pose.orientation.w = 1.0;
    scan_marker.pose.position.x = 0.0;
    scan_marker.pose.position.y = 0.0;
    scan_marker.pose.position.z = 0.0;
    DPVisualizer::SetColor(color, alpha, scan_marker);
}

void DPVisualizer::VizPointCloud(const ros::Publisher& viz_pub, 
                                 const PointCloudPtr& pc) 
{
    sensor_msgs::PointCloud2 msg_pc;
    pcl::toROSMsg(*pc, msg_pc);
    msg_pc.header.frame_id = AOMPLUtil::worldFrameId;
    msg_pc.header.stamp = ros::Time::now();
    viz_pub.publish(msg_pc);
}

void DPVisualizer::SetColor(const VizColor& color, 
                            const float& alpha, 
                            Marker& scan_marker)
{
    std_msgs::ColorRGBA c;
    c.a = alpha;
    if (color == VizColor::RED) {
    c.r = 1.0f, c.g = c.b = 0.f;
    }
    else if (color == VizColor::ORANGE) {
    c.r = 1.0f, c.g = 0.45f, c.b = 0.1f;
    }
    else if (color == VizColor::BLACK) {
    c.r = c.g = c.b = 0.1f;
    }
    else if (color == VizColor::YELLOW) {
    c.r = c.g = 0.9f, c.b = 0.1;
    }
    else if (color == VizColor::BLUE) {
    c.b = 1.0f, c.r = 0.1f, c.g = 0.1f;
    }
    else if (color == VizColor::GREEN) {
    c.g = 0.9f, c.r = c.b = 0.f;
    }
    else if (color == VizColor::EMERALD) {
    c.g = c.b = 0.9f, c.r = 0.f;
    }
    else if (color == VizColor::WHITE) {
    c.r = c.g = c.b = 0.9f;
    }
    else if (color == VizColor::MAGNA) {
    c.r = c.b = 0.9f, c.g = 0.f;
    }
    else if (color == VizColor::PURPLE) {
    c.r = c.b = 0.5f, c.g = 0.f;
    }
    scan_marker.color = c;
}

