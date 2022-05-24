/*
 * OMPL-based 3D Route Planner
 * Copyright (C) 2022 Fan Yang - All rights reserved
 * fanyang2@alumni.cmu.edu  
 */



#include "air_ompl_planner/map_handler.h"

/***************************************************************************************/

void MapHandler::Init(const MapHandlerParams& params) {
    map_params_ = params;
    const int row_num = std::ceil(map_params_.max_length / map_params_.cell_x);
    const int col_num = row_num;
    int level_num = std::ceil(map_params_.ceil_height * 2.0f / map_params_.cell_z);
    neighbor_Lnum_ = std::ceil(map_params_.sensor_range * 2.0f / map_params_.cell_x) + 1; 
    neighbor_Hnum_ = neighbor_Lnum_ / 2; 
    // force to odd number, then the robot will be at the center
    if (level_num % 2 == 0) level_num ++;
    if (neighbor_Lnum_ % 2 == 0) neighbor_Lnum_ ++;
    if (neighbor_Hnum_ % 2 == 0) neighbor_Hnum_ ++;

    // inlitialize grid 
    Eigen::Vector3i pointcloud_grid_size(row_num, col_num, level_num);
    Eigen::Vector3d pointcloud_grid_origin(0,0,0);
    Eigen::Vector3d pointcloud_grid_resolution(map_params_.cell_x, map_params_.cell_x, map_params_.cell_z);
    CloudType cloud_type_temp;

    world_obs_cloud_grid_ = std::make_unique<grid_ns::Grid<CloudType>>(
        pointcloud_grid_size, cloud_type_temp, pointcloud_grid_origin, pointcloud_grid_resolution, 3);

    const int n_cell  = world_obs_cloud_grid_->GetCellNumber();
    for (int i = 0; i < n_cell; i++) {
        world_obs_cloud_grid_->GetCell(i).cloud_ptr  = PointCloudPtr(new PointCloud);
        world_obs_cloud_grid_->GetCell(i).kdtree_ptr = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>());
        world_obs_cloud_grid_->GetCell(i).kdtree_ptr->setSortedResults(false);
    }
    global_visited_induces_.resize(n_cell), util_remove_check_list_.resize(n_cell), util_obs_modified_list_.resize(n_cell);

    std::fill(global_visited_induces_.begin(), global_visited_induces_.end(), 0);
    std::fill(util_obs_modified_list_.begin(), util_obs_modified_list_.end(), 0);
    std::fill(util_remove_check_list_.begin(), util_remove_check_list_.end(), 0);
}

void MapHandler::ResetGripMapCloud() {
    const int n_cell = world_obs_cloud_grid_->GetCellNumber();
    for (int i=0; i<n_cell; i++) {
        world_obs_cloud_grid_->GetCell(i).cloud_ptr->clear();
        AOMPLUtil::ClearKdTree(world_obs_cloud_grid_->GetCell(i).kdtree_ptr);
    }
    std::fill(global_visited_induces_.begin(),     global_visited_induces_.end(),     0);
    std::fill(util_obs_modified_list_.begin(),     util_obs_modified_list_.end(),     0);
    std::fill(util_remove_check_list_.begin(),     util_remove_check_list_.end(),     0);
}

void MapHandler::SetMapOrigin(const Point3D& ori_robot_pos) {
    Point3D map_origin;
    const Eigen::Vector3i dim = world_obs_cloud_grid_->GetSize();
    map_origin.x = ori_robot_pos.x - (map_params_.cell_x * dim.x()) / 2.0f;
    map_origin.y = ori_robot_pos.y - (map_params_.cell_x * dim.y()) / 2.0f;
    map_origin.z = ori_robot_pos.z - (map_params_.cell_z * dim.z()) / 2.0f;
    Eigen::Vector3d pointcloud_grid_origin(map_origin.x, map_origin.y, map_origin.z);
    world_obs_cloud_grid_->SetOrigin(pointcloud_grid_origin);
    is_init_ = true;
    ROS_INFO("MH: Global Cloud Map Grid Initialized.");
}

void MapHandler::UpdateRobotPosition(const Point3D& odom_pos) {
    if (!is_init_) this->SetMapOrigin(odom_pos);
    robot_cell_sub_ = world_obs_cloud_grid_->Pos2Sub(Eigen::Vector3d(odom_pos.x, odom_pos.y, odom_pos.z));
    // Get neighbor indices
    neighbor_obs_indices_.clear();
    const int N = neighbor_Lnum_ / 2;
    const int H = neighbor_Hnum_ / 2;
    Eigen::Vector3i neighbor_sub;
    for (int i = -N; i <= N; i++) {
        neighbor_sub.x() = robot_cell_sub_.x() + i;
        for (int j = -N; j <= N; j++) {
            neighbor_sub.y() = robot_cell_sub_.y() + j;
            for (int k =-H; k <= H; k++) {
                neighbor_sub.z() = robot_cell_sub_.z() + k;
                if (world_obs_cloud_grid_->InRange(neighbor_sub)) {
                    int ind = world_obs_cloud_grid_->Sub2Ind(neighbor_sub);
                    neighbor_obs_indices_.insert(ind);
                }
            }
        }
    }
}

void MapHandler::UpdateObsCloudGrid(const PointCloudPtr& obsCloudInOut) {
    if (!is_init_ || obsCloudInOut->empty()) return;
    std::fill(util_obs_modified_list_.begin(), util_obs_modified_list_.end(), 0);
    for (const auto& point : obsCloudInOut->points) {
        Eigen::Vector3i flag_vec;
        Eigen::Vector3i sub = world_obs_cloud_grid_->Pos2SubExtend(Eigen::Vector3d(point.x, point.y, point.z), map_params_.margin_d, flag_vec);
        if (!world_obs_cloud_grid_->InRange(sub)) continue;
        const int ind = world_obs_cloud_grid_->Sub2Ind(sub);
        world_obs_cloud_grid_->GetCell(ind).cloud_ptr->points.push_back(point);
        util_obs_modified_list_[ind] = 1;
        global_visited_induces_[ind] = 1;
        for (int i = 0; i < 3; i++) {
            if (flag_vec(i) != 0) {
                Eigen::Vector3i extend_sub = sub;
                extend_sub(i) += flag_vec(i);
                if (world_obs_cloud_grid_->InRange(extend_sub)) {
                    const int extend_ind = world_obs_cloud_grid_->Sub2Ind(extend_sub);
                    world_obs_cloud_grid_->GetCell(extend_ind).cloud_ptr->points.push_back(point);
                    util_obs_modified_list_[extend_ind] = 1;
                    global_visited_induces_[extend_ind] = 1;
                }
            }
        }
    }
    // Filter Modified Ceils
    for (int i = 0; i < world_obs_cloud_grid_->GetCellNumber(); ++i) {
      if (util_obs_modified_list_[i] == 1) {
            AOMPLUtil::FilterCloud(world_obs_cloud_grid_->GetCell(i).cloud_ptr, AOMPLUtil::kLeafSize);
            AOMPLUtil::UpdateKdTreeWithCloud(world_obs_cloud_grid_->GetCell(i).cloud_ptr,
                                            world_obs_cloud_grid_->GetCell(i).kdtree_ptr);
      }
    }
}

bool MapHandler::IsPointCollideWithObs(const Point3D& p, const float& radius) {
    PCLPoint pcl_p;
    pcl_p.x = p.x, pcl_p.y = p.y, pcl_p.z = p.z;
    if (!std::isfinite(pcl_p.x) || !std::isfinite(pcl_p.y) || !std::isfinite(pcl_p.z)) {
        return true;
    }
    const Eigen::Vector3i p_sub = world_obs_cloud_grid_->Pos2Sub(Eigen::Vector3d(p.x, p.y, p.z));
    if (!world_obs_cloud_grid_->InRange(p_sub)) {
        return true;
    }
    // local planner limitation
    const float norm_d = std::hypot(p.x - AOMPLUtil::robot_pos.x, p.y - AOMPLUtil::robot_pos.y);
    if (abs(p.z - AOMPLUtil::robot_pos.z) / norm_d  > 0.577f) { // tan(30) == 0.577f
        return true;
    }
    // check with obstacle points
    const std::size_t ind = world_obs_cloud_grid_->Sub2Ind(p_sub);
    if (global_visited_induces_[ind] != 0) {
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        world_obs_cloud_grid_->GetCell(ind).kdtree_ptr->radiusSearch(pcl_p, radius, pointSearchInd, pointSearchSqDis);
        if (pointSearchInd.size() > AOMPLUtil::kCollideN) {
            return true;
        }
    }
    return false;
}

void MapHandler::GetSurroundObsCloud(const PointCloudPtr& obsCloudOut) {
    if (!is_init_) return;
    obsCloudOut->clear();
    for (const auto& neighbor_ind : neighbor_obs_indices_) {
        if (world_obs_cloud_grid_->GetCell(neighbor_ind).cloud_ptr->empty()) continue;
        *obsCloudOut += *(world_obs_cloud_grid_->GetCell(neighbor_ind).cloud_ptr);
    }
}

void MapHandler::GetNeighborCeilsCenters(PointStack& neighbor_centers) {
    if (!is_init_) return;
    neighbor_centers.clear();
    for (const auto& ind : neighbor_obs_indices_) {
        if (global_visited_induces_[ind] == 0) continue;
        Point3D center_p(world_obs_cloud_grid_->Ind2Pos(ind));
        neighbor_centers.push_back(center_p);
    }
}

void MapHandler::GetOccupancyCeilsCenters(PointStack& occupancy_centers) {
    if (!is_init_) return;
    occupancy_centers.clear();
    const int N = world_obs_cloud_grid_->GetCellNumber();
    for (int ind=0; ind<N; ind++) {
        if (global_visited_induces_[ind] == 0) continue;
        Point3D center_p(world_obs_cloud_grid_->Ind2Pos(ind));
        occupancy_centers.push_back(center_p);
    }
}

void MapHandler::RemoveObsCloudFromGrid(const PointCloudPtr& obsCloud) {
    std::fill(util_remove_check_list_.begin(), util_remove_check_list_.end(), 0);
    for (const auto& point : obsCloud->points) {
        Eigen::Vector3i sub = world_obs_cloud_grid_->Pos2Sub(Eigen::Vector3d(point.x, point.y, point.z));
        if (!world_obs_cloud_grid_->InRange(sub)) continue;
        const int ind = world_obs_cloud_grid_->Sub2Ind(sub);
        util_remove_check_list_[ind] = 1;
    }
    for (const auto& ind : neighbor_obs_indices_) {
        if (util_remove_check_list_[ind] == 1 && global_visited_induces_[ind] == 1) {
            AOMPLUtil::RemoveOverlapCloud(world_obs_cloud_grid_->GetCell(ind).cloud_ptr, obsCloud);
            AOMPLUtil::UpdateKdTreeWithCloud(world_obs_cloud_grid_->GetCell(ind).cloud_ptr,
                                            world_obs_cloud_grid_->GetCell(ind).kdtree_ptr);
        }
    }
}
