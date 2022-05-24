#ifndef MAP_HANDLER_H
#define MAP_HANDLER_H

#include "utility.h"

struct MapHandlerParams {
    MapHandlerParams() = default;
    float sensor_range;
    float cell_x;
    float cell_z;
    float max_length;
    float ceil_height;
    float margin_d;
};

struct CloudType {
    CloudType() = default;
    PointCloudPtr cloud_ptr;
    PointKdTreePtr kdtree_ptr;
};

class MapHandler {

public:
    MapHandler() = default;
    ~MapHandler() = default;

    static MapHandlerParams map_params_;

    void Init(const MapHandlerParams& params);
    void SetMapOrigin(const Point3D& robot_pos);

    void UpdateRobotPosition(const Point3D& odom_pos);

    /** Update global cloud grid with incoming clouds 
     * @param CloudInOut incoming cloud ptr and output valid in range points
    */
    void UpdateObsCloudGrid(const PointCloudPtr& obsCloudInOut);

    /** Extract Surrounding Free & Obs clouds 
     * @param SurroundCloudOut output surrounding cloud ptr
    */
    void GetSurroundObsCloud(const PointCloudPtr& obsCloudOut);

    /** Extract Surrounding Free & Obs clouds 
     * @param center the position of the grid that want to extract
     * @param type choose free or obstacle cloud for extraction
    */
    void GetCloudOnPosition(const Point3D& center, 
                            const PointCloudPtr& CloudOut);

    /**
     * Get neihbor cells center positions
     * @param neighbor_centers[out] neighbor centers stack
    */
    void GetNeighborCeilsCenters(PointStack& neighbor_centers);

    /**
     * Get neihbor cells center positions
     * @param occupancy_centers[out] occupanied cells center stack
    */
    void GetOccupancyCeilsCenters(PointStack& occupancy_centers);

    /**
     * Remove pointcloud from grid map
     * @param obsCloud obstacle cloud points that need to be removed
    */ 
    void RemoveObsCloudFromGrid(const PointCloudPtr& obsCloud);

    /**
     * @brief Reset Current Grip Map Clouds
     */
    void ResetGripMapCloud();

    /**
     * @brief main API function to check the collsion in points 
     */
    static bool IsPointCollideWithObs(const Point3D& p, const float& radius);

private:
    int neighbor_Lnum_, neighbor_Hnum_;
    Eigen::Vector3i robot_cell_sub_;
    bool is_init_ = false;

    inline void Expansion2D(const Eigen::Vector3i& csub, std::vector<Eigen::Vector3i>& subs, const int& n) {
        subs.clear();
        for (int ix=-n; ix<=n; ix++) {
            for (int iy=-n; iy<=n; iy++) {
                Eigen::Vector3i sub = csub;
                sub.x() += ix, sub.y() += iy;
                subs.push_back(sub); 
            }
        }
    }

    std::unordered_set<int> neighbor_obs_indices_;  // surrounding obs cloud grid indices stack

    static std::vector<int> global_visited_induces_;
    std::vector<int> util_obs_modified_list_;
    std::vector<int> util_remove_check_list_;

    static std::unique_ptr<grid_ns::Grid<CloudType>> world_obs_cloud_grid_;
 
};

#endif