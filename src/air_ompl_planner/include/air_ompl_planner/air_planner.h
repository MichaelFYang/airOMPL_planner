#ifndef OMPL_PLANNER_H
#define OMPL_PLANNER_H

#include "utility.h"
#include "map_handler.h"
#include "ompl_planner.h"
#include "scan_handler.h"
#include "planner_visualizer.h"


struct PlannerMasterParams {
    PlannerMasterParams() = default;
    float robot_dim; 
    float voxel_dim;
    float sensor_range;
    float viz_ratio;
    float ceil_height;
    float main_run_freq;
    bool  is_static_env;
    std::string world_frame;
};

class PlannerMaster {
public:
    PlannerMaster() = default;
    ~PlannerMaster() = default;

    void Init(); // ROS initialization
    void Loop(); // Main Loop Function

private:
    ros::NodeHandle nh;
    ros::Subscriber odom_sub_, scan_sub_, waypoint_sub_;
    ros::Publisher  goal_pub_, surround_obs_debug_;

    std_msgs::Float32 runtimer_, plan_timer_;

    Point3D robot_pos_, robot_heading_;
    geometry_msgs::PointStamped goal_waypoint_stamped_;

    bool is_cloud_init_, is_odom_init_;

    PointCloudPtr temp_cloud_ptr_;
    tf::TransformListener* tf_listener_;

    /* module objects */
    MapHandler map_handler_;
    DPVisualizer planner_viz_;
    OMPLPlanner ompl_planner_;
    ScanHandler scan_handler_;

    /* ROS Params */
    PlannerMasterParams master_params_;
    MapHandlerParams map_params_;
    OMPLPlannerParams ompl_params_;
    ScanHandlerParams scan_params_;
    
    /* Path Value */
    PointStack execute_path, current_path;

    /* planning internal params */
    bool is_goal_activity_;
    Point3D goal_pos_;

    void LoadROSParams();
    
    bool ProcessCloud(const sensor_msgs::PointCloud2ConstPtr& pc, const PointCloudPtr& cloudOut);

    void PublishWaypointMsg(const Point3D& goal_pos_);

    void ExtractDynamicObsFromScan(const PointCloudPtr& scanCloudIn, 
                                   const PointCloudPtr& obsCloudIn,
                                   const PointCloudPtr& dyObsCloudOut);

    /* Callback Functions */
    void OdomCallBack(const nav_msgs::OdometryConstPtr& msg);
    void ScanCallBack(const sensor_msgs::PointCloud2ConstPtr& pc);
    void WaypointCallBack(const geometry_msgs::PointStamped& route_goal);

    /* define inline functions */
    inline bool PreconditionCheck() {
        if (is_cloud_init_ && is_odom_init_) {
            return true;
        }
        return false;
    }
};

#endif