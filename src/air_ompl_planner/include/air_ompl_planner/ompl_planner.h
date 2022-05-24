#ifndef OMPL_AOMPL_PLANNER_H
#define OMPL_AOMPL_PLANNER_H

#include "utility.h"
#include "map_handler.h"
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

struct OMPLPlannerParams {
    OMPLPlannerParams() = default;
    float dstep;
    float collision_r;
    float converage_d;
    float project_d;
};


class OMPLPlanner {
public:
    OMPLPlanner()  = default;
    ~OMPLPlanner() = default;

    static OMPLPlannerParams ompl_params_;

    void Init(const OMPLPlannerParams& ompl_params); // ROS initialization

    void ResetExecutePath();

    /** Plan Path from current robot position to goal position 
     * @param odom_pos current robot position
     * @param goal_pos goal position
     * @param waypoint[return] next navigation guidence point
     * @param execute_path[return] the path returned by planner
     * @param cur_path[return] the current searched path
    */
    void PlanPath(const Point3D& odom_pos, const Point3D& goal_pos, Point3D& waypoint, PointStack& execute_path, PointStack& cur_path);

private:
    // std::vector<int> visited_list_;
    PointStack execute_path_, execute_path_ori_;

    // start and goal position
    ob::ScopedStatePtr start_pos_;
    ob::ScopedStatePtr goal_pos_;
    /// search space
    ob::StateSpacePtr space_;

    std::shared_ptr<ob::RealVectorBounds> coord_bound_;

    void ExcuteOMPLPlanner(const Point3D& odom_pos, const Point3D& goal_pos, PointStack& path);

    void ExtractPath(const ob::ProblemDefinitionPtr pdef, const Point3D& odomPose, PointStack& plannedPath);

    void UpdaetExecutePath(const Point3D odom_pos, PointStack& path, Point3D& waypoint);

    float PathCost(const PointStack& path);

    bool isValidSegment(const Point3D& p1, const Point3D& p2);

    bool isPathValid(const PointStack& path);
};




#endif