/*
 * OMPL-based 3D Route Planner
 * Copyright (C) 2022 Fan Yang - All rights reserved
 * fanyang2@alumni.cmu.edu  
 */



#include "air_ompl_planner/ompl_planner.h"

/***************************************************************************************/

class MyStateChecker : public ob::StateValidityChecker
{
    public:
    MyStateChecker(const ob::SpaceInformationPtr &si) : ob::StateValidityChecker(si) {}

    virtual bool isValid(const ob::State *state) const
    {
        const auto *coord = state->as<ob::RealVectorStateSpace::StateType>();
        const Point3D check_p(coord->values[0], coord->values[1], coord->values[2]);
        // if ((check_p - AOMPLUtil::robot_pos).norm() < OMPLPlanner::ompl_params_.collision_r) return true; // clear current position
        if (MapHandler::IsPointCollideWithObs(check_p, OMPLPlanner::ompl_params_.collision_r)) {
            return false;
        }
        return true;
    }
};

ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
{
    ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
    obj->setCostThreshold(ob::Cost(std::numeric_limits<double>::max())); // return imediately after find a solution
    return obj;
};

void OMPLPlanner::Init(const OMPLPlannerParams& ompl_params) {
    ompl_params_ = ompl_params;
    // ompl value inits
    const int dim = 3;
    space_       = std::make_shared<ob::RealVectorStateSpace>(dim);
    coord_bound_ = std::make_shared<ob::RealVectorBounds>(dim);
    start_pos_   = std::make_shared<ob::ScopedState<>>(space_);
    goal_pos_    = std::make_shared<ob::ScopedState<>>(space_);

    coord_bound_->setLow(0, -MapHandler::map_params_.max_length/2.0f), coord_bound_->setLow(1, -MapHandler::map_params_.max_length/2.0f), coord_bound_->setLow(2, -MapHandler::map_params_.ceil_height);
    coord_bound_->setHigh(0, MapHandler::map_params_.max_length/2.0f), coord_bound_->setHigh(1, MapHandler::map_params_.max_length/2.0f), coord_bound_->setHigh(2, MapHandler::map_params_.ceil_height);

    space_->as<ob::RealVectorStateSpace>()->setBounds(*coord_bound_.get());
}

void OMPLPlanner::ResetExecutePath() {
    execute_path_.clear();
    execute_path_ori_.clear();
}

void OMPLPlanner::UpdaetExecutePath(const Point3D odom_pos, PointStack& path, Point3D& waypoint) {
    if (path.empty()) {
        ROS_WARN("OMPL: current path is empty");
        return;
    }
    const int N = path.size();
    Point3D cur_p = odom_pos;
    int idx = 0;
    std::vector<int> visited_list = std::vector<int>(path.size(), 0);
    visited_list[0] = 1;
    for (; idx<N; idx++) {
        cur_p = path[idx];
        if (visited_list[idx] == 1) continue;
        if (path.size() > 1 && (cur_p - odom_pos).norm() < ompl_params_.converage_d) {
            visited_list[idx] = 1;
            continue;
        }
        break;
    }
    if (idx > 0) {
        path.erase(path.begin(), path.begin() + idx - 1);
        path[0] = odom_pos;
    }
    const float norm_d = (cur_p - odom_pos).norm();
    if (path.size() > 2 && norm_d < ompl_params_.project_d) { // project  waypoint
        cur_p = odom_pos + (cur_p - odom_pos).normalize() * ompl_params_.project_d;
    }
    waypoint = cur_p;
}


float OMPLPlanner::PathCost(const PointStack& path) {
    if (path.empty()) return AOMPLUtil::kINF;
    Point3D last_p = path[0];
    const int N = path.size();
    float cost = 0.0;
    for (int i=1; i<N; i++) {
        auto cur_p = path[i];
        cost += (cur_p - last_p).norm();
        last_p = cur_p;
    }
    // TODO: add turning cost
    return cost;
}

bool OMPLPlanner::isValidSegment(const Point3D& p1, const Point3D& p2) {
    const float norm = (p2 - p1).norm();
    if (norm < ompl_params_.collision_r) {
        const Point3D center_p = (p1 + p2) / 2.0f;
        if (MapHandler::IsPointCollideWithObs(center_p, ompl_params_.collision_r)) {
            return false;
        }
        return true;
    }
    const Point3D norm_d = (p2 - p1).normalize();
    const float step_d = ompl_params_.collision_r / 1.5f;
    const int steps = (int)std::round(norm / step_d);
    for (int i=0; i<steps; i++) {
        const Point3D center_p = p1 + norm_d * i * step_d;
        if (MapHandler::IsPointCollideWithObs(center_p, ompl_params_.collision_r)) {
            return false;
        }
    }
    return true;
}

bool OMPLPlanner::isPathValid(const PointStack& path) {
    if (path.empty()) return false;
    const int N = path.size();
    Point3D last_p = path[0];
    for (int i=1; i<N; i++) {
        auto cur_p = path[i];
        if (!isValidSegment(last_p, cur_p)) {
            return false;
        }
        last_p = cur_p;
    }
    return true;
}

void OMPLPlanner::ExcuteOMPLPlanner(const Point3D& odom_pos, const Point3D& goal_pos, PointStack& path) {
    // set odom pose
    start_pos_->get()->as<ob::RealVectorStateSpace::StateType>()->values[0] =  odom_pos.x;
    start_pos_->get()->as<ob::RealVectorStateSpace::StateType>()->values[1] =  odom_pos.y;
    start_pos_->get()->as<ob::RealVectorStateSpace::StateType>()->values[2] =  odom_pos.z;
    // set goal pose
    goal_pos_->get()->as<ob::RealVectorStateSpace::StateType>()->values[0] =  goal_pos.x;
    goal_pos_->get()->as<ob::RealVectorStateSpace::StateType>()->values[1] =  goal_pos.y;
    goal_pos_->get()->as<ob::RealVectorStateSpace::StateType>()->values[2] =  goal_pos.z;
    
    // search space information
    ob::SpaceInformationPtr si(std::make_shared<ob::SpaceInformation>(space_));
    si->setStateValidityChecker(std::make_shared<MyStateChecker>(si));
    const double ratio = ompl_params_.collision_r / 1.5f / si->getMaximumExtent();
    si->setStateValidityCheckingResolution(ratio);

    si->setup();
    // problem definition
    ob::ProblemDefinitionPtr pdef(std::make_shared<ob::ProblemDefinition>(si));
    pdef->setStartAndGoalStates(*start_pos_.get(), *goal_pos_.get());
    pdef->setOptimizationObjective(getThresholdPathLengthObj(si));

    /* create different ompl planner instant */

    /* RRT* */
    auto planner(std::make_shared<og::RRTstar>(si));
    planner->setRange(ompl_params_.dstep);

    /* RRT-Connect */
    // auto planner(std::make_shared<og::RRTConnect>(si));
    // planner->setRange(ompl_params_.dstep);
    
    /* BITStar */
    // auto planner(std::make_shared<og::BITstar>(si));

    // configure the planner
    planner->setProblemDefinition(pdef);
    planner->setup();
    ob::PlannerStatus solved = planner->ob::Planner::solve(2.0); // max planning time
    if (solved == ob::PlannerStatus::EXACT_SOLUTION) {// if sucess
        // get the planned path
        ExtractPath(pdef, odom_pos, path);
    } else {
        path.clear();
    }
}

void OMPLPlanner::ExtractPath(const ob::ProblemDefinitionPtr pdef, const Point3D& odomPose, PointStack& plannedPath) {
    plannedPath.clear();
    // get the obtained path
    ob::PathPtr path = pdef->getSolutionPath();
    // print the path to screen
    path->print(std::cout);
    // convert to geometric path
    const auto *path_ = path.get()->as<og::PathGeometric>();
    // iterate over each position
    for(unsigned int i=0; i<path_->getStateCount(); ++i){
        // get state
        const ob::State* state = path_->getState(i);
        // get x coord of the robot
        const auto *coord = state->as<ob::RealVectorStateSpace::StateType>();
        Point3D poseMsg;
        poseMsg.x = coord->values[0], poseMsg.y = coord->values[1], poseMsg.z = coord->values[2];
        plannedPath.push_back(poseMsg);
    }
    std::cout<<"Path size: "<<plannedPath.size()<<std::endl;
}

void OMPLPlanner::PlanPath(const Point3D& odom_pos, const Point3D& goal_pos, Point3D& waypoint, PointStack& return_execute_path, PointStack& cur_path) {
    this->ExcuteOMPLPlanner(odom_pos, goal_pos, cur_path);
    if (!cur_path.empty()) {
        const float cur_cost = PathCost(cur_path);
        const float execute_cost = PathCost(execute_path_);
        ROS_INFO_STREAM("Current path cost: "<<cur_cost<<"; execute path cost: "<<execute_cost);
        if (!isPathValid(execute_path_ori_) || cur_cost < execute_cost * 0.85f) { // add momentum
            execute_path_ = cur_path;
            execute_path_ori_ = cur_path;
        }
    }
    UpdaetExecutePath(odom_pos, execute_path_, waypoint);
    return_execute_path = execute_path_;
}
