#ifndef PLANNER_VISUALIZER_H
#define PLANNER_VISUALIZER_H

#include "utility.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

typedef visualization_msgs::Marker Marker;
typedef visualization_msgs::MarkerArray MarkerArray;


enum VizColor {
    RED     = 0,
    ORANGE  = 1,
    BLACK   = 2,
    YELLOW  = 3,
    BLUE    = 4,
    GREEN   = 5,
    EMERALD = 6,
    WHITE   = 7,
    MAGNA   = 8,
    PURPLE  = 9
};

class DPVisualizer {
private:
    ros::NodeHandle nh_;
    // Utility Cloud 
    PointCloudPtr point_cloud_ptr_;
    // rviz publisher 
    ros::Publisher viz_node_pub_, viz_path_pub_, viz_poly_pub_, viz_graph_pub_;
    ros::Publisher viz_contour_pub_, viz_map_pub_, viz_view_extend;

public:
    DPVisualizer() = default;
    ~DPVisualizer() = default;

    void Init(const ros::NodeHandle& nh);


    // True for non-attempts path
    void VizPath(const PointStack& execute_path, const PointStack& current_path);

    void VizMapGrids(const PointStack& neighbor_centers, 
                     const PointStack& occupancy_centers,
                     const float& ceil_length,
                     const float& ceil_height);


    void VizPoint3D(const Point3D& point, 
                    const std::string& ns,
                    const VizColor& color,
                    const float scale=1.0f,
                    const float alpha=0.9f);

    void VizPointCloud(const ros::Publisher& viz_pub, 
                       const PointCloudPtr& pc);

    static void SetMarker(const VizColor& color, 
                   const std::string& ns,
                   const float& scale, 
                   const float& alpha, 
                   Marker& scan_marker,
                   const float& scale_ratio=AOMPLUtil::kVizRatio);

    static void SetColor(const VizColor& color, const float& alpha, Marker& scan_marker);

};

#endif