#ifndef AOMPL_UTILITY_H
#define AOMPL_UTILITY_H

/* C++ Library */
#include <ros/ros.h>
#include <memory>
#include <string>
#include <time.h>
#include <queue>
#include <algorithm>
#include <unordered_set>
#include <boost/functional/hash.hpp>
/*Internal Library*/
#include "point_struct.h"
#include "grid.h"
/*ROS Library*/
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
/* PCL Library*/
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
/* OMPL Library */
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>


typedef pcl::KdTreeFLANN<PCLPoint>::Ptr PointKdTreePtr;
typedef std::vector<Point3D> PointStack;


class AOMPLUtil {
public:
    AOMPLUtil()  = default;
    ~AOMPLUtil() = default;
    /* Static Constant Values*/
    static const float  kEpsilon;
    static const float  kINF;
    static Point3D robot_pos;
    static Point3D robot_head;
    static float robot_dim;
    static float kLeafSize;
    static float kSensorRange;
    static float kVizRatio;
    static int kCollideN;
    static std::string worldFrameId;

    static PointCloudPtr surround_obs_cloud_;   // surround obstacle cloud
    static PointCloudPtr cur_new_cloud_;

    /* Utility Functions for OMPL-based Air OMPL Planner */

    static void FilterCloud(const PointCloudPtr& point_cloud, const float& leaf_size);

    template<typename T>
    static void FilterCloud(pcl::PointCloud<T>& point_cloud, const float& leaf_size) {
        const Eigen::Vector3d leaf(leaf_size, leaf_size, leaf_size);
        FilterCloud(point_cloud, leaf);
    }

    template<typename T>
    static void FilterCloud(pcl::PointCloud<T>& point_cloud, const Eigen::Vector3d& leaf_size) {
        // filter point cloud with constant leaf size
        using Cloud = pcl::PointCloud<T>;
        using CloudPtr = typename Cloud::Ptr;
        CloudPtr filter_cloud_ptr(new Cloud());
        *filter_cloud_ptr = point_cloud;
        pcl::VoxelGrid<T> vg;
        vg.setInputCloud(filter_cloud_ptr);
        vg.setLeafSize(leaf_size.x(), leaf_size.y(), leaf_size.z());
        vg.filter(point_cloud);
    }

    static void ClearKdTree(const PointKdTreePtr& kdTree_ptr);

    static void UpdateKdTreeWithCloud(const PointCloudPtr& cloud_ptr, const PointKdTreePtr& kdTree_ptr) {
        if (cloud_ptr->empty()) {
            ClearKdTree(kdTree_ptr);
        } else {
            kdTree_ptr->setInputCloud(cloud_ptr);
        }
    }

    static void RemoveOverlapCloud(const PointCloudPtr& cloudInOut,
                                   const PointCloudPtr& cloudRef,
                                   const bool& is_copy_cloud=false);
    
    static void TransformPoint3DFrame(const std::string& from_frame_id,
                                      const std::string& to_frame_id,
                                      const tf::TransformListener* tf_listener,
                                      Point3D& point);

    static bool IsSameFrameID(const std::string& cur_frame, const std::string& ref_frame);

    static void ResetCloudIntensity(const PointCloudPtr& cloudIn, const bool isHigh);

    static void ExtractNewObsPointCloud(const PointCloudPtr& cloudIn,
                                        const PointCloudPtr& cloudRefer,
                                        const PointCloudPtr& cloudNew);

    static geometry_msgs::Point Point3DToGeoMsgPoint(const Point3D& p);

    static void RemoveNanInfPoints(const PointCloudPtr& cloudInOut);

    static void TransformPCLFrame(const std::string& from_frame_id,
                                  const std::string& to_frame_id,
                                  const ros::Time& timestamp,
                                  const tf::TransformListener* tf_listener,
                                  const PointCloudPtr& cloudInOut);

    template<typename T>
    static void CropBoxCloud(pcl::PointCloud<T>& cloudInOut, const Point3D& center_p, const Point3D& crop_size) {
        CropBoxCloud(cloudInOut, cloudInOut, center_p, crop_size);
    }

    template<typename T>
    static void CropBoxCloud(const pcl::PointCloud<T>& cloudIn, pcl::PointCloud<T>& cloudOut, const Point3D& center_p, const Point3D& crop_size) {
        using Cloud = pcl::PointCloud<T>;
        using CloudPtr = typename Cloud::Ptr;
        pcl::CropBox<T> boxFilter;
        CloudPtr copy_cloud_ptr(new Cloud());
        *copy_cloud_ptr = cloudIn;
        Eigen::Vector4f min_vec(center_p.x - crop_size.x,
                                center_p.y - crop_size.y,
                                center_p.z - crop_size.z, 1.0f);
        Eigen::Vector4f max_vec(center_p.x + crop_size.x,
                                center_p.y + crop_size.y,
                                center_p.z + crop_size.z, 1.0f);
        boxFilter.setMin(min_vec), boxFilter.setMax(max_vec);
        boxFilter.setInputCloud(copy_cloud_ptr);
        boxFilter.filter(cloudOut);
    }
};


#endif