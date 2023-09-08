#ifndef __MORIN_MAP_MERGER_H__
#define __MORIN_MAP_MERGER_H__

#include <map_merger/algorithm/map_parser.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <pcl/registration/gicp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

class MapMerger{
    private:
        double loop_detection_threshold_;
        MapParser map_parser_;
        gtsam::ISAM2 isam_;
        vector<landmark> initial_query_map_, initial_target_map_;
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp_;
        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_;
    public:
        MapMerger();

        ~MapMerger();

        bool addQueryMap(const string& map_path);

        bool addTargetMap(const string& map_path);

        bool match(const Eigen::Matrix4d& initial_guess, vector<landmark>& map_output, string map_folder = "");

        void clear();
};
#endif