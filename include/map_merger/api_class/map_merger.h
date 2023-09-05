#ifndef __MORIN_MAP_MERGER_H__
#define __MORIN_MAP_MERGER_H__

#include <map_merger/algorithm/map_parser.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/geometry/Pose3.h>
class MapMerger{
    private:
        MapParser map_parser_;
        gtsam::NonlinearFactorGraph graph_;
        gtsam::ISAM2 isam_;
        vector<vector<landmark>> initial_maps_;
        
    public:
        MapMerger();

        ~MapMerger();

        bool addMap(const string& map_path);

        bool match(size_t query_map_id, size_t target_map_id, const Eigen::Matrix4d& initial_guess);

        void clear();
};
#endif