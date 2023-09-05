#include <map_merger/api_class/map_merger.h>

MapMerger::MapMerger(){
   
}

MapMerger::~MapMerger(){
    
}

bool MapMerger::addMap(const string& map_path){
    vector<landmark> landmarks;
    if(!map_parser_.generateLandmark(map_path, landmarks)){
        return false;
    }

    initial_maps_.push_back(landmarks);
    return true;
}

bool MapMerger::match(size_t query_map_id, size_t target_map_id, const Eigen::Matrix4d& initial_guess){
    if(initial_maps_.size() <= query_map_id || initial_maps_.size() <= target_map_id){
        cout<<"Map ID exceeds the added maps"<<endl;
        return false;
    }
    gtsam::Values initial_landmark_poses;
    for(size_t i = 0; i < initial_maps_[target_map_id].size(); i++){
        initial_landmark_poses.insert(i, gtsam::Pose3(initial_maps_[target_map_id][i].pose));
    }

    for(size_t i = 0; i < initial_maps_[query_map_id].size(); i++){
        Eigen::Matrix4d query_pose = initial_guess * initial_maps_[query_map_id][i].pose;
        initial_landmark_poses.insert(i + initial_maps_[target_map_id].size(), gtsam::Pose3(query_pose));
    }
    return true;
}

void MapMerger::clear(){
    graph_.resize(0);
    isam_.clear();
    initial_maps_.clear();
}