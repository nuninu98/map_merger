#include <map_merger/api_class/map_merger.h>

MapMerger::MapMerger(): loop_detection_threshold_(100.0){
    voxel_grid_.setLeafSize(1.0, 1.0, 1.0);
}

MapMerger::~MapMerger(){
    
}

bool MapMerger::addQueryMap(const string& map_path){
    if(!map_parser_.generateLandmark(map_path, initial_query_map_)){
        return false;
    }
    return true;
}

bool MapMerger::addTargetMap(const string& map_path){
    if(!map_parser_.generateLandmark(map_path, initial_target_map_)){
        return false;
    }
    return true;
}

bool MapMerger::match(const Eigen::Matrix4d& initial_guess, vector<landmark>& map_output, string map_folder){
    gtsam::Values initial_landmark_poses;
    if(initial_query_map_.empty() || initial_target_map_.empty()){
        cout<<"Please add query and target first"<<endl;
        return false;
    }
    // Adding target landmarks
    for(size_t i = 0; i < initial_target_map_.size(); i++){
        initial_landmark_poses.insert(i, gtsam::Pose3(initial_target_map_[i].pose));
    }
    
    // Adding query landmarks
    for(size_t i = 0; i < initial_query_map_.size(); i++){
        Eigen::Matrix4d query_pose_in_target = initial_guess* initial_query_map_[i].pose ;//initial_query_map_[i].pose * initial_guess.inverse();
        initial_landmark_poses.insert(i + initial_target_map_.size(), gtsam::Pose3(query_pose_in_target));
    }

    // Noise factors
    gtsam::NonlinearFactorGraph graph;
    gtsam::noiseModel::Diagonal::shared_ptr prior_noise =
        gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(1.0e-6), gtsam::Vector3::Constant(1.0e-6)).finished());
    
    gtsam::noiseModel::Diagonal::shared_ptr intra_noise =
        gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(1.0e-1), gtsam::Vector3::Constant(1.0e-1)).finished());

    gtsam::noiseModel::Diagonal::shared_ptr inter_noise =
        gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(1.0e-4), gtsam::Vector3::Constant(1.0e-4)).finished());

    graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, initial_landmark_poses.at<gtsam::Pose3>(0), prior_noise));
    // Setting intra constraint between target map landmarks
    for(size_t id = 0; id < initial_target_map_.size() - 1; id++){
        Eigen::Matrix4d pose_gap = initial_target_map_[id].pose.inverse() * initial_target_map_[id + 1].pose;   
        graph.add(gtsam::BetweenFactor<gtsam::Pose3>(id, id + 1, gtsam::Pose3(pose_gap), intra_noise));
    }

    //Setting intra constraint between query map landmarks
    for(size_t id = 0; id < initial_query_map_.size()- 1; id++){
        size_t graph_id = id + initial_target_map_.size();
        Eigen::Matrix4d pose_gap = initial_query_map_[id].pose.inverse() * initial_query_map_[id + 1].pose;
        graph.add(gtsam::BetweenFactor<gtsam::Pose3>(graph_id, graph_id + 1, gtsam::Pose3(pose_gap), intra_noise));
    }

    //============Search for inter constraint================
    pcl::KdTreeFLANN<pcl::PointXYZ> target_landmark_kdtree;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_node_poses(new pcl::PointCloud<pcl::PointXYZ>);
    for(const auto& elem : initial_target_map_){
        pcl::PointXYZ pose(elem.pose(0, 3), elem.pose(1, 3), elem.pose(2, 3));
        target_node_poses->push_back(pose);
    }
    target_landmark_kdtree.setInputCloud(target_node_poses);
    cout<<"INTER LOOP FINDING"<<endl;
    for(size_t i = 0; i < initial_query_map_.size(); i++){
        cout<<"Processing NODE "<<i<<"/"<<initial_query_map_.size()<<endl;

        Eigen::Matrix4d pose_in_target = initial_guess * initial_query_map_[i].pose;
        pcl::PointXYZ query_pose(pose_in_target(0, 3), pose_in_target(1, 3), pose_in_target(2, 3));
        pcl::Indices ids;
        vector<float> dists;
        target_landmark_kdtree.nearestKSearch(query_pose, 1, ids, dists);
        if(dists[0] > loop_detection_threshold_){
            continue;
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(initial_target_map_[ids[0]].points));
        pcl::PointCloud<pcl::PointXYZI>::Ptr query_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(initial_query_map_[i].points));
        voxel_grid_.setInputCloud(query_cloud_ptr);
        voxel_grid_.filter(*query_cloud_ptr);
        voxel_grid_.setInputCloud(target_cloud_ptr);
        voxel_grid_.filter(*target_cloud_ptr);
        gicp_.setInputCloud(query_cloud_ptr);
        gicp_.setInputTarget(target_cloud_ptr);
        pcl::PointCloud<pcl::PointXYZI> aligned;
        Eigen::Matrix4d gicp_initial_guess = initial_target_map_[ids[0]].pose.inverse()* initial_guess * initial_query_map_[i].pose;
        //initial_target_map_[ids[0]].pose.inverse() * (initial_query_map_[i].pose * initial_guess.inverse());
        gicp_.align(aligned, gicp_initial_guess.cast<float>());

        size_t graph_id = initial_target_map_.size() + i;
        Eigen::Matrix4d relative_pose = gicp_.getFinalTransformation().cast<double>();
        graph.add(gtsam::BetweenFactor<gtsam::Pose3>(ids[0], graph_id, gtsam::Pose3(relative_pose), inter_noise));
    } 
    //=======================================================

    //==============Solve Optimization=======================
    isam_.update(graph, initial_landmark_poses);
    gtsam::Values estimated = isam_.calculateEstimate();
    map_output.clear();

    for(size_t i = 0; i < initial_target_map_.size(); i++){
        gtsam::Pose3 pose = estimated.at<gtsam::Pose3>(i);
        landmark lm(pose.matrix(), initial_target_map_[i].points);
        map_output.push_back(lm);
    }

    for(size_t i = 0; i < initial_query_map_.size(); i++){
        gtsam::Pose3 pose = estimated.at<gtsam::Pose3>(i + initial_target_map_.size());
        landmark lm(pose.matrix(), initial_query_map_[i].points);
        map_output.push_back(lm);
    }

    if(map_folder != ""){
        cout<<"Saving map folder in: "<<map_folder<<endl;
        experimental::filesystem::create_directories(map_folder+"/pointClouds");
        ofstream pose_txt(map_folder + "/poses.txt");
        for(size_t i = 0; i < estimated.size(); i++){
            gtsam::Pose3 lm_pose = estimated.at<gtsam::Pose3>(i);
            Eigen::Matrix4d lm_pose_se3 = lm_pose.matrix().cast<double>();
            Eigen::Quaterniond q(lm_pose_se3.block<3, 3>(0, 0));
            pose_txt<<i<<" "<<lm_pose_se3(0, 3)<<" "<<lm_pose_se3(1, 3)<<" "<<lm_pose_se3(2, 3)<<" "
            <<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<endl;
        }
        pose_txt.close();

        for(size_t i = 0; i < initial_target_map_.size(); i++){
            // gtsam::Pose3 pose = estimated.at<gtsam::Pose3>(i);
            // landmark lm(pose.matrix(), initial_target_map_[i].points);
            // map_output.push_back(lm);
            ofstream bin_file(map_folder + "/pointClouds/"+to_string(i)+".bin", ios::out|ios::binary);
            for(size_t j = 0; j < initial_target_map_[i].points.size(); j++){
                bin_file.write((char*)&initial_target_map_[i].points[j].x, sizeof(float));
                bin_file.write((char*)&initial_target_map_[i].points[j].y, sizeof(float));
                bin_file.write((char*)&initial_target_map_[i].points[j].z, sizeof(float));
                bin_file.write((char*)&initial_target_map_[i].points[j].intensity, sizeof(float));
            }
            bin_file.close(); 
        }

        for(size_t i = 0; i < initial_query_map_.size(); i++){
            // gtsam::Pose3 pose = estimated.at<gtsam::Pose3>(i + initial_target_map_.size());
            // landmark lm(pose.matrix(), initial_query_map_[i].points);
            // map_output.push_back(lm);
            ofstream bin_file(map_folder + "/pointClouds/"+to_string(i + initial_target_map_.size())+".bin", ios::out|ios::binary);
            for(size_t j = 0; j < initial_query_map_[i].points.size(); j++){
                bin_file.write((char*)&initial_query_map_[i].points[j].x, sizeof(float));
                bin_file.write((char*)&initial_query_map_[i].points[j].y, sizeof(float));
                bin_file.write((char*)&initial_query_map_[i].points[j].z, sizeof(float));
                bin_file.write((char*)&initial_query_map_[i].points[j].intensity, sizeof(float));
            }
            bin_file.close(); 
        }
    }
    //=======================================================
    
    return true;
}

void MapMerger::clear(){
    isam_.clear();
    initial_query_map_.clear();
    initial_target_map_.clear();
}