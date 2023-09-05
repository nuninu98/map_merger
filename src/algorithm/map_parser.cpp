#include <map_merger/algorithm/map_parser.h>

using namespace std;

MapParser::MapParser(){

}

MapParser::~MapParser(){

}

bool MapParser::generateLandmark(const string& map_folder, vector<landmark>& map_output){
    experimental::filesystem::path sys(map_folder);
    if(!experimental::filesystem::is_directory(sys)){
        cout<<"Path: "<<map_folder<<" is not a directory!"<<endl;
        return false;
    }

    string pc_dir = map_folder + "/pointClouds";
    if(!experimental::filesystem::exists(pc_dir)){
        cout<<"pointClouds folder is not found in: "<<map_folder<<endl;
        return false;
    }

    ifstream pose_file(map_folder + "/poses.txt");
    if(!pose_file.good()){
        cout<<"poses.txt file is not found in: "<<map_folder<<endl;
        return false;
    }
    string sequence = "";
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 0.0; 

    map_output.clear();
    while(pose_file >> sequence >> x >> y>> z >> qx>> qy>> qz>> qw){
        Eigen::Quaterniond q(qw, qx, qy, qz);
        //cout<<sequence<<" "<<x<<" "<<y<<" "<<z<<" "<<qx<<qy<<qz<<" "<<qw<<endl;
        ifstream pointcloud_bin(map_folder + "/pointClouds/"+sequence+".bin", ios::in | ios::binary);
        landmark lm;
        while(pointcloud_bin){
            pcl::PointXYZI pt;
            pointcloud_bin.read((char*)&pt.x, sizeof(float));
            pointcloud_bin.read((char*)&pt.y, sizeof(float));
            pointcloud_bin.read((char*)&pt.z, sizeof(float));
            pointcloud_bin.read((char*)&pt.intensity, sizeof(float));
            lm.points.push_back(pt);
        }
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        pose.block<3, 3>(0, 0) = q.toRotationMatrix();
        pose(0, 3) = x;
        pose(1, 3) = y;
        pose(2, 3) = z;
        lm.pose = pose;
        map_output.push_back(lm);
        pointcloud_bin.close();
    }
    return true;
}
