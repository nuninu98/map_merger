#include <map_merger/data_types/landmark.h>

using namespace std;

landmark::landmark(){

}

landmark::landmark(const Eigen::Matrix4d& pose, const pcl::PointCloud<pcl::PointXYZI>& points): pose(pose), points(points){

}

landmark::~landmark(){
    
}