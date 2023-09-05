#include <ros/ros.h>
#include <map_merger/api_class/map_merger.h>
#include <pcl/filters/voxel_grid.h>
using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv, "test_map_merger");

    ros::NodeHandle nh;
    ros::Publisher pubMap = nh.advertise<sensor_msgs::PointCloud2>("parsed_map", 1);
    MapParser map_parser;
    vector<landmark> map_output;
    if(map_parser.generateLandmark("/home/nuninu98/map_merge/map0", map_output)){
        cout<<"SUCCESS"<<endl;
    }
    else{
        cout<<"FAILED"<<endl;
        return 0;
    }
    //==================Testing Map Parser===============================
    // pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::VoxelGrid<pcl::PointXYZI> voxel;
    // voxel.setLeafSize(1.0, 1.0, 1.0);
    // for(auto elem : map_output){
    //     pcl::PointCloud<pcl::PointXYZI> tf_cloud;
    //     pcl::transformPointCloud(elem.points, tf_cloud, elem.pose);
    //     *map_cloud += tf_cloud;
    //     voxel.setInputCloud(map_cloud);
    //     voxel.filter(*map_cloud);
    // }

    // sensor_msgs::PointCloud2 map_cloud_msg;
    // pcl::toROSMsg(*map_cloud, map_cloud_msg);
    // map_cloud_msg.header.stamp = ros::Time::now();
    // map_cloud_msg.header.frame_id = "map";
    // while(ros::ok()){
    //     cout<<"AA"<<endl;
    //     pubMap.publish(map_cloud_msg);
    //     ros::spinOnce();
    // }
    //========================================================================
    return 0;
}