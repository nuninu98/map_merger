#include <ros/ros.h>
#include <map_merger/api_class/map_merger.h>
using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv, "test_map_merger");

    ros::NodeHandle nh;
    ros::Publisher pub_merged_map = nh.advertise<sensor_msgs::PointCloud2>("merged_map", 1);
    Eigen::Matrix4d pq = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d pt = Eigen::Matrix4d::Identity();
    pq(0, 3) = -2.53;
    pq(1, 3) = 3.42;
    pq(0, 0) = cos(-0.74);
    pq(0, 1) = -sin(-0.74);
    pq(1, 0) = sin(-0.74);
    pq(1, 1) = cos(-0.74);

    pt(0, 3) = -12.84;
    pt(1, 3) = 14.06;
    pt(0, 0) = cos(-0.97);
    pt(0, 1) = -sin(-0.97);
    pt(1, 0) = sin(-0.97);
    pt(1, 1) = cos(-0.97);
    Eigen::Matrix4d init_guess = pt * pq.inverse();
    cout<<"\n"<<init_guess<<endl;

    MapMerger map_merger;
    vector<landmark> merged_map;
    map_merger.addQueryMap("/home/nuninu98/query_map");
    map_merger.addTargetMap("/home/nuninu98/target_map");
    sensor_msgs::PointCloud2 merged_map_ros;
    if(map_merger.match(init_guess, merged_map)){
        pcl::PointCloud<pcl::PointXYZI>::Ptr merged_map_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::VoxelGrid<pcl::PointXYZI> voxel;
        voxel.setLeafSize(0.5, 0.5, 0.5);
        for(auto elem : merged_map){
            pcl::PointCloud<pcl::PointXYZI> tf_cloud;
            pcl::transformPointCloud(elem.points, tf_cloud, elem.pose);
            *merged_map_cloud += tf_cloud;
            voxel.setInputCloud(merged_map_cloud);
            voxel.filter(*merged_map_cloud);
        }
        
        pcl::toROSMsg(*merged_map_cloud, merged_map_ros);
        merged_map_ros.header.stamp = ros::Time::now();
        merged_map_ros.header.frame_id = "map";
        // while(ros::ok()){
        //     pub_merged_map.publish(merged_map_ros);
        //     ros::spinOnce();
        // }
        
    }
    else{
        cout<<"FAILED"<<endl;
        return 0;
    }

    //==================Testing Map Parser===============================
    ros::Publisher pubQueryMap = nh.advertise<sensor_msgs::PointCloud2>("query_map", 1);
    ros::Publisher pubTargetMap = nh.advertise<sensor_msgs::PointCloud2>("target_map", 1);
    MapParser map_parser;
    vector<landmark> query_output, target_output;
    if(map_parser.generateLandmark("/home/nuninu98/query_map", query_output)){
        cout<<"SUCCESS"<<endl;
    }
    else{
        cout<<"FAILED"<<endl;
        return 0;
    }

    if(map_parser.generateLandmark("/home/nuninu98/target_map", target_output)){
        cout<<"SUCCESS"<<endl;
    }
    else{
        cout<<"FAILED"<<endl;
        return 0;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr query_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setLeafSize(0.2, 0.2, 0.2);
    for(auto elem : query_output){
        pcl::PointCloud<pcl::PointXYZI> tf_cloud;
        pcl::transformPointCloud(elem.points, tf_cloud, elem.pose);
        *query_cloud += tf_cloud;
        voxel.setInputCloud(query_cloud);
        voxel.filter(*query_cloud);
    }

    sensor_msgs::PointCloud2 query_cloud_msg;
    pcl::toROSMsg(*query_cloud, query_cloud_msg);
    query_cloud_msg.header.stamp = ros::Time::now();
    query_cloud_msg.header.frame_id = "map";

    pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    for(auto elem : target_output){
        pcl::PointCloud<pcl::PointXYZI> tf_cloud;
        pcl::transformPointCloud(elem.points, tf_cloud, elem.pose);
        *target_cloud += tf_cloud;
        voxel.setInputCloud(target_cloud);
        voxel.filter(*target_cloud);
    }

    sensor_msgs::PointCloud2 target_cloud_msg;
    pcl::toROSMsg(*target_cloud, target_cloud_msg);
    target_cloud_msg.header.stamp = ros::Time::now();
    target_cloud_msg.header.frame_id = "map";
    while(ros::ok()){
        pub_merged_map.publish(merged_map_ros);
        pubQueryMap.publish(query_cloud_msg);
        pubTargetMap.publish(target_cloud_msg);
        ros::spinOnce();
    }
    //========================================================================
    return 0;
}