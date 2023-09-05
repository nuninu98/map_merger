#ifndef __MORIN_MAP_MERGER_LANDMARK_H__
#define __MORIN_MAP_MERGER_LANDMARK_H__

#include <pcl/common/common.h>
#include <Eigen/Dense>
#include <Eigen/Core>

struct landmark{
    Eigen::Matrix4d pose;
    pcl::PointCloud<pcl::PointXYZI> points;

    landmark();

    landmark(const Eigen::Matrix4d& pose, const pcl::PointCloud<pcl::PointXYZI>& points);

    ~landmark();
};
#endif