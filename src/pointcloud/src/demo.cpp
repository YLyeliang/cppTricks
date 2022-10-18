//
// Created by yel on 22-7-5.
//
#include "demo.h"

void write_pcd() {
    // Fill in the cloud data
    pcl::PointCloud<pcl::PointXYZ> cloud;
    // Fill in the cloud data
    cloud.width = 5;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.resize(cloud.width * cloud.height);
    for (auto &point: cloud) {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }
    for (int i = 0; i < cloud.points.size(); ++i) {
        std::cerr << cloud.points[i].x << cloud.points[i].y << cloud.points[i].z;
    }
    pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
}