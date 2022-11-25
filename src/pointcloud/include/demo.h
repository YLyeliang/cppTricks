//
// Created by yel on 22-7-5.
//

#ifndef CPPTRICKS_DEMO_H
#define CPPTRICKS_DEMO_H
#include <iostream>
#include <pcl/common/file_io.h>// for getFilenameWithoutExtension
#include <pcl/console/parse.h>
// #include <pcl/features/range_image_border_extractor.h>
#include <pcl/io/pcd_io.h>
// #include <pcl/keypoints/narf_keypoint.h>
// #include <pcl/range_image/range_image.h>

#include <pcl/point_types.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/visualization/range_image_visualizer.h>
#include "pcl/filters/voxel_grid.h"

void write_pcd();

void VoxelDownSample(pcl::PointCloud<pcl::PointXYZI> &src, pcl::PointCloud<pcl::PointXYZI> &result) {
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
    // voxel_grid.setInputCloud(src);
    // voxel_grid.setDownsampleAllData(true);
    // voxel_grid.setLeafSize(10, 10, 10);
    // voxel_grid.filter(result);
}

#endif//CPPTRICKS_DEMO_H
