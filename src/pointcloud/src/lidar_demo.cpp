//
// Created by yel on 22-11-23.
//
#include "lidar_demo.h"
#include "fstream"
#include "lidar_base.h"
#include "string"
#include <sstream>

using namespace std;

void LidarUTDemo() {
    std::vector<LidarPoint> lidar_pts_;
    const string front_file = "/home/yel/zdrive2/py_project/misctools/debug/rosbag2_2022_10_08-21_15_05_0/lidar_stitch_pcd/1665234906004.txt";
    ifstream inf_front;
    string s;
    inf_front.open(front_file);

    float x;
    float y;
    float z;
    uint8_t intensity;
    uint8_t ring;
    uint16_t time_stamp;

    while (getline(inf_front, s)) {
        stringstream arr(s);
        if (s == "")
            break;
        arr >> x >> y >> z >> intensity >> ring >> time_stamp;
        lidar_pts_.emplace_back(LidarPoint{x, y, z, intensity, ring, time_stamp});
    }
    int debug = 1;
}