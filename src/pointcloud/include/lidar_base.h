//
// Created by yel on 22-11-23.
//

#ifndef CPPTRICKS_LIDAR_BASE_H
#define CPPTRICKS_LIDAR_BASE_H

#include "cstdint"
#include "vector"
// raw point-cloud
#pragma pack(push)
#pragma pack(1)
typedef struct LidarPoint {
    float x;
    float y;
    float z;
    uint8_t intensity;
    uint8_t ring;
    uint16_t time_stamp;
} LidarPoint;

typedef struct LidarPointVcs {
    float x;
    float y;
    float z;
    uint8_t intensity;
    uint8_t ring;
} LidarPointVcs;

#pragma pack(pop)
typedef struct LidarPoints {
    std::vector<LidarPoint> points;
    uint64_t timestamp;
    bool density = true;
} LidarPoints;

typedef struct LidarPointsVcs {
    std::vector<LidarPointVcs> points;
    uint64_t timestamp;
    bool density = true;
} LidarPointsVcs;
#endif//CPPTRICKS_LIDAR_BASE_H
