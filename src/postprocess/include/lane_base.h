//
// Created by yel on 22-10-18.
//

#ifndef CPPTRICKS_LANEBASE_H
#define CPPTRICKS_LANEBASE_H
#include "stdlib.h"
#include "string"
#include "vector"
// traffic
enum TrafficLineType {
    TYPE_UNDECIDED = 0,
    SINGLE_SOLID = 1,
    DOUBLE_SOLID = 2,
    SINGLE_LONG_DASHED = 3,
    SINGLE_SHORT_DASHED = 4,
    DOUBLE_DASHED = 5,
    LEFT_SOLID_RIGHT_DASHED = 6,
    LEFT_DASHED_RIGHT_SOLID = 7,
    DECELERATION_VERTICAL = 8,
    DECELERATION_HORIZONTAL = 9,
    ROAD_CURB = 10,
    CROSS_WALK = 11,
    CONE_LANE = 12,
    STOP_LINE = 13,
    DECELERATION_GIVEWAY = 14,
    RAILING_LINE = 15,
    REVERSIBLE_LANE = 16,
    LANE_CENTER = 17,
    SHADE_AREA = 18
};

enum TrafficColor {
    COLOR_UNDECIDED = 0,
    WHITE = 1,
    YELLOW = 2,
    GREEN = 3,
    BLUE = 4,
    RED = 5,
};

enum TrafficRole {
    ROLE_UNDECIDED = 0,
    HOST_LEFT = 1,
    HOST_RIGHT = 2,
    LEFT_LEFT = 3,
    LEFT_RIGHT = 4,
    RIGHT_LEFT = 5,
    RIGHT_RIGHT = 6,
    LEFT_LEFT_LEFT = 7,
    LEFT_LEFT_RIGHT = 8,
    RIGHT_RIGHT_LEFT = 9,
    RIGHT_RIGHT_RIGHT = 10,
    HOST_CENTER = 11,
    LEFT_CENTER = 12,
    RIGHT_CENTER = 13,
    LEFT_LEFT_CENTER = 14,
    RIGHT_RIGHT_CENTER = 15,
    LEFT_BOUNDARY = 16,
    RIGHT_BOUNDARY = 17
};

typedef struct StateType {
    double c0;
    double c1;
    double c2;
    double c3;
} StateType;

enum TrafficSource {
    SRC_UNKNOWN = 0,
    SRC_NEW = 1,
    SRC_MEASURED = 2,
    SRC_PREDICTED = 3,
};

enum CurveType {
    CurveType_Unknown = 0,
    CurveType_ADAS = 2,
    CurveType_Cubic = 4,
    CurveType_Bezier = 8,
};

enum ExtraKeyPointType {
    KEYPOINT_UNDECIDED = 0,
    DIVISION = 1,
    MERGE = 2,
    VOID_DASH,
    DASH_VOID,
};

enum ProcssModules {
    J3_Module = 0,
    Me_Module = 1,
    ZLANE_Module = 2,
    ZLANE_DEBUG_2D = 3,
    J3_OUT_ZLANE = 4,
    ME_OUT_ZLANE = 5
};

enum CameraID {
    CAMERA_FRONT_WIDE = 0,
    CAMERA_FRONT_LONG = 1,
    CAMERA_REAR = 2,
    CAMERA_LEFT_FRONT = 3,
    CAMERA_LEFT_REAR = 4,
    CAMERA_RIGHT_FRONT = 5,
    CAMERA_RIGHT_REAR = 6,
    CAMERA_UNKNOWN,
};
// me
enum MeLineType {
    ME_DASHED = 0,
    ME_SOLID = 1,
    ME_UNDECIDED = 2,
    ME_DLM = 3,
    ME_BOTTS = 4,
    ME_DECEL = 5,
    ME_INVALID = 6
};

enum MeRoadType {
    ME_ROADEDGE = 0,
    ME_CURB = 1,
    ME_BARRIER = 2,
    ME_CONESPOLES = 3,
    ME_PARKEDCARS = 4,
};

enum MeColor {
    ME_UNKNOWN = 0,
    ME_WHITE = 1,
    ME_YELLOW = 2,
    ME_BLUE = 3,
};

enum MeRole {
    ME_ROLE_UNDECIDED = 0,
    ME_HOST_LEFT = 1,
    ME_HOST_RIGHT = 2,
    ME_LEFT_LEFT = 3,
    ME_LEFT_RIGHT = 4,
    ME_RIGHT_LEFT = 5,
    ME_RIGHT_RIGHT = 6,
};

// j3
enum J3LineType {
    J3_Line_Unknown = 0,
    J3_LaneLine = 2,
    J3_Center = 8,
    J3_Fence = 64,
};
enum J3LineSource {
    //cross status
    J3_SRC_CROSSSING = 1,
    //barrier properties
    J3_SRC_OBSTACLE_BLOCKED = 4,
    J3_SRC_HISTORY_BASED = 16,
    J3_SRC_EXTRAPOLATION = 32,
    //tracking state
    J3_SRC_UNKNOWN = 512,
    J3_SRC_NEW = 1024,
    J3_SRC_MEASURED = 2048,
    J3_SRC_PREDICTED = 4096,
    //con position
    J3_SRC_OBSTACLE_LEFT = 8192,
    J3_SRC_OBSTACLE_RIGHT = 16384,
    //road edge
    J3_EDGE_CURB = 32768,
    J3_EDGE_GUARDRAIL = 65536,
    J3_EDGE_CONCRET_BARRIER = 131072,
    J3_EDGE_WALL = 262144,
    J3_EDGE_CANOPY = 524288,
    J3_EDGE_CONE = 1048576,
    J3_EDGE_OTHER = 2097152,
    //has stop line
    J3_SRC_STOPLINE = 33554432,
    //unpalledl and pallel
    J3_SRC_SEPARATE = 67108864
};
enum J3LaneLineType {
    J3_Laneline_Unknown = 0,
    J3_SolidLine = 2,
    J3_DashedLine = 4,
    J3_ShortDashedLine = 8,
    J3_DoubleSolidLine = 16,
    J3_DoubleDashedLine = 32,
    J3_LeftSolidRightDashed = 64,
    J3_RightSolidLeftDashed = 128,
    J3_ShadeArea = 256,
    J3_LaneVirtualMarking = 512,
    J3_IntersectionVirualMarking = 1024,
    J3_CurbVirtualMarking = 2048,
    J3_UnclosedRoad = 4096,
    J3_RoadVirtualLine = 8192,
    J3_DecelerationSolidLine = 16384,
    J3_DecelerationDahLine = 32768,

};

enum J3Color {
    J3_Color_Unknown = 0,
    J3_Color_WHITE = 2,
    J3_Color_YELLOW = 4,
    J3_Color_ORANGE = 8,
    J3_Color_BLUE = 16,
    J3_Color_GREEN = 32,
    J3_Color_GRAY = 64,
    J3_Color_LETF_GRAY_RIGHT_YELLOW = 128,
    J3_Color_LETF_YELLOW_RIGHT_WHITE = 256,
};

enum J3Role {
    J3_POS_LEFT = 0,
    J3_POS_RIGHT = 2,
    J3_POS_LEFT_LEFT = 4,
    J3_POS_RIGHT_RIGHT = 8,
    J3_POS_LEFT_OUT_SIDE = 16,
    J3_POS_RIGHT_OUT_SIDE = 32,
    J3_POS_LEFT_LEFT_LEFT = 64,
    J3_POS_RIGHT_RIGHT_RIGHT = 128,
};

enum J3CurveType {
    J3_CurveType_Unknown = 0,
    J3_CurveType_ADAS = 2,
    J3_CurveType_Cubic = 4,
    J3_CurveType_Bezier = 8,
};

enum J3ExtraKeyPointType {
    J3_POINT_TYPE_UNKNOWN = 1,
    J3_POINT_TYPE_DIVISION = 2,
    J3_POINT_TYPE_MERGE = 4,
};

typedef struct InternalConfig {
    std::string image_front_wide_topic;
    std::string j3_lane_topic;
    std::string me_lane_topic;
    std::string traffic_lane_topic;
    std::string tf_relative_topic;
    std::string calib_params_topic;
    std::string traffic_lane_zeekr_topic;
    std::string chassis_report_topic;
    std::string det_model_x86;
    std::string freespace_det_model_x86;
    std::string vehicle_config_path_x86;

    std::string det_model_arh64;
    std::string freespace_det_model_arh64;
    std::string vehicle_config_path_arh64;

    std::string front_wide_intrinsics_path;
    std::string front_wide_extrinsics_path;
    int module;
    int l_num;
    int vehicle_id;
    int min_line_len;
    int tracker_type;
    float vehicle_speed_scl;
    int freespace_on;
} InternalConfig;

typedef struct Point2D {
    double x;
    double y;
} Point2D;

typedef struct Point3D {
    double x;
    double y;
    double z;
} Point3D;

typedef struct Point2F {
    float x;
    float y;
} Point2F;

typedef struct Point3F {
    float x;
    float y;
    float z;
} Point3F;

typedef struct RowCluster {
    std::vector<Point2F> cluster;
    Point2F center;
    bool sampled = false;
    bool in_roi = false;

    const RowCluster &operator=(const RowCluster &rc) {
        center = rc.center;
        sampled = rc.sampled;
        in_roi = rc.in_roi;
        cluster.assign(rc.cluster.begin(), rc.cluster.end());
        return *this;
    }
} RowCluster;

typedef struct RowCluster3d {
    std::vector<Point3F> cluster;
    Point3F center;
} RowCluster3d;

typedef struct ExtraKeyPoint {
    Point2F point2d;
    Point3F point3d;
    ExtraKeyPointType type;
} ExtraKeyPoint;

typedef struct LaneObject {
    bool valid = true;
    int id;
    float conf;
    int start2d;
    int end2d;
    TrafficColor color;
    TrafficLineType type;
    TrafficRole role;
    double c0;
    double c1;
    double c2;
    double c3;
    double start;
    double end;
    float slope;
    float nearest_dist_avg;
    std::vector<RowCluster> rcs;
    std::vector<RowCluster3d> rcs3d;
    bool in_roi = false;
    bool sampled = false;
    int roi_num_point = 0;
    std::vector<double> interpolate_x;
    std::vector<bool> visibility_mat;
} LaneObject;

// lane detector output, to be defined
typedef struct LaneInfo {
    std::vector<int> roi_lanes_idx;
    std::vector<LaneObject> lanes;
    std::vector<ExtraKeyPoint> keyPoints;
    uint64_t timestamp;
    LaneInfo() {
    }

    LaneInfo(const LaneInfo &li) {
        timestamp = li.timestamp;
        lanes.assign(li.lanes.begin(), li.lanes.end());
        roi_lanes_idx.assign(li.roi_lanes_idx.begin(), li.roi_lanes_idx.end());
        keyPoints.assign(li.keyPoints.begin(), li.keyPoints.end());
    }

    LaneInfo &operator=(LaneInfo &li) {
        timestamp = li.timestamp;
        lanes.assign(li.lanes.begin(), li.lanes.end());
        roi_lanes_idx.assign(li.roi_lanes_idx.begin(), li.roi_lanes_idx.end());
        keyPoints.assign(li.keyPoints.begin(), li.keyPoints.end());
        return *this;
    }
} LaneInfo;

typedef struct LineCutOffInfo {
    size_t num_points;
    double std;
    double mse_err, mean_err;// fit err
    size_t top, bottom;
} LineCutOffInfo;

typedef struct LineFitInfo {
    double k, b;             // fit paras
    double mse_err, mean_err;// fit err
    double std;
    size_t top, bottom;
    LineCutOffInfo cut_line;
    bool is_straight_lane;
} LineFitInfo;

typedef struct LaneFitInfo {
    LineFitInfo right_line;
    LineFitInfo left_line;
} LaneFitInfo;

typedef struct Point3DWeight {
    Point3F point;
    float weight;
} Point3DWeight;

#endif//CPPTRICKS_LANEBASE_H
