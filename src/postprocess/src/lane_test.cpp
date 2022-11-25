//
// Created by yel on 22-10-18.
//
#include "lane_base.h"
using namespace std;

void LaneTest() {
    vector<LaneObject> obj;
    bool emp = obj.empty();
    obj.clear();

    int type = 1;
    bool flag = false;
    switch (type) {
        case 0:
        case 1:
        case 2:
            flag = true;
        default:
            break;
    }

    bool debug = true;
}