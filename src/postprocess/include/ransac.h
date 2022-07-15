//
// Created by yel on 22-7-7.
//

#ifndef CPPTRICKS_RANSAC_H
#define CPPTRICKS_RANSAC_H
#include "Eigen/Dense"
#include "iostream"
#include "vector"

using namespace std;

typedef struct Point2D {
    float x;
    float y;
} Point2D;

class Ransac {
public:
private:
    vector<Point2D> inliers_;
    float residual_threshold_;
    int min_samples_;
    int max_trials_;
    int stop_score_;
    float stop_n_inliers;
    

};

#endif//CPPTRICKS_RANSAC_H
