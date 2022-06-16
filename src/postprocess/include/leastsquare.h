//
// Created by 叶亮 on 2022/5/29.
//

#ifndef CPPTRICKS_LEASTSQUARE_H
#define CPPTRICKS_LEASTSQUARE_H

#include "opencv2/opencv.hpp"
#include <Eigen/Dense>
#include <vector>
using namespace std;

Eigen::VectorXf LSFit(vector<float> &X, vector<float> &Y, uint8_t orders);

Eigen::VectorXf norm_test();
void ransacFitCV(const std::vector<cv::Point2f> &points,
                 cv::Vec4f &line,
                 int iterations = 1000,
                 double sigma = 1.,
                 double k_min = -7.,
                 double k_max = 7.);

#endif//CPPTRICKS_LEASTSQUARE_H
