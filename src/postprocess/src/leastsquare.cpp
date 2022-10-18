// created by yel on 2022/5/29
#include "leastsquare.h"
#include <Eigen/Dense>
#include <iostream>
using namespace std;

/**
 * @brief Fit polynomial using Least Square Method.
 *
 * @param X X-axis coordinate vector of sample data.
 * @param Y Y-axis coordinate vector of sample data.
 * @param orders Fitting order which should be larger than zero.
 * @return Eigen::VectorXf Coefficients vector of fitted polynomial.
 */
Eigen::VectorXf LSFit(vector<float> &X, vector<float> &Y, uint8_t orders) {
    // input verification
    if (X.size() < 2 || Y.size() < 2 || X.size() != Y.size() || orders < 1)
        exit(-1);
    // map sample data from STL vector to eigen vector
    Eigen::Map<Eigen::VectorXf> sampleX(X.data(), X.size());
    Eigen::Map<Eigen::VectorXf> sampleY(Y.data(), Y.size());

    // Vandermonde matrix of X-axis coordinate vector of sample data
    Eigen::MatrixXf mtxVndermonde(X.size(), orders + 1);
    Eigen::VectorXf colVandermonde = sampleX;// Vandermonde column

    // construct Vandermonde matrix column by column
    for (size_t i = 0; i < orders + 1; ++i) {
        // first col
        if (i == 0) {
            mtxVndermonde.col(0) = Eigen::VectorXf::Constant(X.size(), 1, 1);
            continue;
        }
        if (1 == i) {
            mtxVndermonde.col(1) = colVandermonde;
            continue;
        }
        colVandermonde = colVandermonde.array() * sampleX.array();// order+1, x -> x^2 -> x^3
        mtxVndermonde.col(i) = colVandermonde;
    }

    // calculate coeff vector of fitted polynomial
    Eigen::VectorXf coeff = (mtxVndermonde.transpose() * mtxVndermonde).inverse() * mtxVndermonde.transpose() * sampleY;
    return coeff;
}

// least square
void svdLS() {
    Eigen::MatrixXf A = Eigen::MatrixXf::Random(3, 2);
    std::cout << "Here is the matrix A:\n"
              << A << std::endl;
    Eigen::VectorXf b = Eigen::VectorXf::Random(3);
    std::cout << "Here is the right hand side b:\n"
              << b << std::endl;
    std::cout << "The least-squares solution is:\n"
              << A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b) << std::endl;
}

void QrLS() {
    Eigen::Matrix<double, 4, 1> coeff;
    vector<double> x_arr{20.043245315551758, 22.528451919555664, 25.27334976196289, 28.585643768310547, 32.69932174682617, 37.923240661621094, 44.82033157348633, 53.95246887207031, 68.51797485351562, 115.67473602294922, 130.2449951171875, 149.37713623046875, 175.43032836914062};
    vector<double> y_arr{-16.89411735534668, -17.106796264648438, -17.431556701660156, -17.869461059570312, -18.46766471862793, -18.6855525970459, -18.749042510986328, -19.42156982421875, -20.659299850463867, -25.28095054626465, -25.842105865478516, -26.950170516967773, -28.683124542236328};
    vector<double> pos_w{1,
                         1,
                         1,
                         1,
                         1,
                         1,
                         1,
                         1,
                         1,
                         1,
                         0.75,
                         0.75,
                         0.75,
                         0.75};
    size_t n = x_arr.size();
    int order = 3;
    // create data matrix
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A(n, order + 1);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> W(n, n);
    Eigen::Matrix<double, Eigen::Dynamic, 1> y(n);
    Eigen::Matrix<double, Eigen::Dynamic, 1> result;

    for (size_t i = 0; i < n; ++i) {
        for (size_t k = 0; k < n; ++k) {
            W(i, k) = 0;
        }
        W(i, i) = pos_w[i];
        for (int j = 0; j <= order; ++j) {
            A(i, j) = static_cast<double>(std::pow(x_arr[i], j));
        }
        y(i) = y_arr[i];
    }
    A = W * A;
    y = W * y;
    // solve linear least squares
    result = A.householderQr().solve(y);
    assert(result.size() == order + 1);

    for (int j = 0; j <= 3; ++j) {
        coeff(j) = (j <= order) ? result(j) : static_cast<double>(0);
    }
    int debug = 1;
}


//Eigen::VectorXf ransacFit(const vector<float> &X, const vector<float> &Y, uint8_t orders) {
//}
void ransacFitCV(const std::vector<cv::Point2f> &points,
                 cv::Vec4f &line,
                 int iterations,
                 double sigma,
                 double k_min,
                 double k_max) {
    unsigned int n = points.size();

    if (n < 2) {
        return;
    }

    cv::RNG rng;
    double bestScore = -1.;
    for (int k = 0; k < iterations; k++) {
        int i1 = 0, i2 = 0;
        while (i1 == i2) {
            i1 = rng(n);
            i2 = rng(n);
        }
        const cv::Point2f &p1 = points[i1];
        const cv::Point2f &p2 = points[i2];

        cv::Point2f dp = p2 - p1;//直线的方向向量
        dp *= 1. / norm(dp);
        double score = 0;

        if (dp.y / dp.x <= k_max && dp.y / dp.x >= k_min) {
            for (int i = 0; i < n; i++) {
                cv::Point2f v = points[i] - p1;
                double d = v.y * dp.x - v.x * dp.y;//向量a与b叉乘/向量b的摸.||b||=1./norm(dp)
                //score += exp(-0.5*d*d/(sigma*sigma));//误差定义方式的一种
                if (fabs(d) < sigma)
                    score += 1;
            }
        }
        if (score > bestScore) {
            line = cv::Vec4f(dp.x, dp.y, p1.x, p1.y);
            bestScore = score;
        }
    }
}


Eigen::VectorXf norm_test() {
    float x[5] = {1, 2, 3, 4, 5};
    float y[5] = {7, 35, 103, 229, 431};

    vector<float> X(x, x + sizeof(x) / sizeof(float));
    vector<float> Y(y, y + sizeof(y) / sizeof(float));

    Eigen::VectorXf result(LSFit(X, Y, 3));

    cout << "\nThe coefficients vector is: \n"
         << endl;
    cout << result << endl;
    return result;
    //    for (size_t i = 0; i < result.size(); ++i)
    //        cout << "theta_" << i << ": " << result[i] << endl;
}