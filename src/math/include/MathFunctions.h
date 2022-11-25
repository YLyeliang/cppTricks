//
// Created by 叶亮 on 2022/5/19.
//

#ifndef CPPTRICKS_MATHFUNCTIONS_H
#define CPPTRICKS_MATHFUNCTIONS_H

extern double power(double base, int exponent);

#include "math.h"
#include <iostream>
#include <vector>
using namespace std;

double f(double x, double a, double b, double c, double d) { return a * x * x * x + b * x * x + c * x + d; }

double fd(double x, double a, double b, double c) { return 3 * a * x * x + 2 * b * x + c; }

void newton(double a, double b, double c, double d, vector<double> &v) {
    vector<double>::iterator it;//vector迭代器

    bool u;                          //用来判断是否重复
    for (int x0 = 0; x0 <= 50; x0++)//在一个大区域中逐个点用牛顿法，可找出大多数3次方程所有根
    {
        double x1 = x0;
        while (abs(f(x1, a, b, c, d)) > 0.0001) {
            double x = x1;
            x1 = x - f(x, a, b, c, d) / fd(x, a, b, c);
        }
        for (it = v.begin(); it != v.end(); it++) {
            if (abs((*it - x1)) < 0.05) {
                u = 1;
                break;
            }
        }
        if (u != 1 && x1 < 1000000000) {
            cout << x1 << " ";
            v.push_back(x1);//把已得到的解添加到vector，用于防止重复
        }
        u = 0;
    }
}

#endif//CPPTRICKS_MATHFUNCTIONS_H
