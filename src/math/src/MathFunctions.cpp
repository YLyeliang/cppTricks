// created by yel on 2022/5/19
#include "iostream"
using namespace std;

double power(double base, int exponent) {
    double result = base;
    int i;
    if (exponent == 0) {
        return 1;
    }
    for (i = 1; i < exponent; ++i) {
        result = result * base;
    }
    return result;
}