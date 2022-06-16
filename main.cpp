#include "config.h"
#include "library.h"
#include <argparse/argparse.hpp>
#include <fstream>
#include <iostream>
#ifdef USE_MYMATH
#include "MathFunctions.h"
#else
#include <cmath>
#endif
#include "leastsquare.h"
#include "nlohmann/json.hpp"
#include "opencv2/opencv.hpp"

using namespace std;

void ransacTest() {
    cv::Mat image(720, 1280, CV_8UC3, cv::Scalar(125, 125, 125));

    //以车道线参数为(0.7657,-0.6432,534,548)生成一系列点
    double k = -0.6432 / 0.7657;
    double b = 548 - k * 534;

    std::vector<cv::Point2f> points;

    for (int i = 360; i < 720; i += 10) {
        cv::Point2f point(int((i - b) / k), i);
        points.emplace_back(point);
    }

    //加入直线的随机噪声
    cv::RNG rng((unsigned) time(NULL));
    for (int i = 360; i < 720; i += 10) {
        int x = int((i - b) / k);
        x = rng.uniform(x - 10, x + 10);
        int y = i;
        y = rng.uniform(y - 30, y + 30);
        cv::Point2f point(x, y);
        points.emplace_back(point);
    }

    //加入噪声
    for (int i = 0; i < 720; i += 20) {
        int x = rng.uniform(1, 640);
        int y = rng.uniform(1, 360);

        cv::Point2f point(x, y);
        points.emplace_back(point);
    }


    int n = points.size();
    for (int j = 0; j < n; ++j) {
        cv::circle(image, points[j], 5, cv::Scalar(0, 0, 0), -1);
    }


    //RANSAC 拟合
    if (1) {
        cv::Vec4f lineParam;
        ransacFitCV(points, lineParam, 1000, 10);
        double k = lineParam[1] / lineParam[0];
        double b = lineParam[3] - k * lineParam[2];

        cv::Point p1, p2;
        p1.y = 720;
        p1.x = (p1.y - b) / k;

        p2.y = 360;
        p2.x = (p2.y - b) / k;

        cv::line(image, p1, p2, cv::Scalar(0, 255, 0), 2);
    }


    //最小二乘法拟合
    if (1) {
        cv::Vec4f lineParam;
        cv::fitLine(points, lineParam, cv::DIST_L2, 0, 0.01, 0.01);
        double k = lineParam[1] / lineParam[0];
        double b = lineParam[3] - k * lineParam[2];

        cv::Point p1, p2;
        p1.y = 720;
        p1.x = (p1.y - b) / k;

        p2.y = 360;
        p2.x = (p2.y - b) / k;

        cv::line(image, p1, p2, cv::Scalar(0, 0, 255), 2);
    }


    cv::imshow("image", image);
    cv::waitKey(0);
}


void jsonPractice(const string input, const string output) {
    nlohmann::json j;
    // add a number that is stored as double (note the implicit conversion of j to an object)
    j["pi"] = 3.141;

    // add a Boolean that is stored as bool
    j["happy"] = true;

    // add a string that is stored as std::string
    j["name"] = "Niels";

    // add another null object by passing nullptr
    j["nothing"] = nullptr;

    // add an object inside the object
    j["answer"]["everything"] = 42;

    // add an array that is stored as std::vector (using an initializer list)
    j["list"] = {1, 0, 2};

    // add another object (using an initializer list of pairs)
    j["object"] = {{"currency", "USD"}, {"value", 42.99}};

    // instead, you could also write (which looks very similar to the JSON above)
    nlohmann::json j2 = {
            {"pi", 3.141},
            {"happy", true},
            {"name", "Niels"},
            {"nothing", nullptr},
            {"answer", {{"everything", 42}}},
            {"list", {1, 0, 2}},
            {"object", {{"currency", "USD"}, {"value", 42.99}}}};

    // Serialization / Deserialization
    // To/from strings
    // by appending _json

    // create object from string literal
    nlohmann::json j3 = "{ \"happy\": true, \"pi\": 3.141 }"_json;

    // or even nicer with a raw string literal
    auto j4 = R"(
        {
        "happy": true,
            "pi": 3.141
          }
        )"_json;

    string s = j.dump();
    cout << "dump to string: " << s << endl;

    // read from file
    ifstream i(input);
    nlohmann::json j5;
    i >> j;
    ofstream o(output);
    // the setw manipulator was overloaded to set the indentation for pretty printing
    o << setw(4) << j << endl;
    cout << "write json to file:" << output << endl;
}

int main(int argc, char *argv[]) {
    argparse::ArgumentParser program("main");
    program.add_argument("square")
            .help("display the square of a given integer")
            .scan<'i', int>();
    //    program.add_argument("-v", "--verbose");// parameter packing
    program.add_argument("-i", "--input");
    program.add_argument("-o", "--output");

    try {
        program.parse_args(argc, argv);
    } catch (const std::runtime_error &err) {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        std::exit(1);
    }

    auto input = program.get<int>("square");
    std::cout << (input * input) << std::endl;

    std::cout << "Hello, World!" << std::endl;

    std::cout << "math power:" << std::endl;
    double base = 2;
    int exponent = 4;
#ifdef USE_MYMATH
    cout << "Use our own math library" << endl;
    double result = power(base, exponent);
#else
    cout << "Use the standard library" << endl;
    double result = pow(base, exponent);
#endif
    cout << base << "^" << exponent << "=" << result << endl;


    // json test
    cout << "Json practice" << endl;
    //    auto f_in = program.get("i");
    //    auto f_out = program.get("o");
    //    jsonPractice(f_in, f_out);

    // least square method test
    Eigen::VectorXf coeff = norm_test();

    // ransac test
    ransacTest();


    return 0;
}
