#include "config.h"
#include "library.h"
// #include <argparse/argparse.hpp>
#include <fstream>
#include <iostream>
#ifdef USE_MYMATH
#include "MathFunctions.h"
#else
#include <cmath>
#endif
#include "leastsquare.h"
#include "matrix_demo.h"
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


struct pt2d {
    float x;
    float y;
};

void jsonPractice(const string input, const string output) {
    nlohmann::json j;

    j["2vec"] = {};
    // struct test, custom struct not work
    pt2d pp{1, 2};
    vector<float> pp2{1, 2};
    j["struct"] = pp2;

    // vector<vector> test
    vector<vector<int>> vec2{{1, 2, 3}, {2, 2, 3}};
    j["2vec"][0]["points"] = vec2;
    j["2vec"][1] = vec2;

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
    // ifstream i(input);
    // nlohmann::json j5;
    // i >> j;
    ofstream o(output);
    // the setw manipulator was overloaded to set the indentation for pretty printing
    o << setw(4) << j << endl;
    cout << "write json to file:" << output << endl;
}

void ReadTxtDemo() {
    string file = "../result_smooth.txt";
    ifstream inf;
    string s;
    inf.open(file);
    double timestamp, pitch, roll;
    vector<double> t_arr;
    vector<double> p_arr;
    vector<double> r_arr;
    while (getline(inf, s)) {
        stringstream word(s);
        if (s == " ")
            break;
        word >> timestamp >> pitch >> roll;
        cout << timestamp << pitch << roll << endl;
        t_arr.push_back(timestamp / 100);
        p_arr.push_back(pitch);
        r_arr.push_back(roll);
    }
    sort(t_arr.begin(), t_arr.end(), [](double x, double y) { return x < y; });

    double tmp = t_arr[0];
    for (const auto a: t_arr)
        cout << fixed << setprecision(2) << a << endl;
}

void WriteTxtDemo() {
    string file = "write_demo.txt";
    ofstream out_file(file);
    Eigen::Quaterniond quat(0, 1, 2, 3);
    Eigen::Vector3d t_dd(22, 33, 44);
    cout << quat.w() << quat.x() << quat.y() << quat.z() << endl;
    cout << t_dd.x() << t_dd.y() << t_dd.z() << endl;
    out_file << quat.w() << quat.x() << quat.y() << quat.z() << t_dd.x() << t_dd.y() << t_dd.z() << endl;
    out_file << setprecision(10) << M_PI << M_PI << M_PI << M_PI << endl;
}

void vector_demo() {
    // 初始化vector指定长度，但不赋值时的初始值测试
    vector<pair<int, int>> vec_pair(5);
    cout << vec_pair[4].first << endl;

    // 迭代器长度计算
    auto length = vec_pair.end() - vec_pair.begin();

    int debug = 1;
}

void EigenDemo() {
    chrono::steady_clock::time_point s_time = std::chrono::steady_clock::now();
    int n = 10;
    Eigen::Matrix<float, 3, Eigen::Dynamic> last_coord(3, n);
    Eigen::Matrix<float, 3, Eigen::Dynamic> cur_coord(3, n);
    Eigen::Matrix3d mat3d = Eigen::Matrix3d::Ones();
    Eigen::Vector3d vec3d(1, 2, 3);
    cout << mat3d << endl;
    cout << vec3d << endl;
    // broadcast,
    // auto res = mat3d + vec3d.transpose();
    mat3d.colwise() += vec3d;
    cout << mat3d << endl;
    // cur_coord = rotation_matrix4 * last_coord;
    int debug = 1;
}
struct lan3d {
    float x;
    float y;
    float z;
};
struct lane3dw {
    lan3d pt;
    float w;
};
int main(int argc, char *argv[]) {
    lan3d a{1, 2, 3};
    vector<lane3dw> Points{{a, 0.5}};
    auto dd = Points.back().pt.x - Points.front().pt.x;
    // EigenDemo();
    // vector_demo();

    // 多线程demo
    // ProducerConsumerDemo();

    // json demo
    // jsonPractice("dd", "tmp.json");
    //    Eigen::VectorXf coeff = norm_test();
    // ransac test
    // ransacTest();
    // WriteTxtDemo();

    // sort test
    // vector<double> dd{9,5,32,22,2,2,12,5,6,7,3,0.12312312,-0.12312312};
    // vector<pair<double,int>> aa;
    // for (auto a:dd)
    //     aa.emplace_back(a,0);
    // std::sort(dd.begin(),dd.end(),[](const double i,const double j){return i<j;});
    // std::sort(aa.begin(),aa.end(),[](pair<double,int> i,pair<double,int> j){return i.first<j.first;});
    // cout<<aa.end()->first<<endl;
    // cout<<aa.back().first<<endl;
    // dd.resize(0);
    // WriteTxtDemo();

    // MatrixBlockDemo();

    return 0;
}
