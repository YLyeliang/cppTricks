//
// Created by yel on 22-6-22.
//
#include "matrix_demo.h"
using namespace std;

Eigen::Matrix4d extrinsic_matrix_;
Eigen::Matrix4d ground_vehicle_matrix_;

int ReadCameraExtrinsic(const std::string& yaml_file)
{
    YAML::Node node = YAML::LoadFile(yaml_file);
    double w = node["transform"]["rotation"]["w"].as<double>();
    double x = node["transform"]["rotation"]["x"].as<double>();
    double y = node["transform"]["rotation"]["y"].as<double>();
    double z = node["transform"]["rotation"]["z"].as<double>();
    double tx = node["transform"]["translation"]["x"].as<double>();
    double ty = node["transform"]["translation"]["y"].as<double>();
    double tz = node["transform"]["translation"]["z"].as<double>();
    extrinsic_matrix_.setIdentity();
    Eigen::Quaterniond quat(w, x, y, z);

    /*Eigen::Matrix3d R_inv = (quat.toRotationMatrix()).inverse();
    extrinsic_matrix_.block<3, 3>(0, 0) = R_inv;
    Eigen::Vector3d T(tx, ty, tz);
    Eigen::Vector3d T_inv = R_inv * T;
    extrinsic_matrix_(0, 3) = T_inv(0) * (-1);
    extrinsic_matrix_(1, 3) = T_inv(1) * (-1);
    extrinsic_matrix_(2, 3) = T_inv(2) * (-1);*/
    // 车体和相机之间的转换关系
    extrinsic_matrix_.block<3, 3>(0, 0) = quat.toRotationMatrix();
    extrinsic_matrix_(0, 3) = tx;
    extrinsic_matrix_(1, 3) = ty;
    extrinsic_matrix_(2, 3) = tz;

    //地面和车体之间的关系
    ground_vehicle_matrix_.setIdentity();
    //sensor_para_sp_->vehicle_para_.tire_radius_m_;//轮胎半径
    //ground_vehicle_matrix_(2, 3) = 0; // to do
    //相机和地面之间的关系
    //Eigen::Matrix4d extrinsic_matrix_inverse_ = (ground_vehicle_matrix_ * extrinsic_matrix_).inverse();

    return 0;
}


void UpdateExtrinsicMatrixByPitch(const float &pitch,
                                                const Eigen::Matrix4d &origin_extrinsic_matrix,
                                                Eigen::Matrix4d &refine_extrinsic_matrix) {
    // update extrinsic use vanish points
    refine_extrinsic_matrix.block<4, 1>(0, 3) = origin_extrinsic_matrix.block<4, 1>(0, 3);

    Eigen::Matrix3d rotation_matrix = origin_extrinsic_matrix.block<3, 3>(0, 0);
    Eigen::Matrix3d trans_rotation_matrix;
    cout<<origin_extrinsic_matrix<<endl;
    trans_rotation_matrix.block<1, 3>(0, 0) = -1.f * rotation_matrix.block<1, 3>(1, 0);
    trans_rotation_matrix.block<1, 3>(1, 0) = -1.f * rotation_matrix.block<1, 3>(2, 0);
    trans_rotation_matrix.block<1, 3>(2, 0) = rotation_matrix.block<1, 3>(0, 0);
    cout<<trans_rotation_matrix<<endl;

    Eigen::Matrix3d trans_rotation_matrix_inverse = trans_rotation_matrix.inverse();
    cout<< trans_rotation_matrix_inverse<<endl;
    Eigen::Vector3d trans_euler_angle_inverse = trans_rotation_matrix_inverse.eulerAngles(0, 1, 2);
    cout<<trans_euler_angle_inverse<<endl;

    if (fabs(trans_euler_angle_inverse(0)) > M_PI / 2 &&
        fabs(trans_euler_angle_inverse(1)) > M_PI / 2 &&
        fabs(trans_euler_angle_inverse(2)) > M_PI / 2) {
        trans_euler_angle_inverse(0) = trans_euler_angle_inverse(0) - M_PI;
        trans_euler_angle_inverse(1) = M_PI - trans_euler_angle_inverse(1);
        trans_euler_angle_inverse(2) = trans_euler_angle_inverse(2) - M_PI;
    }
    cout<<trans_euler_angle_inverse<<endl;

    Eigen::AngleAxisd trans_roll_angle(Eigen::AngleAxisd(trans_euler_angle_inverse(2), Eigen::Vector3d::UnitZ()));
    Eigen::AngleAxisd trans_yaw_angle(Eigen::AngleAxisd(trans_euler_angle_inverse(1), Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd trans_pitch_angle(Eigen::AngleAxisd(trans_euler_angle_inverse(0)-pitch, Eigen::Vector3d::UnitX()));

    Eigen::AngleAxisd trans_rotation_vector;
    trans_rotation_vector = trans_pitch_angle * trans_yaw_angle * trans_roll_angle;

    rotation_matrix = trans_rotation_vector.matrix().inverse();
    trans_rotation_matrix.block<1, 3>(0, 0) = rotation_matrix.block<1, 3>(2, 0);
    trans_rotation_matrix.block<1, 3>(1, 0) = -1.f * rotation_matrix.block<1, 3>(0, 0);
    trans_rotation_matrix.block<1, 3>(2, 0) = -1.f * rotation_matrix.block<1, 3>(1, 0);

    refine_extrinsic_matrix.block<3, 3>(0, 0) = trans_rotation_matrix.block<3, 3>(0, 0);
}

void MatrixCopyTest(const Eigen::Matrix4d &orin,
                    Eigen::Matrix4d &refine){
    refine.block<4,1>(0,0) = orin.block<4,1>(0,2);
    Eigen::Matrix3d rotation_matrix = orin.block<3,3>(0,0);
    rotation_matrix.block<3,1>(0,2)<< 3,3,3;
    return;
}

// 欧拉角(RPY)， 旋转矩阵，旋转向量，四元数之间的转换
void MiscTransform(){
    /**** 1. 旋转向量 ****/
    cout << endl << "********** AngleAxis **********" << endl;
    //1.0 初始化旋转向量,沿Z轴旋转45度的旋转向量
    Eigen::AngleAxisd rotation_vector1 (M_PI/4, Eigen::Vector3d(0, 0, 1));

    //1.1 旋转向量转换为旋转矩阵
    //旋转向量用matrix()转换成旋转矩阵
    Eigen::Matrix3d rotation_matrix1 = Eigen::Matrix3d::Identity();
    rotation_matrix1 = rotation_vector1.matrix();
    cout << "rotation matrix1 =\n" << rotation_matrix1 << endl;
    //或者由罗德里格公式进行转换
    rotation_matrix1 = rotation_vector1.toRotationMatrix();
    cout << "rotation matrix1 =\n" << rotation_matrix1 << endl;

    /*1.2 旋转向量转换为欧拉角*/
    //将旋转向量转换为旋转矩阵,再由旋转矩阵转换为欧拉角,详见旋转矩阵转换为欧拉角
    Eigen::Vector3d eulerAngle1 = rotation_vector1.matrix().eulerAngles(2,1,0);
    cout << "eulerAngle1, z y x: " << eulerAngle1 << endl;

    /*1.3 旋转向量转四元数*/
    Eigen::Quaterniond quaternion1(rotation_vector1);
    //或者
    Eigen::Quaterniond quaternion1_1;
    quaternion1_1 = rotation_vector1;
    cout << "quaternion1 x: " << quaternion1.x() << endl;
    cout << "quaternion1 y: " << quaternion1.y() << endl;
    cout << "quaternion1 z: " << quaternion1.z() << endl;
    cout << "quaternion1 w: " << quaternion1.w() << endl;

    cout << "quaternion1_1 x: " << quaternion1_1.x() << endl;
    cout << "quaternion1_1 y: " << quaternion1_1.y() << endl;
    cout << "quaternion1_1 z: " << quaternion1_1.z() << endl;
    cout << "quaternion1_1 w: " << quaternion1_1.w() << endl;


    /**** 2. 旋转矩阵 *****/
    cout << endl << "********** RotationMatrix **********" << endl;
    //2.0 旋转矩阵初始化
    Eigen::Matrix3d rotation_matrix2;
    rotation_matrix2 << 0.707107, -0.707107, 0, 0.707107, 0.707107, 0, 0, 0, 1;
    ;
    //或直接单位矩阵初始化
    Eigen::Matrix3d rotation_matrix2_1 = Eigen::Matrix3d::Identity();

    //2.1 旋转矩阵转换为欧拉角
    //ZYX顺序，即先绕x轴roll,再绕y轴pitch,最后绕z轴yaw,0表示X轴,1表示Y轴,2表示Z轴
    Eigen::Vector3d euler_angles = rotation_matrix2.eulerAngles(2, 1, 0);
    cout << "yaw(z) pitch(y) roll(x) = " << euler_angles.transpose() << endl;

    //2.2 旋转矩阵转换为旋转向量
    Eigen::AngleAxisd rotation_vector2;
    rotation_vector2.fromRotationMatrix(rotation_matrix2);
    //或者
    Eigen::AngleAxisd rotation_vector2_1(rotation_matrix2);
    cout << "rotation_vector2 " << "angle is: " << rotation_vector2.angle() * (180 / M_PI)
         << " axis is: " << rotation_vector2.axis().transpose() << endl;

    cout << "rotation_vector2_1 " << "angle is: " << rotation_vector2_1.angle() * (180 / M_PI)
         << " axis is: " << rotation_vector2_1.axis().transpose() << endl;

    //2.3 旋转矩阵转换为四元数
    Eigen::Quaterniond quaternion2(rotation_matrix2);
    //或者
    Eigen::Quaterniond quaternion2_1;
    quaternion2_1 = rotation_matrix2;
    cout << "quaternion2 x: " << quaternion2.x() << endl;
    cout << "quaternion2 y: " << quaternion2.y() << endl;
    cout << "quaternion2 z: " << quaternion2.z() << endl;
    cout << "quaternion2 w: " << quaternion2.w() << endl;

    cout << "quaternion2_1 x: " << quaternion2_1.x() << endl;
    cout << "quaternion2_1 y: " << quaternion2_1.y() << endl;
    cout << "quaternion2_1 z: " << quaternion2_1.z() << endl;
    cout << "quaternion2_1 w: " << quaternion2_1.w() << endl;


    /**** 3. 欧拉角 ****/
    cout << endl << "********** EulerAngle **********" << endl;
    //3.0 初始化欧拉角(Z-Y-X，即RPY, 先绕x轴roll,再绕y轴pitch,最后绕z轴yaw)
    Eigen::Vector3d ea(0.785398, -0, 0);

    //3.1 欧拉角转换为旋转矩阵
    Eigen::Matrix3d rotation_matrix3;
    rotation_matrix3 = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitX());
    cout << "rotation matrix3 =\n" << rotation_matrix3 << endl;

    //3.2 欧拉角转换为四元数,
    Eigen::Quaterniond quaternion3;
    quaternion3 = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitZ()) *
                  Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitX());
    cout << "quaternion3 x: " << quaternion3.x() << endl;
    cout << "quaternion3 y: " << quaternion3.y() << endl;
    cout << "quaternion3 z: " << quaternion3.z() << endl;
    cout << "quaternion3 w: " << quaternion3.w() << endl;

    //3.3 欧拉角转换为旋转向量
    Eigen::AngleAxisd rotation_vector3;
    rotation_vector3 = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitX());
    cout << "rotation_vector3 " << "angle is: " << rotation_vector3.angle() * (180 / M_PI)
         << " axis is: " << rotation_vector3.axis().transpose() << endl;


    /**** 4.四元数 ****/
    cout << endl << "********** Quaternion **********" << endl;
    //4.0 初始化四元素,注意eigen Quaterniond类四元数初始化参数顺序为w,x,y,z
    Eigen::Quaterniond quaternion4(0.92388, 0, 0, 0.382683);

    //4.1 四元数转换为旋转向量
    Eigen::AngleAxisd rotation_vector4(quaternion4);
    //或者
    Eigen::AngleAxisd rotation_vector4_1;
    rotation_vector4_1 = quaternion4;
    cout << "rotation_vector4 " << "angle is: " << rotation_vector4.angle() * (180 / M_PI)
         << " axis is: " << rotation_vector4.axis().transpose() << endl;

    cout << "rotation_vector4_1 " << "angle is: " << rotation_vector4_1.angle() * (180 / M_PI)
         << " axis is: " << rotation_vector4_1.axis().transpose() << endl;

    //4.2 四元数转换为旋转矩阵
    Eigen::Matrix3d rotation_matrix4;
    rotation_matrix4 = quaternion4.matrix();
    Eigen::Matrix3d rotation_matrix4_1;
    rotation_matrix4_1 = quaternion4.toRotationMatrix();
    cout << "rotation matrix4 =\n" << rotation_matrix4 << endl;
    cout << "rotation matrix4_1 =\n" << rotation_matrix4_1 << endl;


    //4.4 四元数转欧拉角(Z-Y-X，即RPY)
    Eigen::Vector3d eulerAngle4 = quaternion4.matrix().eulerAngles(2,1,0);
    cout << "yaw(z) pitch(y) roll(x) = " << eulerAngle4.transpose() << endl;
}

void Lidar2VCSDebug(){
    Eigen::Quaterniond lidar_quat(0.9999421426827765,-0.005876674928257103, -0.008628512560051503, 0.002593212248659294);
    Eigen::Vector3d lidar_trans(3.935,0.0,0.647);
    Eigen::Matrix3d lidar_rotate_matrix = lidar_quat.toRotationMatrix();
    Eigen::Vector3d point(18.284977, 32.932728, 5.8179064);
    Eigen::Vector3d trans_point = lidar_rotate_matrix * point + lidar_trans;

    return;
}

void MatrixBlockDemo(){
    // block实验
    // Eigen::Matrix4d  m = Eigen::Matrix4d::Identity();
    // m.setOnes();
    // m.block<3,1>(0,0) <<0,0,0;
    // cout<<m<<endl;

    // 矩阵拷贝
    // Eigen::Matrix4d orin = Eigen::Matrix4d::Identity();
    // Eigen::Matrix4d refine = orin;
    // MatrixCopyTest(orin,refine);

    // 各种转换
    MiscTransform();

    // 雷达转VCS测试
    // Lidar2VCSDebug();

    ReadCameraExtrinsic("/home/yel/c_projects/cppTricks/camera_front_wide_2_vehicle_extrinsics.yaml");
    UpdateExtrinsicMatrixByPitch(0.017,extrinsic_matrix_,ground_vehicle_matrix_);
    cout<<"Success"<<endl;


}