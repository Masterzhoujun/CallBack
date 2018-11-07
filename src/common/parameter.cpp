#include "common/parameter.h"

#include <opencv2/core/eigen.hpp>
#include <vector>
#include "common/utils.h"

using std::vector;
using std::cout;
using std::endl;

Matrix3d Parameter::R_cam0_imu, Parameter::R_cam1_imu, Parameter::R_cam1_cam0;
Vector3d Parameter::t_cam0_imu, Parameter::t_cam1_imu, Parameter::t_cam1_cam0;

double Parameter::time_shift;

int Parameter::image_row;
int Parameter::image_col;

cv::Vec4d Parameter::cam0_intrinsics;
cv::Vec4d Parameter::cam1_intrinsics;

cv::Vec4d Parameter::cam0_distortion_coeffs;
cv::Vec4d Parameter::cam1_distortion_coeffs;

int Parameter::grid_row;
int Parameter::grid_col;

int Parameter::per_grid_feature_num;

Parameter::Parameter()
{}

Parameter::~Parameter()
{}

bool LoadParameters(ros::NodeHandle& nh)
{
    //get camera0 and camera1 image size
    vector<int> cam0_resolution_temp(2);
    nh.getParam("cam0/resolution", cam0_resolution_temp);

    vector<int> cam1_resolution_temp(2);
    nh.getParam("cam1/resolution", cam1_resolution_temp);
    
    if (cam0_resolution_temp[0] != cam1_resolution_temp[0] || 
            cam0_resolution_temp[1] != cam1_resolution_temp[1])
        ROS_WARN("Stereo image do not have same size!");

    Parameter::image_row = cam0_resolution_temp[1];
    Parameter::image_col = cam0_resolution_temp[0]; 

    //get camera0 and camera1 intrinsics matrix
    vector<double> cam0_intrinsics_tmp(4);
    nh.getParam("cam0/intrinsics", cam0_intrinsics_tmp);
    Parameter::cam0_intrinsics[0] = cam0_intrinsics_tmp[0]; // fx
    Parameter::cam0_intrinsics[1] = cam0_intrinsics_tmp[1]; // fy
    Parameter::cam0_intrinsics[2] = cam0_intrinsics_tmp[2]; // cx
    Parameter::cam0_intrinsics[3] = cam0_intrinsics_tmp[3]; // cy

    vector<double> cam1_intrinsics_tmp(4);
    nh.getParam("cam1/intrinsics", cam1_intrinsics_tmp);
    Parameter::cam1_intrinsics[0] = cam1_intrinsics_tmp[0];
    Parameter::cam1_intrinsics[1] = cam1_intrinsics_tmp[1];
    Parameter::cam1_intrinsics[2] = cam1_intrinsics_tmp[2];
    Parameter::cam1_intrinsics[3] = cam1_intrinsics_tmp[3];

    vector<double> cam0_distortion_coeffs_temp(4);
    nh.getParam("cam0/distortion_coeffs", cam0_distortion_coeffs_temp);
    Parameter::cam0_distortion_coeffs[0] = cam0_distortion_coeffs_temp[0];
    Parameter::cam0_distortion_coeffs[1] = cam0_distortion_coeffs_temp[1];
    Parameter::cam0_distortion_coeffs[2] = cam0_distortion_coeffs_temp[2];
    Parameter::cam0_distortion_coeffs[3] = cam0_distortion_coeffs_temp[3];

    vector<double> cam1_distortion_coeffs_temp(4);
    nh.getParam("cam1/distortion_coeffs", cam1_distortion_coeffs_temp);
    Parameter::cam1_distortion_coeffs[0] = cam1_distortion_coeffs_temp[0];
    Parameter::cam1_distortion_coeffs[1] = cam1_distortion_coeffs_temp[1];
    Parameter::cam1_distortion_coeffs[2] = cam1_distortion_coeffs_temp[2];
    Parameter::cam1_distortion_coeffs[3] = cam1_distortion_coeffs_temp[3];
   
    // get imu-camera parameter
    cv::Mat T_cam0_imu = utils::getTransformCV(nh, "cam0/T_cam_imu");
    cv::Mat R = T_cam0_imu.rowRange(0, 3).colRange(0, 3);
    cv::Mat t = T_cam0_imu.rowRange(0, 3).col(3);
    cv::cv2eigen(R, Parameter::R_cam0_imu);
    cv::cv2eigen(t, Parameter::t_cam0_imu);
    Eigen::Quaterniond Q(Parameter::R_cam0_imu);
    Parameter::R_cam0_imu = Q.normalized();

    cv::Mat T_cam1_imu = utils::getTransformCV(nh, "cam1/T_cam_imu");
    R = T_cam1_imu.rowRange(0, 3).colRange(0, 3);
    t = T_cam1_imu.rowRange(0, 3).col(3);
    cv::cv2eigen(R, Parameter::R_cam1_imu);
    cv::cv2eigen(t, Parameter::t_cam1_imu);
    Eigen::Quaterniond Q1(Parameter::R_cam1_imu);
    Parameter::R_cam1_imu = Q1.normalized();

    cv::Mat T_cam1_cam0 = utils::getTransformCV(nh, "cam1/T_cam1_cam0");
    R = T_cam1_cam0.rowRange(0, 3).colRange(0, 3);
    t = T_cam1_cam0.rowRange(0, 3).col(3);
    cv::cv2eigen(R, Parameter::R_cam1_cam0);
    cv::cv2eigen(t, Parameter::t_cam1_cam0);
    Eigen::Quaterniond Q2(Parameter::R_cam1_cam0);
    Parameter::R_cam1_cam0 = Q2.normalized();
   
    nh.param<int>("grid_row", Parameter::grid_row, 4);
    nh.param<int>("grid_col", Parameter::grid_col, 4);

    nh.param<int>("per_grid_feature_num", Parameter::per_grid_feature_num, 2);
    nh.param<double>("time_shift", Parameter::time_shift, 0.0);

    ROS_INFO("===========================================");
    ROS_INFO("camera resolution: %d, %d",
        Parameter::image_row, Parameter::image_col);
    ROS_INFO("cam0_intrinscs: %f, %f, %f, %f",
        Parameter::cam0_intrinsics[0], Parameter::cam0_intrinsics[1],
        Parameter::cam0_intrinsics[2], Parameter::cam0_intrinsics[3]);
    ROS_INFO("cam0_distortion_coefficients: %f, %f, %f, %f",
        Parameter::cam0_distortion_coeffs[0], Parameter::cam0_distortion_coeffs[1],
        Parameter::cam0_distortion_coeffs[2], Parameter::cam0_distortion_coeffs[3]);

    ROS_INFO("cam1_intrinscs: %f, %f, %f, %f",
        Parameter::cam1_intrinsics[0], Parameter::cam1_intrinsics[1],
        Parameter::cam1_intrinsics[2], Parameter::cam1_intrinsics[3]);
    ROS_INFO("cam1_distortion_coefficients: %f, %f, %f, %f",
        Parameter::cam1_distortion_coeffs[0], Parameter::cam1_distortion_coeffs[1],
        Parameter::cam1_distortion_coeffs[2], Parameter::cam1_distortion_coeffs[3]);

    cout << Parameter::R_cam0_imu << endl;
    cout << "t_cam0_imu " << Parameter::t_cam0_imu.transpose() << endl;

    cout << Parameter::R_cam1_imu << endl;
    cout << "t_cam1_imu " << Parameter::t_cam1_imu.transpose() << endl;

    cout << Parameter::R_cam1_cam0 << endl;
    cout << "t_cam1_cam0 " << Parameter::t_cam1_cam0.transpose() << endl;

    ROS_INFO("grid_row: %d", Parameter::grid_row);
    ROS_INFO("grid_col: %d", Parameter::grid_col);
    ROS_INFO("per_grid_feature_num: %d", Parameter::per_grid_feature_num);
    ROS_INFO("time_shift: %f", Parameter::time_shift);
    ROS_INFO("===========================================");
    return true;
}