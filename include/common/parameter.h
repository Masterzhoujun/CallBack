#ifndef __PARAMETER_H__
#define __PARAMETER_H__

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

using Eigen::Matrix3d;
using Eigen::Vector3d;

class Parameter
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Parameter();

    ~Parameter();

    // Roation matrix from cam to imu
    static Matrix3d R_cam0_imu, R_cam1_imu, R_cam1_cam0;
    // translation vector from cam to imu
    static Vector3d t_cam0_imu, t_cam1_imu, t_cam1_cam0;
    // t_imu = t_cam + shift
    static double time_shift;
    
    // image size
    static int image_row, image_col;

    // camera0 and camera1 intrinsics matrix
    static cv::Vec4d cam0_intrinsics, cam1_intrinsics;

    static cv::Vec4d cam0_distortion_coeffs, cam1_distortion_coeffs;
    // gird size
    static int grid_row, grid_col;

    static int per_grid_feature_num;

};// end of class definition

bool LoadParameters(ros::NodeHandle& nh);

#endif