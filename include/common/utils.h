// get kalibr data
#ifndef __UTILS_H__
#define __UTILS_H__

#include <ros/ros.h>
#include <string>
#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Geometry>

using Eigen::Isometry3d;
using std::string;
using cv::Mat;

namespace utils{
    Isometry3d getTransformEigen(const ros::NodeHandle& nh,
                                          const string& field);

    Mat getTransformCV(const ros::NodeHandle& nh,
                       const string& filed);

    Mat getVec16Transform(const ros::NodeHandle &nh,
                          const string &field);

    Mat getKalibrStyleTransform(const ros::NodeHandle &nh,
                                const string &field);
    
};

#endif