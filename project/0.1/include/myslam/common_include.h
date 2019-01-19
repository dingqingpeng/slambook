#ifndef COMMON_INCLUDE_H
#define COMMON_INCLUDE_H

// std
#include <iostream>
#include <string>
#include <memory>
#include <unordered_map>
using namespace std;

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;

// Sophus
#include <sophus/se3.h>
using Sophus::SE3;

// OpenCV
#include <opencv2/core/core.hpp>
using cv::Mat;

#endif