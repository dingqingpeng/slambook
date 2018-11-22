#ifndef VO1_H_
#define VO1_H_

// #include <iostream>
#include <opencv2/core/core.hpp>

/* Brief: Find feature matches in two images
 * Parameters:
 *     img1 -- source image 1
 *     img2 -- source image 2
 *     keypoints1 -- key points in image 1
 *     keypoints2 -- key points in image 2
 *     matches -- selected matches between two images
 * Return:
 *     void
 */
void find_feature_matches(const cv::Mat& img1,
                          const cv::Mat& img2,
                          std::vector<cv::KeyPoint>& keypoints1,
                          std::vector<cv::KeyPoint>& keypoints2,
                          std::vector<cv::DMatch>&   matches);

/* Brief: Estimate camera motion between two images, using epipolar geometry
 *        Recover R and t from essential matrix
 * Parameters:
 *     keypoints1 -- key points in image 1
 *     keypoints2 -- key points in image 2
 *     matches -- selected matches between two images
 *     R -- rotation matrix
 *     t -- translation vector
 * Return:
 *     void
 */
void pose_estimation_2d2d(const std::vector<cv::KeyPoint>& keypoints1,
                          const std::vector<cv::KeyPoint>& keypoints2,
                          const std::vector<cv::DMatch>&   matches,
                          cv::Mat& R,
                          cv::Mat& t);

/* Brief: Compute 3D position 
 * Parameters:
 *     keypoints1 -- key points in image 1
 *     keypoints2 -- key points in image 2
 *     matches -- selected matches between two images
 *     R -- rotation matrix
 *     t -- translation vector
 *     points -- 3D points in homogeneous coordinates
 * Return:
 *     void
 */
void triangulation(const std::vector<cv::KeyPoint>& keypoints1,
                   const std::vector<cv::KeyPoint>& keypoints2,
                   const std::vector<cv::DMatch>&   matches,
                   const cv::Mat& R,
                   const cv::Mat& t,
                   std::vector<cv::Point3d>& points);

/* Brief: Convert pixel coord to normalized plane
 * Parameters:
 *     p -- point coord (u, v)
 *     K -- camera intrinsics
 * Return:
 *     coord in normalized plane
 */
cv::Point2f pixel2cam( const cv::Point2d& p, const cv::Mat& K );

#endif