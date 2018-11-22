#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <typeinfo>

using namespace std;
using namespace cv;

void find_feature_matches(const cv::Mat& img1,
                          const cv::Mat& img2,
                          std::vector<cv::KeyPoint>& keypoints1,
                          std::vector<cv::KeyPoint>& keypoints2,
                          std::vector<cv::DMatch>&   matches)
{
    // Initialization
    Mat descriptor1, descriptor2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    // 1. Detect Oriented FAST corner points
    detector->detect( img1, keypoints1 );
    detector->detect( img2, keypoints2 );

    // 2. Compute BRIEF descriptor
    descriptor->compute( img1, keypoints1, descriptor1 );
    descriptor->compute( img2, keypoints2, descriptor2 );

    // Mat outimg;
    // drawKeypoints( img1, keypoints1, outimg, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    // imshow( "ORB Feature points", outimg );
    // waitKey(0);

    // 3. Pair BRIEF descriptors in two images, using Hamming distance
    vector<DMatch> all_matches;
    matcher->match( descriptor1, descriptor2, all_matches );

    // 4. Select pairs
    double min_dist = 10000, max_dist = 0;
    // Find minimum and maximum distance of all pairs
    for(int i = 0; i < descriptor1.rows; i++)
    {
        double dist = all_matches[i].distance;
        if(dist < min_dist) min_dist = dist;
        if(dist > max_dist) max_dist = dist;
    }
    cout << "-- Max dist = " << max_dist << endl;
    cout << "-- Min dist = " << min_dist << endl;
    // Consider those pairs wrong, whose distance is greater than twice min_dist;
    // but setting a lower limmit, say 30, is necessary, because minimum distance may sometimes be very small
    for(int i = 0; i < descriptor1.rows; i++)
    {
        if (all_matches[i].distance <= max(2*min_dist, 30.0))
        {
            matches.push_back(all_matches[i]);
        }
    }
}

void pose_estimation_2d2d(const std::vector<cv::KeyPoint>& keypoints1,
                          const std::vector<cv::KeyPoint>& keypoints2,
                          const std::vector<cv::DMatch>&   matches,
                          cv::Mat& R,
                          cv::Mat& t)
{
    // Camera intrinsics, TUM Freiburg2
    Mat K = ( Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );

    // Convert keypoints to Point2f type
    vector<Point2f> points1;
    vector<Point2f> points2;
    for(int i = 0; i < (int)matches.size(); i++)
    {
        points1.push_back( keypoints1[matches[i].queryIdx].pt );
        points2.push_back( keypoints2[matches[i].trainIdx].pt );
    }

    // Compute fundamental matrix F
    Mat fundamental_matrix;
    fundamental_matrix = findFundamentalMat(points1, points2, CV_FM_8POINT);
    cout << "Fundamental matrix is " << endl << fundamental_matrix << endl;

    // Compute essential matrix E
    Point2d principle_point(325.1, 249.7);      // Camera optical center, TUM dataset calibration data
    double focal_length = 521;                  // Camera focal length, TUM dataset calibration data
    Mat essential_matrix;
    essential_matrix = findEssentialMat(points1, points2, focal_length, principle_point);
    cout << "Essential matrix is " << endl << essential_matrix << endl;

    // Compute homography matrix H
    Mat homography_matrix;
    homography_matrix = findHomography(points1, points2, RANSAC, 3);
    cout << "Homography matrix is " << endl << homography_matrix << endl;

    // Recover rotation and translate from E
    recoverPose(essential_matrix, points1, points2, R, t, focal_length, principle_point);
    cout << "R = " << endl << R << endl;
    cout << "t = " << endl << t << endl;
}

cv::Point2f pixel2cam( const cv::Point2d& p, const cv::Mat& K )
{
    return Point2f( (p.x - K.at<double>(0,2)) / K.at<double>(0, 0),
                    (p.y - K.at<double>(1,2)) / K.at<double>(1, 1) );
}

void triangulation(const std::vector<cv::KeyPoint>& keypoints1,
                   const std::vector<cv::KeyPoint>& keypoints2,
                   const std::vector<cv::DMatch>&   matches,
                   const cv::Mat& R,
                   const cv::Mat& t,
                   std::vector<cv::Point3d>& points)
{
    // Construct a project matrix of the first camera
    Mat T1 = ( Mat_<float>(3, 4) << 
               1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1, 0 );

    // Project matrix of the second camera
    Mat T2 = ( Mat_<float>(3, 4) <<
               R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
               R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
               R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0) );

    // Camera intrinsics
    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    // Convert pixel coord to normalized plane coord
    vector<Point2f> pts1, pts2;
    for( auto m: matches )
    {
        pts1.push_back( pixel2cam(keypoints1[m.queryIdx].pt, K) );
        pts2.push_back( pixel2cam(keypoints2[m.trainIdx].pt, K) );
    }

    // Compute homogeneous reconstructed points coord with triangulation method
    Mat pts;
    triangulatePoints(T1, T2, pts1, pts2, pts);

    // Convert to non-homogeneous coordinate
    for(int i = 0; i < pts.cols; i++)
    {
        Mat x = pts.col(i);
        x /= x.at<float>(3, 0);    // Normalize
        Point3d p( x.at<float>(0, 0),   // Note: data type must be "float" (not "double") here, or the result will be wrong
                   x.at<float>(1, 0),
                   x.at<float>(2, 0) );
        points.push_back( p );
    }
}
