#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "vo1.h"

using namespace std;
using namespace cv;

int main(int argc, char const *argv[])
{
    // Print usage information if wrong usage occured.
    if( argc != 3 )
    {
        cout << "application usage: feature_extraction img1 img2" << endl;
        return 1;
    }

    // Import images
    Mat img1 = imread( argv[1], CV_LOAD_IMAGE_COLOR );
    Mat img2 = imread( argv[2], CV_LOAD_IMAGE_COLOR );

    // Extract features
    vector<KeyPoint> keypoints1, keypoints2;
    vector<DMatch> matches;
    find_feature_matches(img1, img2, keypoints1, keypoints2, matches);
    cout << matches.size() << " pairs of matches are found." << endl;

    // Estimate motion between two images
    Mat R, t;
    pose_estimation_2d2d(keypoints1, keypoints2, matches, R, t);

    // Triangulation
    vector<Point3d> points;
    triangulation(keypoints1, keypoints2, matches, R, t, points);

    // Check reprojection of triangulated point and feature point
    // Compute coordinates in normalized plane
    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    for(int i = 0; i < matches.size(); i++)
    {
        // The first frame
        // Convert feature point coordinate to normalized plane
        Point2d pt1 = pixel2cam(keypoints1[matches[i].queryIdx].pt, K);
        // Project 3D point on the normalized plane
        Point2d pt1_3d( points[i].x / points[i].z,
                        points[i].y / points[i].z );
        cout << "point in the first camera frame:  " << pt1 << endl;
        cout << "piont projected from 3d point:    " << pt1_3d << endl;

        // The second frame
        // Convert feature point coordinate to normalized plane
        Point2d pt2 = pixel2cam(keypoints2[matches[i].trainIdx].pt, K);
        // Project 3D point on the normalized plane
        Mat pt2_3d_trans = R * ( Mat_<double>(3, 1) << points[i].x, points[i].y, points[i].z ) + t;
        pt2_3d_trans /= pt2_3d_trans.at<double>(2, 0);
        cout << "point in the second camera frame: " << pt2 << endl;
        cout << "piont projected from 3d point:    " << pt2_3d_trans.t() << endl;
        cout << "-----------------------------" << endl;
    }

    return 0;
}




