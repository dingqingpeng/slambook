#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
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

    // Check E = t^R*scale
    Mat t_x = ( Mat_<double>(3, 3) << 
                                  0, -t.at<double>(2, 0),  t.at<double>(1, 0),
                 t.at<double>(2, 0),                   0, -t.at<double>(0, 0),
                -t.at<double>(1, 0),  t.at<double>(0, 0),                   0 );
    cout << "t^R = " << endl << t_x * R << endl;

    // Check epipolar constraint
    Mat K = ( Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    for(DMatch m: matches)
    {
        Point2d pt1 = pixel2cam( keypoints1[m.queryIdx].pt, K );
        Mat y1 = ( Mat_<double>(3,1) << pt1.x, pt1.y, 1 );
        Point2d pt2 = pixel2cam( keypoints2[m.trainIdx].pt, K );
        Mat y2 = ( Mat_<double>(3,1) << pt2.x, pt2.y, 1 );
        Mat d = y2.t() * t_x * R * y1;
        // cout << "epipolar constraint = " << d << endl;
    }

    return 0;
}
