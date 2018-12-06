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
    if( argc != 5 )
    {
        cout << "application usage: feature_extraction img1 img2 depth1 depth2" << endl;
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

    // Construct 3D points and 3D-2D pairs
    Mat depthImg1 = imread(argv[3], CV_LOAD_IMAGE_UNCHANGED);  // Depth image is single channel image, pixels represented by 16 bit unsigned number
    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    vector<Point3f> pts_3d;     // 3D point position under first camera coordinate
    vector<Point2f> pts_2d;     // Projection position of 3D points under second camera coordinate;
    for(DMatch m: matches)
    {
        ushort depth = depthImg1.ptr<ushort>( (int)keypoints1[m.queryIdx].pt.y )[ (int)keypoints1[m.queryIdx].pt.x ];
        if( depth == 0 ) continue;  // Invalid depth
        float d = depth / 5000.0;
        Point2d p1 = pixel2cam(keypoints1[m.queryIdx].pt, K);
        pts_3d.push_back( Point3f( p1.x * d, p1.y * d, d ) );
        pts_2d.push_back( keypoints2[m.trainIdx].pt );
    }
    cout << pts_3d.size() << " 3D-2D pairs found." << endl;

    // Solve PnP problem
    Mat r, t;
    solvePnP( pts_3d, pts_2d, K, Mat(), r, t, false, SOLVEPNP_EPNP );     // flag = SOLVEPNP_ITERATIVE. Is this option Bundle Adjustment?
    Mat R;
    Rodrigues( r, R );  // Convert to matrix

    cout << "-- R = " << endl << R << endl << endl;
    cout << "-- t = " << endl << t << endl << endl;
    cout << "-----------------------------" << endl;

    cout << "Calling bundle adjustment..." << endl;
    bundleAdjustment( pts_3d, pts_2d, K, R, t );

    return 0;
}
