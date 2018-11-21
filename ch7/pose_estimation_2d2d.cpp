#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

void find_feature_matches(const cv::Mat& img1,
                          const cv::Mat& img2,
                          std::vector<cv::KeyPoint>& keypoints1,
                          std::vector<cv::KeyPoint>& keypoints2,
                          std::vector<cv::DMatch>&   matches);

void pose_estimation_2d2d(std::vector<cv::KeyPoint>& keypoints1,
                          std::vector<cv::KeyPoint>& keypoints2,
                          std::vector<cv::DMatch>&   matches,
                          cv::Mat& R,
                          cv::Mat& t);

cv::Point2d pixel2cam( const cv::Point2d& p, const cv::Mat& K );

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
        cout << "epipolar constraint = " << d << endl;
    }

    return 0;
}

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

void pose_estimation_2d2d(std::vector<cv::KeyPoint>& keypoints1,
                          std::vector<cv::KeyPoint>& keypoints2,
                          std::vector<cv::DMatch>&   matches,
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

cv::Point2d pixel2cam( const cv::Point2d& p, const cv::Mat& K )
{
    return Point2d( (p.x - K.at<double>(0,2)) / K.at<double>(0, 0),
                    (p.y - K.at<double>(1,2)) / K.at<double>(1, 1) );
}
