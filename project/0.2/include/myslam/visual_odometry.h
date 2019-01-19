#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include "myslam/common_include.h"
#include "myslam/map.h"
#include <opencv2/features2d/features2d.hpp>

namespace myslam
{

class VisualOdometry
{
public:
    VisualOdometry();
    ~VisualOdometry();

    bool addFrame( Frame::Ptr frame );

private:
    void extractKeypoints();
    void computeDescriptors();
    void featureMatching();
    void set3DPoints();
    void poseEstimationPnP();
    bool checkEstimatedPose();
    bool checkKeyFrame();
    void addKeyFrame();

public:
    typedef shared_ptr<VisualOdometry> Ptr;
    enum VOState
    {
        INITIALIZING = -1,
        RUNNING      = 0,
        LOST
    };

    VOState    state_;
    Map::Ptr   map_;
    Frame::Ptr ref_;
    Frame::Ptr cur_;

    cv::Ptr<cv::ORB>     orb_;
    vector<cv::Point3f>  pts3d_in_ref_;
    vector<cv::KeyPoint> keypts_in_cur_;
    Mat                  descriptors_in_ref_;
    Mat                  descriptors_in_cur_;
    vector<cv::DMatch>   feature_matches_;

    SE3 Tcr_estimated_;
    int num_inliers_;
    int min_inliers_;
    int num_lost_;
    int max_num_lost_;
    float match_ratio_;
    double key_frame_min_rot_;
    double key_frame_min_trans_;
};

}

#endif // VISUALODOMETRY_H