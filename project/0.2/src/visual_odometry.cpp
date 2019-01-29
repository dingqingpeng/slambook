#include "myslam/visual_odometry.h"
#include "myslam/config.h"
#include <opencv2/calib3d/calib3d.hpp>

namespace myslam
{

VisualOdometry::VisualOdometry()
    : state_( INITIALIZING ), map_( new Map ), ref_( nullptr ), cur_( nullptr ), num_inliers_( 0 ), num_lost_( 0 )
{
    num_of_features_     = Config::get<int>   ( "number_of_features" );
    scale_factor_        = Config::get<double>( "scale_factor" );
    level_pyramid_       = Config::get<int>   ( "level_pyramid" );
    match_ratio_         = Config::get<float> ( "match_ratio" );
    min_inliers_         = Config::get<int>   ( "min_inliers" );
    max_num_lost_        = Config::get<int>   ( "max_num_lost" );
    key_frame_min_rot_   = Config::get<double>( "keyframe_rotation" );
    key_frame_min_trans_ = Config::get<double>( "keyframe_translation" );
    orb_ = cv::ORB::create ( num_of_features_, scale_factor_, level_pyramid_ );
}

VisualOdometry::~VisualOdometry() {}

bool VisualOdometry::addFrame( Frame::Ptr frame )
{
    switch ( state_ )
    {
        case INITIALIZING:
            ref_ = cur_ = frame;
            map_->insertKeyFrame( frame );
            extractKeypoints();
            computeDescriptors();
            set3DPoints();
            state_ = RUNNING;
            break;
            
        case RUNNING:
            cur_ = frame;
            extractKeypoints();
            computeDescriptors();
            featureMatching();
            poseEstimationPnP();
            if( checkEstimatedPose() )
            {
                cur_->T_c_w_ = Tcr_estimated_ * ref_->T_c_w_;
                num_lost_ = 0;
                ref_ = cur_;
                set3DPoints();
                if( checkKeyFrame() )
                {
                    addKeyFrame();
                }
            }
            else
            {
                num_lost_ ++;
                if( num_lost_ > max_num_lost_ )
                {
                    state_ = LOST;
                }
                return false;
            }
            break;

        case LOST:
            cout<<"vo has lost."<<endl;
            break;
    
        default:
            break;
    }
    return true;
}

void VisualOdometry::extractKeypoints()
{
    orb_->detect( cur_->color_, keypts_in_cur_ );
}

void VisualOdometry::computeDescriptors()
{
    orb_->compute( cur_->color_, keypts_in_cur_, descriptors_in_cur_ );
}

void VisualOdometry::featureMatching()
{
    vector<cv::DMatch> matches;
    cv::BFMatcher matcher ( cv::NORM_HAMMING );
    matcher.match( descriptors_in_ref_, descriptors_in_cur_, matches );

    // Find minimum and distance of all pairs
    float min_dist = std::min_element( matches.begin(), matches.end(),
                                       [](const cv::DMatch& m1, const cv::DMatch& m2)
                                       { return m1.distance < m2.distance; } )->distance;

    // Consider those pairs wrong, whose distance is greater than twice min_dist;
    // but setting a lower limmit, say 30, is necessary, because minimum distance may sometimes be very small
    feature_matches_.clear();
    for(cv::DMatch& m: matches)
    {
        if (m.distance <= max<float>(match_ratio_*min_dist, 30.0))
        {
            feature_matches_.push_back(m);
        }
    }
    cout<<"good matches: "<<feature_matches_.size()<<endl;
}

void VisualOdometry::set3DPoints()
{
    pts3d_in_ref_.clear();
    descriptors_in_ref_ = Mat();
    for(int i = 0; i < keypts_in_cur_.size(); i++)
    {
        double d = ref_->findDepth( keypts_in_cur_[i] );
        if( d > 0 )
        {
            Vector3d p_cam = ref_->camera_->pixel2camera(
                Vector2d(keypts_in_cur_[i].pt.x, keypts_in_cur_[i].pt.y), d );
            pts3d_in_ref_.push_back( cv::Point3f( p_cam(0,0), p_cam(1,0), p_cam(2,0) ));
            descriptors_in_ref_.push_back(descriptors_in_cur_.row(i));
        }
    }
}

void VisualOdometry::poseEstimationPnP()
{
    // Construct 3D points and 3D-2D pairs
    Mat K = ( cv::Mat_<double> ( 3,3 ) << ref_->camera_->fx_, 0, ref_->camera_->cx_,
                                          0, ref_->camera_->fy_, ref_->camera_->cy_,
                                          0, 0, 1 );
    vector<cv::Point3f> pts_3d;     // 3D point position under first camera coordinate
    vector<cv::Point2f> pts_2d;     // Projection position of 3D points under second camera coordinate;
    for(cv::DMatch m: feature_matches_)
    {
        pts_3d.push_back( pts3d_in_ref_[m.queryIdx] );
        pts_2d.push_back( keypts_in_cur_[m.trainIdx].pt );
    }

    // Solve PnP problem
    Mat r, t, inliers;
    cv::solvePnPRansac( pts_3d, pts_2d, K, Mat(), r, t, false, 100, 4.0, 0.99, inliers );
    num_inliers_ = inliers.rows;
    cout<<"pnp inliers: "<<num_inliers_<<endl;
    Tcr_estimated_ = SE3( SO3(r.at<double>(0,0), r.at<double>(1,0), r.at<double>(2,0)), 
                          Vector3d( t.at<double>(0,0), t.at<double>(1,0), t.at<double>(2,0) ) );
    
    // Mat R;
    // Rodrigues( r, R );  // Convert to matrix
    // cout << "Calling bundle adjustment..." << endl;
    // bundleAdjustment( pts_3d, pts_2d, K, R, t );
}

bool VisualOdometry::checkEstimatedPose()
{
    if ( num_inliers_ < min_inliers_ )
    {
        cout<<"reject because inlier is too small: "<<num_inliers_<<endl;
        return false;
    }
    Sophus::Vector6d d = Tcr_estimated_.log();
    if ( d.norm() > 5.0 )
    {
        cout<<"reject because motion is too large: "<<d.norm()<<endl;
        return false;
    }
    return true;
}

bool VisualOdometry::checkKeyFrame()
{
    Sophus::Vector6d d = Tcr_estimated_.log();
    Vector3d trans = d.head<3>();
    Vector3d rot = d.tail<3>();
    if ( rot.norm() > key_frame_min_rot_ || trans.norm() > key_frame_min_trans_ )
        return true;
    return false;
}

void VisualOdometry::addKeyFrame()
{
    cout<<"adding a key-frame"<<endl;
    map_->insertKeyFrame ( cur_ );
}

}