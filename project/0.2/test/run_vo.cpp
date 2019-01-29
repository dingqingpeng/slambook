#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp>

#include "myslam/visual_odometry.h"
#include "myslam/config.h"

int main(int argc, char const *argv[])
{
    if ( argc != 2 )
    {
        cout<<"usage: run_vo parameter_file"<<endl;
        return 1;
    }

    myslam::Config::setParameterFile( argv[1] );
    myslam::VisualOdometry::Ptr vo( new myslam::VisualOdometry );

    string dataset_dir = myslam::Config::get<string>( "dataset_dir" );
    ifstream fin(dataset_dir + "/associate.txt");
    if( !fin )
    {
        cout<<"please generate the associate file called associate.txt!"<<endl;
        return 1;
    }

    vector<string> rgb_files, depth_files;
    vector<double> rgb_times, depth_times;
    while ( !fin.eof() )
    {
        string rgb_time, rgb_file, depth_time, depth_file;
        fin>>rgb_time>>rgb_file>>depth_time>>depth_file;
        rgb_times.push_back ( atof ( rgb_time.c_str() ) );
        depth_times.push_back ( atof ( depth_time.c_str() ) );
        rgb_files.push_back ( dataset_dir+"/"+rgb_file );
        depth_files.push_back ( dataset_dir+"/"+depth_file );

        if ( fin.good() == false )
            break;
    }

    // visualization
    cv::viz::Viz3d vis("Visual Odometry");
    vis.setWindowSize( cv::Size( 640, 480 ) );
    cv::viz::WCoordinateSystem world_coor(1.0), camera_coor(0.5);
    vis.setBackgroundColor(cv::viz::Color::black());
    cv::Point3d cam_pos( 0.0, -1.0, -1.0 ), cam_focal_point(0,0,0), cam_y_dir(0,1,0);
    cv::Affine3d cam_pose = cv::viz::makeCameraPose( cam_pos, cam_focal_point, cam_y_dir );
    vis.setViewerPose( cam_pose );
    
    world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
    camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
    vis.showWidget( "World", world_coor );
    vis.showWidget( "Camera", camera_coor );

    cv::Point3d point_begin(0.0, 0.0, 0.0);
    cv::Point3d point_end;

    cout<<"read total "<<rgb_files.size() <<" entries"<<endl;
    myslam::Camera::Ptr camera ( new myslam::Camera );
    for(int i = 0; i < rgb_files.size(); i++)
    {
        Mat color = cv::imread ( rgb_files[i] );
        Mat depth = cv::imread ( depth_files[i], cv::IMREAD_UNCHANGED );
        if ( color.data==nullptr || depth.data==nullptr )
            break;
        myslam::Frame::Ptr pFrame = myslam::Frame::createFrame();
        pFrame->camera_ = camera;
        pFrame->color_ = color;
        pFrame->depth_ = depth;
        pFrame->time_stamp_ = rgb_times[i];

        boost::timer timer;
        vo->addFrame ( pFrame );
        cout<<"VO costs time: "<<timer.elapsed()<<endl;

        if ( vo->state_ == myslam::VisualOdometry::LOST )
            break;
        SE3 Tcw = pFrame->T_c_w_.inverse();

        // show the map and the camera pose 
        cv::Affine3d M(
            cv::Affine3d::Mat3( 
                Tcw.rotation_matrix()(0,0), Tcw.rotation_matrix()(0,1), Tcw.rotation_matrix()(0,2),
                Tcw.rotation_matrix()(1,0), Tcw.rotation_matrix()(1,1), Tcw.rotation_matrix()(1,2),
                Tcw.rotation_matrix()(2,0), Tcw.rotation_matrix()(2,1), Tcw.rotation_matrix()(2,2)
            ), 
            cv::Affine3d::Vec3(
                Tcw.translation()(0,0), Tcw.translation()(1,0), Tcw.translation()(2,0)
            )
        );
    
        vector<cv::viz::WLine> lines; 
        point_end = cv::Point3d( Tcw.translation()(0, 0), Tcw.translation()(1, 0), Tcw.translation()(2, 0) );

        cv::viz::WLine line(point_begin, point_end, cv::viz::Color::green());
        lines.push_back(line); //收集  第一个 line 到 当前line
        for (vector<cv::viz::WLine>::iterator iter = lines.begin(); iter != lines.end(); iter++)
        {
            string id = to_string(i);
            vis.showWidget(id, *iter);
            i++;
        }
        point_begin = point_end;

        cv::namedWindow( "image" );
        cv::moveWindow( "image", 20, 1080/2 );
        cv::imshow("image", color );
        cv::waitKey(50);
        vis.setWidgetPose( "Camera", M);
        vis.spinOnce(1, false);

        // char c = 'q';
        // if(i == 0)
        //     while ((c = getchar()) == 'c') ;

        // vis.saveScreenshot("Trajectory" + to_string(i) + ".png");
    }
    
    return 0;
}
