#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <boost/format.hpp>
#include "cameraCalib.h"

/* Brief: Print program info
 * Parameters:
 *     None
 * Return:
 *     Void
 */
static void help()
{
    cout << "This is a camera calibration program." << endl;
    cout << "Reference: OpenCV camera calibration sample." << endl;
}

/* Brief: import image using cv::imread
 * Parameters:
 *     i -- iterator indicating file name
 * Return:
 *     result -- image in cv::Mat type
 */
Mat fileRead(int i)
{
    Mat result;
    boost::format fmt( "../%s/%d.%s" );	// file format
    result = imread( (fmt%"chessBoard"%(i+1)%"jpg").str(), IMREAD_COLOR );
    return result;
}

static inline void read(const FileNode& node, Settings& x, const Settings& default_value = Settings())
{
    if(node.empty())
        x = default_value;
    else
        x.read(node);
}

int main(int argc, char const *argv[])
{
    help();

    //! [Read configure file]
    Settings s;
    const string inputConfigFileName = argc > 1? argv[1] : "../default.xml";
    FileStorage fs(inputConfigFileName, FileStorage::READ);     // Read configurations
    if (!fs.isOpened())       // Check if configuration file is successfully opened
    {
        cout << "Could not open configuration file: " << inputConfigFileName << endl;
        return -1;
    }
    else
        cout << inputConfigFileName << endl;
    fs["Settings"] >> s;      //? How does this work?  ?//
    fs.release();

    if (!s.validInput)
    {
        cout << "Invalid input occured. Application stopped." << endl;
        return -1;
    }
    //! End of [Read configure file]

    Size imageSize;
    Mat cameraIntrinsics, distCoeffs;
    vector< vector<Point2f> > imagePoints;
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErr;
    double totalAvgErr;

    //! [Get input]
    for(;;)
    {
        Mat view;
        
        view = s.nextImage();

        // namedWindow("chessboard", 0);
        // resizeWindow("chessboard", 500, 500);
        // imshow("chessboard", view);
        // waitKey(100);

        // If view.empty() = true, which indicates that all images in the image list is imported, then calibrate.
        if(view.empty())
        {
            cout << "starting calibration" << endl;
            runCalibration(s, imageSize, cameraIntrinsics, distCoeffs, imagePoints, rvecs, tvecs, reprojErr, totalAvgErr);
            break;
        }
        //! End of [Get input]
        imageSize = view.size();

        //! [Find pattern]
        bool found;
        vector<Point2f> pointBuf;
        found = findChessboardCorners( view, s.boardSize, pointBuf,
                                       CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE );
        //! End of [Find pattern]

        //! [Pattern found]
        // Improve corner's coordinate accuracy for chess board
        if (found)
        {
            Mat viewGray;
            cvtColor(view, viewGray, COLOR_BGR2GRAY);
            cornerSubPix( viewGray, pointBuf, Size(11,11),
                        Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.1 ));
            imagePoints.push_back(pointBuf);
            // Draw the corners
            drawChessboardCorners(view, s.boardSize, Mat(pointBuf), found);
            namedWindow("chessboard", 0);
            resizeWindow("chessboard", 1000, 1000);
            imshow("chessboard", view);
            waitKey(0);
        }
        //! End of [Pattern found]
    }
    
    cout << cameraIntrinsics << endl;
    return 0;
}


// No more