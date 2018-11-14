#include <opencv2/calib3d.hpp>
#include "cameraCalib.h"

void Settings::read(const FileNode& node)
{
    node["BoardSize_Width"]  >> boardSize.width;
    node["BoardSize_Height"] >> boardSize.height;
    node["Square_Size"]      >> squareSize;
    node["Calibrate_Pattern"]>> patternToUse;
    node["Calibrate_NrOfFrameToUse"] >> nrFrames;
    node["Calibrate_FixAspectRatio"] >> aspectRatio;
    node["Input"]                    >> input;
    validate();
}

void Settings::validate()
{
    validInput = true;
    
    if (boardSize.width <= 0 || boardSize.height<= 0) 
    {
        cerr << "Invalid board size." << endl;
        validInput = false;
    }
    if (squareSize < 1e-6)
    {
        cerr << "Invalid square size." << endl;
        validInput = false;
    }
    if (nrFrames <= 0)
    {
        cerr << "Invalid number of frame." << endl;
        validInput = false;
    }

    if (readStringList(input, imageList))
    {
        nrFrames = (nrFrames > (int)imageList.size())? nrFrames: (int)imageList.size();
    }
    

    atImageList = 0;        // If validation passes, then set counter to 0.
}

Mat Settings::nextImage()
{
    Mat result;

    if( atImageList < imageList.size() )
        result = imread(imageList[atImageList++], IMREAD_COLOR);

    return result;
}

bool Settings::readStringList( const string& filename, vector<string>& l )
{
    l.clear();
    FileStorage fs(filename, FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != FileNode::SEQ )
        return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
        l.push_back((string)*it);
    return true;
}

bool runCalibration( Settings& s, Size& imageSize, Mat& cameraIntrinsics, Mat& distCoeffs ,
                     vector< vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
                     vector<float>& reprojErr, double& totalAvgErr )
{
    cameraIntrinsics = Mat::eye(3, 3, CV_64F);
    //! [Check if input image has fixed aspect ratio]
    if(s.aspectRatio)
        cameraIntrinsics.at<double>(0, 0) = s.aspectRatio;
    //! End of [Check if input image has fixed aspect ratio]
    distCoeffs = Mat::zeros(8, 1, CV_64F);

    //! [Find corner positions on chess board]
    vector< vector<Point3f> > objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0]);
    //! End of [Find corner positions on chess board]

    //! [Find camera intrinsics and extrinsics]
    objectPoints.resize(imagePoints.size(), objectPoints[0]);
    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraIntrinsics, distCoeffs,
                                 rvecs, tvecs, CALIB_FIX_K4|CALIB_FIX_K5);
    cout << "Reprojection error reported by calibrateCamera: " << rms << endl;
    //! End of [Find camera intrinsics and extrinsics]

    bool ok = checkRange(cameraIntrinsics) && checkRange(distCoeffs);
    cout << (ok? "Calibration succeeded": "Calibration failed") << endl;
    // totalAvgErr = computeReprojectionErrors()

    return ok;
}

void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners)
{
    for(int i = 0; i < boardSize.height; i++)
    {
        for(int j = 0; j < boardSize.width; j++)
        {
            corners.push_back(Point3f( j*squareSize, i*squareSize, 0 ));
        }
    }
}
