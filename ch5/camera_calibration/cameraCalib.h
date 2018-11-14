#ifndef CAMERACALIB_H_
#define CAMERACALIB_H_

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

class Settings
{
public:
    Settings(): validInput(false) {}
    
    /* Brief: import data from filenode
     * Parameters:
     *     node -- reference to FileNode
     * Return:
     *     Void
     */
    void read(const FileNode& node);

    /* Brief: Validate parameters imported from configuration file
     * Parameters:
     *     None
     * Return:
     *     Void
     */
    void validate();

    bool readStringList( const string& filename, vector<string>& l );

    /* Brief: Import next image from the image list.
     * Parameters:
     *     None
     * Return:
     *     result -- image read from the image list, cv::Mat format.
     */
    Mat nextImage();

public:
    Size  boardSize;     // Size of the board, number of items
    float squareSize;    // The size of a square in some user defined metric system (pixel, millimeter)
    string patternToUse;
    int nrFrames;        // The number of frames to use for calibration
    float aspectRatio;   // fx/fy, where fx is considered as a fixed value
    string input;        // Input image series directory
    
    bool validInput;     // True if the setting file passes validation
    size_t atImageList;  // Counter to determine whether current image is still in image list
    vector<string> imageList;
};

/* Brief: Calibrate camera
 * Parameters:
 *     s -- settings
 *     imageSize -- size of image
 *     corners -- board corners container
 *     cameraIntrinsics -- camera intrinsics
 *     distCoeffs -- distortion coefficients
 *     imagePoints -- container of corner positions on image
 *     rvecs -- output array of rotation vector
 *     tvecs -- output array of translate vector
 *     reprojErr -- container of reprojection error
 *     totalAvgErr -- average of total reprojection error
 * Return:
 *     Void
 */
bool runCalibration( Settings& s, Size& imageSize, Mat& cameraIntrinsics, Mat& distCoeffs,
                     vector< vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
                     vector<float>& reprojErr, double& totalAvgErr );

/* Brief: Validate parameters imported from configuration file
 * Parameters:
 *     boardSize -- size of board, in items, such as 9 blocks * 6 blocks
 *     squareSize -- size of squares in pixels
 *     corners -- container of corner positions on board 
 * Return:
 *     Void
 */
void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners);

#endif