#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <boost/format.hpp>

using namespace std;
using namespace cv;

class Settings
{
public:
    /* Brief: import data from filenode
    * Parameters:
    *     node -- reference to FileNode
    * Return:
    *     Void
    */
    void read(const FileNode& node)
    {
        node["BoardSize_Width"]  >> boardSize.width;
        node["BoardSize_Height"] >> boardSize.height;
        node["Square_Size"]      >> squareSize;
        node["Calibrate_Pattern"]>> patternToUse;
        node["Calibrate_NrOfFrameToUse"] >> nrFrames;
        validate();
    }
    void validate()
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
        
        
    }
public:
    Size  boardSize;     // Size of the board, number of items
    float squareSize;    // The size of a square in some user defined metric system (pixel, millimeter)
    string patternToUse;
    int nrFrames;        // The number of frames to use for calibration
    
    bool validInput;     // True if the setting file passes validation
};

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

int main(int argc, char const *argv[])
{
    help();

    //! [Read configure file]
    Settings s;
    //! End of [Read configure file]

    //! [Get input]
    for(int i = 0; ; ++i)
    {
        Mat view;

        view = fileRead(i);
        if(view.empty())
        {
            break;
        }
        cout << i << ".jpg read" << endl;
    }
    //! End of [Get input]

    //! [Find pattern]
    bool found;
    found = findChessboardCorners( view,  )
    //! End of [Find pattern]
    
    return 0;
}


// No more