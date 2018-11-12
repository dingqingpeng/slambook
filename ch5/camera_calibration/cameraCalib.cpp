#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <boost/format.hpp>

using namespace std;
using namespace cv;

static void help()
{
    cout << "This is a camera calibration program." << endl;
    cout << "Reference: OpenCV camera calibration sample." << endl;
}

Mat fileRead(int i)
{
    Mat result;
    boost::format fmt( "../%s/%d.%s" );	// file format
    result = imread( (fmt%"chessBoard"%(i+1)%"png").str(), IMREAD_COLOR );
    return result;
}

int main(int argc, char const *argv[])
{
    help();

    // Get input
    for(;;)
    {
        /* code */
    }
    
    return 0;
}
