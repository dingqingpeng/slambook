#include <iostream>
#include <chrono>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

int main(int argc, char const *argv[])
{
	// Import image from argv[1]
	Mat image;
	image = imread( argv[1] );

	// Error process
	if ( image.data == nullptr )
	{
		cerr << "File " << argv[1] << "not found." << endl;
		return 0;
	}

	cout << "image width = " << image.cols << ", height = " << image.rows;
	cout << ", channels = " << image.channels() << endl;
	// namedWindow("image");
	imshow("image", image);
	waitKey(0);

	// Determine image type
	if (image.type() != CV_8UC1 && image.type() != CV_8UC3)
	{
		cout << "Please input a color image or grey image." << endl;
		return 0;
	}

	// Traverse the image
	// Use std::chrono to time the algorithm
	chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
	for (size_t y = 0; y < image.rows; ++y)
	{
		for (size_t x = 0; x < image.cols; ++x)
		{
			// Read (x, y) pixel
			// Use cv::Mat::ptr to get row pointer
			unsigned char* row_ptr = image.ptr<unsigned char> (y);
			unsigned char* data_ptr = &row_ptr[ x * image.channels() ];

			// Output every channel of the pixel
			for (int i = 0; i != image.channels(); ++i)
			{
				unsigned char data = data_ptr[i];
			}
		}
	}

	chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
	chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
	cout << "time consumed: " << time_used.count() << "s." << endl;

	// Direct assignment will not copy data, ( 'cause a cv::Mat object is actually a pointer ) 
	Mat image_another = image;
	// Change of image_another will cause a change in image
	image_another(Rect(0, 0, 100, 100)).setTo(0);
	imshow("image", image);
	waitKey(0);

	// Use clone() to copy data
	Mat image_clone = image.clone();
	image_clone(Rect(0, 0, 100, 100)).setTo(255);
	imshow("image", image);
	imshow("image_clone", image_clone);
	waitKey(0);

	destroyAllWindows();

	return 0;
}