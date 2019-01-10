#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <stdio.h>
using namespace cv;
using namespace std;

vector<Mat> Images;
int N_SLICES = 4;
float WEIGHT[4];
int PT[4];

Mat RemoveBackground(Mat image)
{
    Mat mask, resultAnd, resultNot, resultMask;

    Mat lower(1,1, CV_8UC3, Scalar(0,0,0));
	Mat upper(1,1, CV_8UC3, Scalar(100,100,100));
	//cout << "lower = "<< endl << ""  << lower << endl << endl;
	//cout << "upper = "<< endl << ""  << upper << endl << endl;
	inRange(image, lower, upper, mask);
//	cout << "mask = "<< endl << " "  << mask << endl << endl;
//	cout << "image = "<< endl << " "  << image << endl << endl;
//	cout << "in range passed"<< endl;
	bitwise_and(image, image, resultAnd, mask=mask);
	bitwise_not(resultAnd,resultNot,mask);
	
	subtract(Scalar::all(255),resultNot,resultMask);
	return resultMask;
}

Point getContourCenter(vector<Point> contour)
{
	Moments mu;	
  	Point mc;

	// Get the moment
	mu = moments(contour, false); 
    if (mu.m00 == 0)
	{
		return mc;
	}
    //  Get the mass center
  	mc = Point(mu.m10/mu.m00 , mu.m01/mu.m00);
	return mc;	
}

int Aprox(int a, int b, int error)
{
	if (abs(a - b) < error)
	{
		return 1; //True
	}
	else
	{
		return 0; // False	
	}
}

float getContourExtent(vector<Point> contour)
{
	Rect bounding_rect;
	//  Find the area of contour
	double area = contourArea(contour,false);
	// Find the bounding rectangle for the contour  
	bounding_rect=boundingRect(contour); 
	//x,y,w,h = cv2.boundingRect(contour)
	double rect_area = (bounding_rect.width)*(bounding_rect.height);
	if (rect_area > 0)
	{
		return (float(area)/rect_area);
	}
}
	
void Process(Mat img, int index)
{

	int largest_area=0;
	int largest_contour_index=0;
	//cv::Rect bounding_rect;
	Point zero;
	Point contourCenter;
	Point middle;
	int contourCenterX;
	int prev_cX;	
	vector<Point> MainContour;
	vector<Point> prev_MC;
	//cout << contourCenterX << endl;
	
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	Mat imgray,thresh;
	
	cvtColor(img,imgray,COLOR_BGR2GRAY);
	double ret = threshold(imgray,thresh,100,255,THRESH_BINARY_INV);
	findContours(thresh,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE);

	prev_MC = MainContour;

	//cout << contours.size() << endl;
	try
	{
		if(contours.size() != 0)
		{
			// Find the area of contour
			for(int i = 0; i < contours.size(); i++)
			{
		    	if (contourArea(contours[i],false) != 0)
		    	{
		    	double a = contourArea(contours[i],false);  
		    	if(a>largest_area)
			    	{
			        	largest_area=a;
			        	largest_contour_index=i;
			        	MainContour = contours[i];
			        }
		   		}
		    }

		    //cout << "Width : " << img.cols << endl;
			//cout << "Height: " << img.rows << endl;
		
			int width = img.cols;
			int height = img.rows;

			middle.x = int(width/2); //Get X coordenate of the middle point
			middle.y = int(height/2); //Get Y coordenate of the middle point
			
			//cout << middleY << " " << middleX << endl;	
		
			prev_cX = contourCenterX;
			contourCenter = getContourCenter(MainContour);
			if (contourCenter != zero)
			{
				contourCenterX = contourCenter.x;
				if (abs(prev_cX-contourCenterX) > 5)
				{
					for (int i = 0; i < contours.size(); i++)
					{
						if (getContourCenter(contours[i]) != zero)
						{
							int tmp_cx = getContourCenter(contours[i]).x;
							if (Aprox(tmp_cx, prev_cX, 5) == 1)
							{
								MainContour = contours[i];
								if (getContourCenter(MainContour) != zero)
								{
									contourCenterX = getContourCenter(MainContour).x;
								}
							}
						}
					}			
				}
			}
			else
			{
				contourCenterX = 0;
			}
		

			int direction1 =  int((middle.x - contourCenterX) * getContourExtent(MainContour));
			
			Point dX;
			dX.x = contourCenterX;
			dX.y = middle.y;
			drawContours(img,contours,largest_contour_index,Scalar(0,255,0),3);
			circle(img, dX , 7, Scalar(255,255,255), -1); //Draw dX circle WHITE
			circle(img, middle, 3, Scalar(0,0,255), -1); //Draw middle circle RED	

			Point placeholder1;
			Point placeholder2;

			placeholder1.x = (contourCenterX + 20);
			placeholder1.y = middle.y;
			
			placeholder2.x = (contourCenterX + 20);
			placeholder2.y = (middle.y + 35);


			WEIGHT[index] = getContourExtent(MainContour);
			PT[index] = middle.x-contourCenterX;

			String pt = to_string(PT[index]);			
			String weight = "Weight:" + to_string(WEIGHT[index]);
			
			putText(img,pt, placeholder1, FONT_HERSHEY_SIMPLEX, 1, Scalar(200,0,200),2);
			putText(img,weight, placeholder2, FONT_HERSHEY_SIMPLEX , 0.5,Scalar(200,0,200),1);
		}
	}
	catch ( cv::Exception & e )
	{	
	 	//cerr << e.msg << endl; // output exception message
	}	
}

void SlicePart(Mat im, vector<Mat> images, int slices)
{	
	int sl = int(im.rows/slices);	
	
	//cout << sl << endl;
	for (int i = 0; i < slices; i++)
	{
		int part = sl*i;
		//cout << im.rows << endl;
//		crop_img = im[part:part+sl, 0:width] 
		//cout << part << " " << part+sl << endl;
		Mat crop_img = im(Rect(0,part,im.cols,sl));
		Process(crop_img, i);
	}
}
/*0 <= roi.x && 
0 <= roi.width && 
roi.x + roi.width <= m.cols && 
0 <= roi.y && 
0 <= roi.height && 
roi.y + roi.height <= m.rows
*/
int main(int, char**)
{
	
	Mat frame;
	//--- INITIALIZE VIDEOCAPTURE
	VideoCapture cap;
	// open the default camera using default API
	// cap.open(0);
	// OR advance usage: select any API backend
	int deviceID = 0;             // 0 = open default camera
	int apiID = cv::CAP_ANY;      // 0 = autodetect default API
	
	// open selected camera using selected API
	cap.open(deviceID + apiID);
	// check if we succeeded
	if (!cap.isOpened()) 
	{
		cerr << "ERROR! Unable to open camera\n";
		return -1;
	}
	//--- GRAB AND WRITE LOOP
	cout << "Start grabbing" << endl << "Press any key to terminate" << endl;
	for (;;)
	{
		// wait for a new frame from camera and store it into 'frame'
		cap.read(frame);
		// check if we succeeded
		if (frame.empty()) 
		{
			cerr << "ERROR! blank frame grabbed\n";
			break;
		}

		Mat nobg = RemoveBackground(frame);
		// show frame and wait for a key with timeout long enough to show images
		//Process(nobg);
		SlicePart(nobg, Images, N_SLICES);
		imshow("Final", nobg);
		
		for(int i = 0; i < N_SLICES; i++)
		{
			cout << PT[i] << endl;
		}
		
		if (waitKey(5) >= 0)
		{
			break;
		}
	}
	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}