#include "single_eye_body.h"

#include <time.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <asm/types.h>
#include <linux/videodev2.h>

#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <pthread.h>
#include <stdbool.h>
#include <thread>

//#include <jpeglib.h>

 /* OpenCV library */
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/opencv_modules.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <iomanip>

//ros
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/UInt8.h>


#define RESIZE_WIDTH  320
#define RESIZE_HEIGHT 240
#define SECOND_TO_MRCROSECOND     (1000000)

#define TRUE	1
#define FALSE	0
#define FAILURE	3
#define SUCCESS_USER	2

typedef signed char         int8;   // 8 bits signed integer
typedef unsigned char       uint8;  // 8 bits unsigned integer
typedef short int           int16;  // 16 bits signed integer
typedef unsigned short int  uint16; // 16 bits unsigned integer
typedef int					int32;  // 32bits signed integer
typedef unsigned int		uint32; // 32bits unsigned integer
typedef float               float32;// 32 bits floating point

#define MESSAGE_PRINTF printf
/* OpenCV library */


#define VIDEO_FILE_NAME "/data/workspace/laneDetects/LaneMarkings_Detection-ok/VideoTest20200418-ok.avi"
//#define VIDEO_FILE_NAME "/data/video/smartcar.mp4"
#define CASCADE_FILE_NAME "/data/workspace/laneDetects/test/Car_lane_sign_detection-master/cars3.xml"
//#define CASCADE1_FILE_NAME "/data/workspace/laneDetects/test/Car_lane_sign_detection-master/traffic_light.xml"
//#define CASCADE2_FILE_NAME "/data/workspace/laneDetects/test/Car_lane_sign_detection-master/stop_sign.xml"
//#define CASCADE3_FILE_NAME "/data/workspace/laneDetects/test/Car_lane_sign_detection-master/pedestrian.xml"
#define CASCADE4_FILE_NAME "/data/workspace/laneDetects/test/Car_lane_sign_detection-master/left-sign.xml"
#define CASCADE5_FILE_NAME "/data/workspace/laneDetects/test/Car_lane_sign_detection-master/right-sign.xml"

#define CAR_IMAGE "/home/robot/robot_ws/src/smart_car_mc110/car.png"
#define LEFT_SIGN_IMAGE "/home/robot/robot_ws/src/smart_car_mc110/left.png"
#define RIGHT_SIGN_IMAGE "/home/robot/robot_ws/src/smart_car_mc110/right.png"


using namespace cv;
using namespace std;
using namespace ros;

single_eye_body::single_eye_body(ros::NodeHandle &n)
{
	imgTrans = new image_transport::ImageTransport(n);
	imgPub = imgTrans->advertise("/smart_car_mc110/single_eye/img", 10);
	ctanSlopPub = n.advertise<std_msgs::UInt8>("/smart_car_mc110/single_eye/ctanSlop", 10);

	thread detechThread(&single_eye_body::laneDectionThreadHandler, this);
	detechThread.detach();
}

single_eye_body::~single_eye_body()
{
	delete imgTrans;
}






void single_eye_body::cvDetectLane(Mat &mFrame)
{
	Mat mGray, mCanny, imageROI,mGray1, mGray2, carTrack , mask, IPM_ROI, IPM, IPM_Gray, IPM1, IPM2 ,IPM_Gray2, mFrame2;
	CascadeClassifier cars, traffic_light, stop_sign, pedestrian,sign, sign2;
	vector<Rect> cars_found, traffic_light_found, stop_sign_found ,pedestrian_found ,sign_found, sign_found2, cars_tracking;
	vector<Mat> cars_tracking_img;
	vector<int> car_timer;

	//cars.load(CASCADE_FILE_NAME);
	// traffic_light.load(CASCADE1_FILE_NAME);
	// stop_sign.load(CASCADE2_FILE_NAME);
	// pedestrian.load(CASCADE3_FILE_NAME);
	// sign.load(CASCADE4_FILE_NAME);
	// sign2.load(CASCADE5_FILE_NAME);

	int level=0,a=mFrame.rows;

	// Variable for storing video frames
	Mat frame;

	// Apply the classifier to the frame
	mFrame2 = mFrame.clone();
	imageROI = mFrame(Rect(0,mFrame.rows/2,mFrame.cols,mFrame.rows/2));
	//IPM_ROI = imageROI(Rect(0,65,imageROI.cols,(imageROI.rows-65)));
	IPM_ROI = imageROI(Rect(0,0,imageROI.cols,(imageROI.rows-65)));
	IPM_ROI = IPM_ROI.clone();

	cvtColor(imageROI, mGray, COLOR_BGR2GRAY);
	cvtColor(mFrame, mGray2, COLOR_BGR2GRAY);

	mGray.copyTo(mGray1);
	equalizeHist(mGray, mGray);  //直方图均衡化，用于提高图像的质量

	//cars cascade
	// cars.detectMultiScale(mGray, cars_found, 1.1, 5, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));
	// draw_locations(mFrame, cars_found, Scalar(0, 255, 0),"Car");
	// draw_locations(mFrame2, cars_found, Scalar(0, 255, 0),"Car");

	//        //traffic lights cascade
	//        traffic_light.detectMultiScale(mGray, traffic_light_found, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));
	//        draw_locations(mFrame, traffic_light_found, Scalar(0, 255, 255),"traffic light");
	//
	//        //stop sign cascade
	//        stop_sign.detectMultiScale(mGray, stop_sign_found, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));
	//        draw_locations(mFrame, stop_sign_found, Scalar(0, 0, 255),"Stop Sign");
	//
	//
	//        //pedestrian cascade
	//        pedestrian.detectMultiScale(mGray, pedestrian_found, 1.1, 1, 0 | CASCADE_SCALE_IMAGE, Size(20,50));
	//        draw_locations(mFrame, pedestrian_found, Scalar(255, 0, 0),"Pedestrian");

	//stop sign cascade
	// sign.detectMultiScale(mGray2, sign_found, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));
	// draw_locations(mFrame, sign_found, Scalar(0, 143, 255),"Left Arrow");

	// sign2.detectMultiScale(mGray2, sign_found2, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));
	// draw_locations(mFrame, sign_found2, Scalar(0, 143, 255),"Right Arrow");

	Point2f inputQuad[4];
	Point2f outputQuad[4];

	Mat IPM_Matrix( 2, 4, CV_32FC1 );
	Mat IPM_Matrix_inverse;

	// Set the IPM_Matrix matrix the same type and size as input
	IPM_Matrix = Mat::zeros( mFrame.rows, mFrame.cols, mFrame.type() );

	// The 4 points that select quadilateral on the input , from top-left in clockwise order
	// These four pts are the sides of the rect box used as input
	inputQuad[0] = Point2f( 0,0);
	inputQuad[1] = Point2f( IPM_ROI.cols,0);
	inputQuad[2] = Point2f( IPM_ROI.cols,IPM_ROI.rows);
	inputQuad[3] = Point2f( 0,IPM_ROI.rows);           //
	// The 4 points where the mapping is to be done , from top-left in clockwise order
	outputQuad[0] = Point2f( 0,0 );
	outputQuad[1] = Point2f( mFrame.cols,0);
	outputQuad[2] = Point2f( mFrame.cols-120,mFrame.rows);
	outputQuad[3] = Point2f( 120,mFrame.rows);

	// Get the Perspective Transform Matrix i.e. IPM_Matrix
	IPM_Matrix = getPerspectiveTransform( inputQuad, outputQuad );
	invert(IPM_Matrix,IPM_Matrix_inverse);

	// Apply the Perspective Transform just found to the src image
	warpPerspective(IPM_ROI,IPM,IPM_Matrix,mFrame.size() );

	cvtColor(IPM, IPM_Gray, COLOR_BGR2GRAY);
	GaussianBlur(IPM_Gray, IPM_Gray, Size(7,7), 1.5, 1.5);
	Canny(IPM_Gray, IPM_Gray, 5, 40, 3);

	//Canny(IPM_Gray, IPM_Gray, 100, 200, 3);
	IPM.copyTo(IPM1);
	IPM.copyTo(IPM2);


	//nested loops to eliminate the angled "lines" edges and trim the IPM

	for (int i=0; i<IPM_Gray.rows; i++){
		uchar* data= IPM_Gray.ptr<uchar>(i);
		for (int j=0; j<IPM_Gray.cols; j++)
		{
			if(i<0 || i>480)
			{
				// process each pixel
				data[j]= data[j]>level?level:0;
			}else{
				if(data[j]<=255 && data[j]>240 ){
					for(int m=j;m<j+20;m++){
						a=m;
						data[m]=0;
					}
					j=a;
					break;
				}
			}
		}
	}

	for (int i=0; i<IPM_Gray.rows; i++){
		uchar* data= IPM_Gray.ptr<uchar>(i);
		for(int j=IPM_Gray.cols;j>0;j--){
			if(data[j]<=255 && data[j]>240){
				for(int m=j;m>j-20;m--){
					data[m]=0;
				}
				j=j-20;
				break;
			}
		}
	}

	GaussianBlur( IPM_Gray,IPM_Gray, Size( 5, 5 ), 1.5, 1.5 );

	vector<Vec4i> lines;
	HoughLinesP(IPM_Gray,lines,1, 0.01, 120 ,10,600 );
	// HoughLinesP(IPM_Gray,lines,1, 0.01, 120  );

	vector<Point> laneShade,laneShade1,laneShade2;
	float d=0.00,d1=0.00;
	int s=0;
	int n=mFrame.cols;
	Point e,f,g,h,A,B,C,D;
	float angle;float atmp;
	for( size_t i = 0; i < lines.size(); i++ ){
		float p=0,t=0;
		Vec4i l = lines[i];
		if((l[0]-l[2])==0){
			atmp=-CV_PI/2;
			angle=-90;
		}else{
			t=(l[1]-l[3])/(l[0]-l[2]);
			atmp=atan(t);
			angle=atmp*180/CV_PI;
		}

		if(angle>50 ||  angle<(-50)){

			p=(l[0]+l[2])/2;
			line(IPM1,Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,0,0), 3,CV_AA);
			if(p<RESIZE_WIDTH/2){
				if(p>s){
					s=p;
					d=320-(s);
					e=Point(l[0],l[1]);
					f=Point(l[2],l[3]);
					A= GetWrappedPoint(IPM_Matrix_inverse,e);
					B =GetWrappedPoint(IPM_Matrix_inverse,f);
					A.y += 245;
					B.y += 245;

					double lengthAB = sqrt((A.x - B.x)*(A.x - B.x) + (A.y - B.y)*(A.y - B.y));
					if(A.y > B.y){
					A.x = B.x + (B.x - A.x) / lengthAB * -350;
					A.y = B.y + (B.y - A.y) / lengthAB * -350;

					}else{
						B.x = B.x + (B.x - A.x) / lengthAB * 350;
						B.y = B.y + (B.y - A.y) / lengthAB * 350;

					}

				}

			}
			if(p>RESIZE_WIDTH/2){
				if(p<n){
					n=p;
					d1=(n)-320;
					g=Point(l[0],l[1]);
					h=Point(l[2],l[3]);
					C= GetWrappedPoint(IPM_Matrix_inverse,g);
					C.y +=245;
					D =GetWrappedPoint(IPM_Matrix_inverse,h);
					D.y +=245;
					double lengthCD = sqrt((C.x - D.x)*(C.x - D.x) + (C.y - D.y)*(C.y - D.y));
					if(C.x > D.x){
					C.x = D.x + (D.x - C.x) / lengthCD * -350;
					C.y = D.y + (D.y - C.y) / lengthCD * -350;
					}else{
						D.x = D.x + (D.x - C.x) / lengthCD * +350;
						D.y = D.y + (D.y - C.y) / lengthCD * +350;

					}
				}

			}

		}
	}


	line(IPM2,e, f, Scalar(255,0,0), 3,CV_AA);  //scalar 将图像设置成单一灰度和颜色
	line(IPM2,g, h, Scalar(0,0,255), 3,CV_AA);

	/**************************************************************************************************************/
	#if 1
	#define MaxLeftSlope   0.8
	#define MaxRightSlope  0.8
	unsigned int tanSlope = 0;
	unsigned char ctanFirstSlop = 0;
	static  unsigned char  ctanSloptmp = 100;

	float tanLeftSlop  = 0.0;
	float tanRightSlop = 0.0;
	static float tanLeftSlopTmp  = 0.5;
	static float tanRightSlopTmp = 0.5;
	int   leftLineNum  = 0;
	int   rightLineNum = 0;

	if( ((e.x == 0) && (e.y == 0)) && ((f.x == 0) && (f.y == 0)) ){
		leftLineNum = 0;
	}
	else{
		leftLineNum = 1;
	}

	if( ((g.x == 0) && (g.y == 0)) && ((g.x == 0) && (g.y == 0)) ){
		rightLineNum = 0;
	}
	else{
		rightLineNum = 1;
	}

	if(leftLineNum == 1){
		tanLeftSlop  = ((f.x - e.x)*1.0) / ((f.y - e.y)*1.0);
		if(((f.x - e.x) ==0) || ((f.y - e.y) ==0)){
			tanLeftSlop = tanLeftSlopTmp;
		}
		tanLeftSlopTmp = tanLeftSlop;
	}
	else{
		tanLeftSlop = MaxLeftSlope;
	}
	if(rightLineNum == 1){
		tanRightSlop = ((h.x - g.x)*1.0) / ((g.y - h.y)*1.0);
		if(((h.x - g.x) ==0) || ((g.y - h.y) ==0)){
			tanRightSlop = tanRightSlopTmp;
		}
		tanRightSlopTmp = tanRightSlop;

	}
	else{
		tanRightSlop = MaxRightSlope;
	}



	tanSlope = (unsigned int)(fabsf(tanLeftSlop / tanRightSlop) * 100);
	if ((tanSlope / 5 + 80) >= 255){
		ctanFirstSlop = 255;
	}
	else{
		ctanFirstSlop = tanSlope / 5 + 80;
	}

	if((leftLineNum == 0) && (rightLineNum == 0))
	{
		ctanFirstSlop  = 100;
	}
//	printf("[left]e.x = %03d, e.y = %03d, f.x = %03d, f.y = %03d, [Right] g.x = %03d, g.y = %03d, h.x = %03d, h.y = %03d;"
//			" tanLeftSlop = %f, tanRightSlop = %f, tanSlope = %d, ",
//			e.x, e.y, f.x, f.y, g.x, g.y, h.x, h.y, tanLeftSlop, tanRightSlop, tanSlope );

	//printf("ctanFirstSlop=%d\n", ctanFirstSlop);
 	#endif

	/***************************************************************************************/


	/***********************************************第一次滤波,过滤波动较大的数据**************************************/
	/*限幅滤波法(又称程序判断滤波法)
	  A.方法:
	   根据经验判断,确定两次采样允许的最大偏差值(设为A)
	   每次检测到新值时判断:
	   如果本次值与上次值之差<=A,则本次值有效
	   如果本次值与上次值之差>A,则本次值无效,放弃本次值,用上次值代替本次值
	  B.优点:
	   能有效克服因偶然因素引起的脉冲干扰
	  c.缺点:
	   无法抑制那种周期性的干扰
	   平滑度差
	*/
#if 1
	static  unsigned char  ctanFirstLastSlop  = 100;
 	static  int  ctanFirstCount = 0;
	#define DiffCtanFristSlopValue  75
	unsigned char ctanSecondSlop = 0;

	if (ctanFirstCount == 0){
		//printf("1111");
		ctanFirstLastSlop = ctanFirstSlop;
		ctanSecondSlop    = ctanFirstSlop;
		//printf("ctanFirstSlop = %d, ctanFirstLastSlop = %d, ctanSecondSlop=%d \n", ctanFirstSlop, ctanFirstLastSlop, ctanSecondSlop);
	}
	else{
		if ( ctanFirstSlop - ctanFirstLastSlop >= 0 )
		{
			if ((ctanFirstSlop - ctanFirstLastSlop  <= DiffCtanFristSlopValue)){
				//printf("2222");
				ctanSecondSlop    = ctanFirstSlop;
			}
			else if ((ctanFirstSlop - ctanFirstLastSlop > DiffCtanFristSlopValue)){
				//printf("3333");
				ctanSecondSlop    = ctanFirstLastSlop;
			}
			else{

			}
		}
		else{
			if (ctanFirstLastSlop - ctanFirstSlop <= DiffCtanFristSlopValue){
				//printf("4444");
				ctanSecondSlop  = ctanFirstSlop;
			}
			else if(ctanFirstLastSlop - ctanFirstSlop > DiffCtanFristSlopValue){
				//printf("5555");
				ctanSecondSlop  = ctanFirstLastSlop;
			}
			else{

			}
		}



		//printf("ctanFirstSlop = %d, ctanFirstLastSlop = %d, ctanSecondSlop=%d \n", ctanFirstSlop, ctanFirstLastSlop, ctanSecondSlop);

		ctanFirstLastSlop = ctanSecondSlop;  //ctanFirstSlop;
	}
	ctanFirstCount ++;
	//printf("ctanSecondSlop=%d\n", ctanSecondSlop);

#endif
	/***********************************************第一次滤波,过滤波动较大的数据**********************************/



	/********************************************第二次滤波,再次过滤波动较大的数据**********************************/
#if 1
 	static unsigned char ctanSecondLastSlop = 100;
	unsigned char ctanThreeSlop = 0;

	#define DiffCtanSecondSlopValue  5
	#define DiffCtanSecondSlopKey    0.9

	if ((ctanSecondSlop - ctanSecondLastSlop > DiffCtanSecondSlopValue)){
		//printf("1111");
		ctanThreeSlop = ctanSecondLastSlop * DiffCtanSecondSlopKey + ctanSecondSlop * (1 - DiffCtanSecondSlopKey);
	}
	if ((ctanSecondLastSlop - ctanSecondSlop > DiffCtanSecondSlopValue)){
		//printf("2222");
		ctanThreeSlop = ctanSecondSlop * DiffCtanSecondSlopKey + ctanSecondLastSlop * (1 - DiffCtanSecondSlopKey);
	}
	else{
		//printf("3333");
		ctanThreeSlop = ctanSecondSlop;
	}

	ctanSecondLastSlop = ctanThreeSlop;
 	//printf("ctanSecondSlop = %d, ctanSecondLastSlop = %d, ctanThreeSlop=%d \n", ctanSecondSlop, ctanSecondLastSlop, ctanThreeSlop);
	//printf("ctanThreeSlop=%d\n",  ctanThreeSlop);

#endif
 	/********************************************第二次滤波,再次过滤波动较大的数据**********************************/


	/********************************************第三次滤波,分阶段过滤波动较大的数据********************************/
#if 1
//	#define DoubleK1  1.2   // ctanSlop < 90
//	#define DoubleK2  1.0   // 90  <= ctanSlop < 110
//	#define DoubleK3  0.8	// 110 <= ctanSlop < 130
//	#define DoubleK4  0.8	// 130 <= ctanSlop < 150
//	#define DoubleK5  0.6	// 150 <= ctanSlop < 170
//	#define DoubleK6  0.4	// 170 <= ctanSlop < 190
//	#define DoubleK7  0.1	// 190 <= ctanSlop <= 255

	#define DoubleK1  1.0   // 95  <= ctanSlop < 105
	#define DoubleK2  1.0   // 105 <= ctanSlop < 115
	#define DoubleK3  0.01	// 115 <= ctanSlop < 135
	#define DoubleK4  1.0	// 200 <= ctanSlop <= 210
	#define DoubleK5  0.9	// 210 <= ctanSlop <= 255

	#define SingleK1  1.0   // ctanSlop < 120
	#define SingleK2  1.0   // 120  <= ctanSlop < 150
	#define SingleK3  1.0   // 150  <= ctanSlop < 180
	#define SingleK4  0.9   // 180  <= ctanSlop < 210
	#define SingleK5  0.01   // 210  <= ctanSlop < 240
	#define SingleK6  0.01   // 240  <= ctanSlop < 255
	static unsigned  int tanSlopetemp = 100;

	if((leftLineNum == 0) && (rightLineNum == 0)){  // no lanes exist
		printf("noLanes");
		ctanSlop = 100;
	}

	if((leftLineNum == 1) && (rightLineNum == 1)){  // two lanes exist
		printf("TwoLanes");
		if(ctanThreeSlop < 95)
		{
			ctanSlop = 95;
		}
		else if((ctanThreeSlop >= 95) && (ctanThreeSlop < 105)){
			ctanSlop = ctanThreeSlop * DoubleK1;
		}
//		else if((ctanThreeSlop >= 105) && (ctanThreeSlop < 115)){
//			ctanSlop = 95 * DoubleK1 + (ctanThreeSlop - 105)* DoubleK2;
//		}
//		else if((ctanThreeSlop >= 115) && (ctanThreeSlop < 135)){
//			ctanSlop = 95 * DoubleK1 + 20*DoubleK2 + (ctanThreeSlop - 115)* DoubleK3;
//		}
		else if((ctanThreeSlop >=200) && (ctanThreeSlop <= 210)){
			ctanSlop = ctanThreeSlop*DoubleK4;
		}
		else if ((ctanThreeSlop >=210) && (ctanThreeSlop <= 255)){
			ctanSlop = ctanThreeSlop*DoubleK5;
		}
		else{
			ctanSlop  = 105;
		}


 		}
		else{  // one lanes exist or no lanes exist
			printf("OneLanes");
			if ((ctanThreeSlop < 120)){
				//printf("single111");
				ctanSlop = ctanThreeSlop * SingleK1;
			}
			else if ((ctanThreeSlop >= 120) && (ctanThreeSlop < 150)){
				//printf("single222");
				ctanSlop =  120*SingleK1 + (ctanThreeSlop-120) * SingleK2;
			}
			else if ((ctanThreeSlop >= 150) && (ctanThreeSlop < 180)){
				//printf("single333");
				ctanSlop = 120*SingleK1 + 30*SingleK2 + (ctanThreeSlop-150) * SingleK3;
			}
			else if ((ctanThreeSlop >= 180) && (ctanThreeSlop < 210)){
				//printf("single444");
				ctanSlop = 120*SingleK1 + 30*(SingleK2 + SingleK3)  + (ctanThreeSlop-180) * SingleK4;
			}
			else if ((ctanThreeSlop >= 210) && (ctanThreeSlop < 240)){
				//printf("single555");
				ctanSlop = 120*SingleK1 + 30*(SingleK2 + SingleK3 + SingleK4) + (ctanThreeSlop-210) * SingleK5;
			}
			else if ((ctanThreeSlop >= 240) && (ctanThreeSlop <= 255)){
				//printf("single555");
				ctanSlop = 120*SingleK1 + 30*(SingleK2 + SingleK3 + SingleK4 + SingleK5) + (ctanThreeSlop-240) * SingleK6;
			}
			else{

			}
 		}



 	//printf("ctanThreeSlop = %d,ctanSlop=%d\n",  ctanThreeSlop, ctanSlop);
	printf("ctanSlop=%d\n",   ctanSlop);
#endif


	/********************************************第三次滤波,分阶段过滤波动较大的数据********************************/

	if(A.x < B.x){
		laneShade.push_back(B);
		laneShade.push_back(A);
	}else{
		laneShade.push_back(A);
		laneShade.push_back(B);
	}

	if(C.x > D.x){
		laneShade.push_back(C);
		laneShade.push_back(D);
	}else{
		laneShade.push_back(D);
		laneShade.push_back(C);
	}

	laneShade1.push_back(Point((laneShade[0].x+laneShade[3].x)/2,laneShade[0].y+20));
	laneShade1.push_back(Point((laneShade[0].x+laneShade[3].x)/2 +45,laneShade[1].y));
	laneShade1.push_back(Point((laneShade[0].x+laneShade[3].x)/2 -45,laneShade[2].y));
	laneShade1.push_back(Point((laneShade[0].x+laneShade[3].x)/2,laneShade[3].y+20));

	laneShade2.push_back(Point((laneShade[0].x+laneShade[3].x)/2,laneShade[0].y+20));
	laneShade2.push_back(Point((laneShade[0].x+laneShade[3].x)/2 +25,laneShade[2].y));
	laneShade2.push_back(Point((laneShade[0].x+laneShade[3].x)/2 -25,laneShade[2].y));
	laneShade2.push_back(Point((laneShade[0].x+laneShade[3].x)/2,laneShade[3].y+20));


	Point zero  = Point(0,0);
	if(laneShade[0]!=zero && laneShade[1]!=zero && laneShade[2]!=zero && laneShade[3]!=zero && laneShade[2].y>0){
		Mat laneMask= mFrame.clone();
		fillConvexPoly(laneMask, laneShade, Scalar(0,200,0));  //(255,144,30)
		fillConvexPoly(mFrame, laneShade1, Scalar(0,200,0));
		fillConvexPoly(mFrame, laneShade2, Scalar(255,255,255));
		addWeighted(mFrame, 0.6, laneMask, 0.4, 3, mFrame);
	}



 }
 



double single_eye_body::getPSNR(const Mat& I1, const Mat& I2)
{
    Mat s1;
    absdiff(I1, I2, s1);       // |I1 - I2|
    s1.convertTo(s1, CV_32F);  // cannot make a square on 8 bits
    s1 = s1.mul(s1);           // |I1 - I2|^2
    
    Scalar s = sum(s1);         // sum elements per channel
    
    double sse = s.val[0] + s.val[1] + s.val[2]; // sum channels
    
    if( sse <= 1e-10) // for small values return zero
        return 0;
    else
    {
        double  mse =sse /(double)(I1.channels() * I1.total());
        double psnr = 10.0*log10((255*255)/mse);
        return psnr;
    }
}

Point single_eye_body::GetWrappedPoint(Mat M, const Point& p)
{
    cv::Mat_<double> src(3/*rows*/,1 /* cols */);
    
    src(0,0)=p.x;
    src(1,0)=p.y;
    src(2,0)=1.0;
    
    cv::Mat_<double> dst = M*src;
    dst(0,0) /= dst(2,0);
        dst(1,0) /= dst(2,0);
    return Point(dst(0,0),dst(1,0));
}

void single_eye_body::draw_locations(Mat & img, vector< Rect > &locations, const Scalar & color, string text)
{

    Mat img1, car, carMask ,carMaskInv,car1,roi1, LeftArrow , LeftMask, RightArrow,RightMask;


    img.copyTo(img1);
    string dis;

	if (!locations.empty())
	{

        double distance= 0;
        
        for( int i = 0 ; i < locations.size() ; ++i){
            
            if (text=="Car"){
                car = imread(CAR_IMAGE);
                carMask = car.clone();
                cvtColor(carMask, carMask, CV_BGR2GRAY);
                locations[i].y = locations[i].y + img.rows/2; // shift the bounding box
                distance = (0.0397*2)/((locations[i].width)*0.00007);// 2 is avg. width of the car
                Size size(locations[i].width/1.5, locations[i].height/3);
                resize(car,car,size, INTER_NEAREST);
                resize(carMask,carMask,size, INTER_NEAREST);
                Mat roi = img.rowRange(locations[i].y-size.height, (locations[i].y+locations[i].height/3)-size.height).colRange(locations[i].x, (locations[i].x  +locations[i].width/1.5));
                bitwise_and(car, roi, car);
                car.setTo(color, carMask);
                add(roi,car,car);
                car.copyTo(img1.rowRange(locations[i].y-size.height, (locations[i].y+locations[i].height/3)-size.height).colRange(locations[i].x, (locations[i].x  +locations[i].width/1.5)));
                
            }else if((text=="Pedestrian")){
                distance = (0.0397*0.5)/((locations[i].width)*0.00007);//0.5 is avg. width of a person
            }else if((text=="Stop Sign")){
                distance = (0.0397*0.75)/((locations[i].width)*0.00007);//0.75 is avg. width of the stopsign
            }else if((text=="Left Arrow")){
                LeftArrow = imread(LEFT_SIGN_IMAGE);
                LeftMask = LeftArrow.clone();
                cvtColor(LeftMask, LeftMask, CV_BGR2GRAY);
                //locations[i].y = locations[i].y + img.rows/2; // shift the bounding box
                Size size(locations[i].width/2, locations[i].height/1.5);
                resize(LeftArrow,LeftArrow,size, INTER_NEAREST);
                resize(LeftMask,LeftMask,size, INTER_NEAREST);
                distance = (0.0397*0.4)/((locations[i].width)*0.00007);//0.35 is avg. width of the   Chevron Arrow sign

                if (locations[i].y-size.height>0){
                    
                Mat roi1 = img.rowRange(locations[i].y-size.height,(locations[i].y+locations[i].height/1.5)-size.height).colRange(locations[i].x+5, (locations[i].x+5+locations[i].width/2));
                bitwise_and(LeftArrow, roi1, LeftArrow);
                LeftArrow.setTo(color, LeftMask);
                add(roi1,LeftArrow,LeftArrow);
                LeftArrow.copyTo(img1.rowRange(locations[i].y-size.height,(locations[i].y+locations[i].height/1.5)-size.height).colRange(locations[i].x+5 ,(locations[i].x +5+locations[i].width/2 )));
                }
                
            }else if((text=="Right Arrow")){
                RightArrow = imread(RIGHT_SIGN_IMAGE);
                RightMask = RightArrow.clone();
                cvtColor(RightMask, RightMask, CV_BGR2GRAY);
                //locations[i].y = locations[i].y + img.rows/2; // shift the bounding box
                Size size(locations[i].width/2, locations[i].height/1.5);
                resize(RightArrow,RightArrow,size, INTER_NEAREST);
                resize(RightMask,RightMask,size, INTER_NEAREST);
                distance = (0.0397*0.4)/((locations[i].width)*0.00007);//0.35 is avg. width of the   Chevron Arrow sign

                if (locations[i].y-size.height>0){
                    
                    Mat roi1 = img.rowRange(locations[i].y-size.height,(locations[i].y+locations[i].height/1.5)-size.height).colRange(locations[i].x+5, (locations[i].x+5+locations[i].width/2));
                    bitwise_and(RightArrow, roi1, RightArrow);
                    RightArrow.setTo(color, RightMask);
                    add(roi1,RightArrow,RightArrow);
                    RightArrow.copyTo(img1.rowRange(locations[i].y-size.height,(locations[i].y+locations[i].height/1.5)-size.height).colRange(locations[i].x+5 ,(locations[i].x +5+locations[i].width/2 )));
                }
                
            }
            stringstream stream;
            stream << fixed << setprecision(2) << distance;
            dis = stream.str() + "m";
            rectangle(img,locations[i], color, -1);
        }
        addWeighted(img1, 0.8, img, 0.2, 0, img);
        
        for( int i = 0 ; i < locations.size() ; ++i){
        
            rectangle(img,locations[i],color,1.8);
            
            putText(img, text, Point(locations[i].x+1,locations[i].y+8), FONT_HERSHEY_DUPLEX, 0.3, color, 1);
            putText(img, dis, Point(locations[i].x,locations[i].y+locations[i].height-5), FONT_HERSHEY_DUPLEX, 0.3, Scalar(255, 255, 255), 1);
            
            
            if (text=="Car"){
                locations[i].y = locations[i].y - img.rows/2; // shift the bounding box
            }
        
        }
        
	}
}




void single_eye_body::laneDectionThreadHandler()
{
 	Mat mFrame(Size(RESIZE_WIDTH, RESIZE_HEIGHT), CV_8UC3);  // img waited to process
	Mat originFrame; // to save origin img
	cv_bridge::CvImage cvImg;
	ctanSlop = 0;
	static bool quit = false;

 	VideoCapture capture; 
	capture.open("/dev/video10"); //如果是笔记本，0打开的是自带的摄像头，1 打开外接的相机
	if(!capture.isOpened()){
		ROS_WARN("fail to open camera!");
	}



    struct timeval tStart,tEnd;    //变量保存程序开始时间，和结束时间
    float timeElapse;              //变量保存程序耗费时间

	while (!quit)
	{
//        gettimeofday(&tStart,NULL);  //记录程序开始时间
        /**********************************************************************/
		capture.read(originFrame);

		cvImg.header.stamp = ros::Time::now();
		cvImg.encoding = "bgr8";
		cvImg.image = originFrame;

		sensor_msgs::Image imgMsg;
		cvImg.toImageMsg(imgMsg);
		imgPub.publish(imgMsg);

	
		resize(originFrame, mFrame, Size(RESIZE_WIDTH, RESIZE_HEIGHT), CV_INTER_LINEAR);


         /***********************************************************************/

		// //  Lane-detected func
        //cvDetectLane(mFrame);
		ctanSlop++;
		std_msgs::UInt8 slopMsg;
		slopMsg.data = ctanSlop;
		ctanSlopPub.publish(slopMsg);

		int key = cvWaitKey(10);
		if(key == 27) {
		    quit = true;
		}

//        gettimeofday(&tEnd,NULL);    // 记录程序结束时间
//        timeElapse = tEnd.tv_usec-tStart.tv_usec + SECOND_TO_MRCROSECOND*(tEnd.tv_sec-tStart.tv_sec);    // 以微秒来计数
//        printf("Function Elapse Time:%f s\n",timeElapse/ SECOND_TO_MRCROSECOND);

	}

	
}
