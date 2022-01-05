#ifndef __SINGLE_EYE_MAIN__
#define __SINGLE_EYE_MAIN__

#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <string.h>
#include <vector>


class single_eye_body
{
private:
    volatile unsigned char ctanSlop;    //add by andy 20200415

    //基于单目摄像头进行视觉检测车道线线程
    void laneDectionThreadHandler();

    long getMatches(const cv::Mat& Car1, const cv::Mat& Car2);
    double getPSNR(const cv::Mat& I1, const cv::Mat& I2);
    cv::Point GetWrappedPoint(cv::Mat M, const cv::Point& p);
    void draw_locations(cv::Mat & img, std::vector< cv::Rect > & locations, const cv::Scalar & color,std::string text);

    void cvDetectLane(cv::Mat &mFrame);

public:
    single_eye_body(ros::NodeHandle &n);
    ~single_eye_body();
};



#endif