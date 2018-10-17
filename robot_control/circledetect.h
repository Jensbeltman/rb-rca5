#ifndef CIRCLEDETECT_H
#define CIRCLEDETECT_H

#include <opencv2/opencv.hpp>
#include "buffer.h"

class circleDetect
{
public:
    circleDetect();

    int search(cv::Mat &);
    int getAmountBlue(cv::Mat img);

private:
    buffer circBuffer;
    //std::vector<cv::Point> circleCenters;
};

#endif // CIRCLEDETECT_H
