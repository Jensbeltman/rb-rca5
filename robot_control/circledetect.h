#ifndef CIRCLEDETECT_H
#define CIRCLEDETECT_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>
#include "buffer.h"

class circleDetect
{
public:
    circleDetect();

    void detect(ConstImageStampedPtr &msg);
    int search(cv::Mat &);
    int getAmountBlue(cv::Mat img);

private:
    buffer circBuffer;
    //std::vector<cv::Point> circleCenters;
};

#endif // CIRCLEDETECT_H
