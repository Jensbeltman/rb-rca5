#include "movetopoint.h"

#include <opencv2/opencv.hpp>
#include <math.h>
#include <iostream>

MoveToPoint::MoveToPoint()
{

}

void MoveToPoint::setGoal(float x, float y)
{
    goal = std::make_pair(x, y);
}

distAndAngle MoveToPoint::leftToGoal(float positionX, float positionY)
{
    distAndAngle output;

    output.distance = sqrt(pow(goal.first-positionX, 2)+pow(goal.second-positionY, 2));
    output.angle = atan2(goal.second-positionY, goal.first-positionX);

    return output;
}

distAndAngle MoveToPoint::leftToGoal()
{
    distAndAngle output;

    output.distance = sqrt(pow(goal.first-position.first, 2)+pow(goal.second-position.second, 2));
    output.angle = atan2(goal.second-position.second, goal.first-position.first) - orientation;

    //std::cout << "Distance: " << output.distance << " angle: " << output.angle << std::endl;

    return output;
}

void MoveToPoint::setPosition(ConstPosesStampedPtr &msg)
{
    for (int i = 0; i < msg->pose_size(); i++) {
        if (msg->pose(i).name() == "pioneer2dx") {
            position.first = msg->pose(i).position().x()*72/25.4*0.5 + 60;
            position.second = -1*msg->pose(i).position().y()*72/25.4*0.5 + 40;
            orientation = -M_PI+2*atan2(msg->pose(i).orientation().w(), msg->pose(i).orientation().z());
        }
    }
    //std::cout << "x: " << position.first << " y: " << position.second << std::endl;
}

void MoveToPoint::displayGoal(ConstPosesStampedPtr &msg)
{
    cv::Mat _map = cv::imread("../models/bigworld/meshes/floor_plan.png", cv::IMREAD_COLOR);
    int init_x = _map.cols/2;
    int init_y = _map.rows/2;
    float x = 0;
    float y = 0;
    //cv::Mat doubleMap(_map.rows*2, _map.cols*2, CV_8UC3, cv::Scalar(0, 0, 0));
    //resize(_map, doubleMap, doubleMap.size(), 0, 0, cv::INTER_NEAREST);

    for (int i = 0; i < msg->pose_size(); i++) {
        if (msg->pose(i).name() == "pioneer2dx") {
            x = msg->pose(i).position().x()*72/25.4*0.5;
            y = -1*msg->pose(i).position().y()*72/25.4*0.5;
        }
    }
    _map.at<cv::Vec3b>(init_y + y , init_x + x) = {0, 0, 150};

    _map.at<cv::Vec3b>(goal.second, goal.first) = {0, 0, 200};
    cv::line(_map, cv::Point(init_x + x, init_y + y), cv::Point(goal.first, goal.second), cv::Scalar(200, 0, 0), 1, cv::LINE_8);

    //cv::namedWindow("Map", cv::WINDOW_FREERATIO);
    //cv::imshow("Map", _map);
}
