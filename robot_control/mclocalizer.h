﻿#ifndef MCLOCALIZER_H
#define MCLOCALIZER_H

#include <mutex>
#include <array>

#include <opencv2/opencv.hpp>
#include "laserscanner.h"

#define N_CONF 8

using namespace cv;

struct conf {
  float x;
  float y;
  float dir;
  float weight;
};


class MCLocalizer
{
public:
    MCLocalizer(){}
    MCLocalizer(Mat m, uchar s);
    ~MCLocalizer();

    // Gazebo subscribers
    void localPoseCallback(ConstPosesStampedPtr &_msg);
    void lidarScanCallback(ConstLaserScanStampedPtr &msg);

    void globalPoseCallback(ConstPosesStampedPtr &_msg); // for debug only

    void show();


private:
    float s = ((72.0 / 25.4) * 2);

    std::mutex * _ptex = new std::mutex();
    std::mutex * _btex = new std::mutex();
    int scale;

    Mat map;

    Point2f upos;
    float   uphi;

    Point2f apos; // * actual pos
    float   aphi; // * and dir

    LaserScanner laserScanner;

    std::default_random_engine gen;
    std::normal_distribution<float> ndist = std::normal_distribution<float>(1.0, 0.2);
    std::normal_distribution<float>   ndist_conf = std::normal_distribution<float> (0,N_CONF/3);

    float t = -1;
    float pl = 0;
    float pr = 0;

    conf  bel[N_CONF];
    std::array<conf, N_CONF> tbel;

    void tempBelief();
    void calcWeights(LaserScan *ls);
    void localize(LaserScan *ls);





};

#endif // MCLOCALIZER_H
