#ifndef MCLOCALIZER_H
#define MCLOCALIZER_H

#include <opencv2/opencv.hpp>
#include "laserscanner.h"

using namespace cv;

class MCLocalizer
{
public:
    MCLocalizer();

    // Gazebo subscribers
    void localPoseCallback(ConstPosesStampedPtr &_msg);
    void lidarScanCallback(ConstLaserScanStampedPtr &msg);

    void globalPoseCallback(ConstPosesStampedPtr &_msg); // for debug only


private:

    Point2f upos;
    float   uphi;

    LaserScanner laserScanner;

    std::default_random_engine generator;
    std::normal_distribution<float> distribution = std::normal_distribution<float>(1.0, 0.1);

    float t = -1;
    float pl = 0;
    float pr = 0;

    void localize(LaserScan *ls);





};

#endif // MCLOCALIZER_H
