#ifndef LASERSCANNER_H
#define LASERSCANNER_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>

struct LaserScan{
    LaserScan(float* p, int np){
        pts = p;
        ntps = np;
    }
    float* pts;
    int ntps;
    /*~LaserScan(){  // deleted null ptr, i guess it handle the deletion of pts in the delete of the LaserScan
        delete pts;
    }*/
};

class LaserScanner
{
public:
    LaserScanner();
    void parseScan(ConstLaserScanStampedPtr &_msg);

    cv::Mat visualizeScan(float px_per_m); //could be changed to int size for desired dim of img

private:

    bool   param_set = false;
    float _range_min;
    float _range_max;
    float _angle_min;
    float _angle_increment;
    int   _nranges;

    LaserScan* _current_scan;

};

#endif // LASERSCANNER_H
