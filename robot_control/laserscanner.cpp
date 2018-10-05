#include "laserscanner.h"

LaserScanner::LaserScanner()
{

}

void LaserScanner::parseScan(ConstLaserScanStampedPtr &msg)
{
    if(!param_set){
        _angle_min = float(msg->scan().angle_min());
        _angle_increment = float(msg->scan().angle_step());

        _range_min = float(msg->scan().range_min());
        _range_max = float(msg->scan().range_max());

        _nranges = msg->scan().ranges_size();
        param_set = true;
    }

    float* rays = new float[_nranges];

    for (int i = 0; i < _nranges; i++)
      rays[i] = std::min(float(msg->scan().ranges(i)), _range_max);

    if(_current_scan != nullptr){
        delete _current_scan;
    }
    _current_scan = new LaserScan(rays, _nranges);
}

cv::Mat LaserScanner::visualizeScan(float px_per_m)
{
    // TODO: remake visualization of scanner
    return cv::Mat();
}

