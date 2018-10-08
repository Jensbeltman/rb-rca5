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
        //delete _current_scan;
    }
    _current_scan = new LaserScan(rays, _nranges);
}

LaserScan *LaserScanner::getScan()
{
    return _current_scan;
}

LaserScan *LaserScanner::generateScan(Mat* map, Point2f pos, float dir)
{
    float scale = (72. / 25.4) * 2;
    Point2f p = pos + 0.8 * Point2f(cos(dir), sin(dir));

    float* rays = new float[_nranges];

    for(int i = 0; i < _nranges; i++){
        rays[i] = ray(map, p, dir + _angle_min + _angle_increment * i);
    }
    return new LaserScan(rays, _nranges);
}


float LaserScanner::ray(Mat* map, Point2f p, float angle){
    Point2f delta(cos(angle),sin(angle));
    for(float r = 0; r < _range_max; r+=.5){
        if(map->at<uchar>(p+r*delta) == 0)
            return r;
    }
    return _range_max;
}

cv::Mat LaserScanner::visualizeScan(float px_per_m)
{
    // TODO: remake visualization of scanner
    return cv::Mat();
}

