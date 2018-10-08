#ifndef MAP_H
#define MAP_H

#include <opencv2/opencv.hpp>
#include "laserscanner.h"

using namespace cv;

struct Robconf{
    Point2f pos;
    float dir;
    float score;
};

class Map {
public:
  Map(Mat *img, uchar scaling);

  void parseScan(ConstLaserScanStampedPtr &msg);
  void updatePose(ConstPosesStampedPtr &_msg);

  void show();
  void showLidar();

  Mat* getMap();

private:
  Mat map;

  Point2f pos = Point2f(0, 0);
  double dir = 0;

  Point2f tpos = Point2f(0, 0);
  double tdir = 0;

  LaserScanner laserScanner;

  void localize(LaserScan* ls);
  float ray(Point2f p, float r, float angle);

  std::default_random_engine generator;
  std::normal_distribution<float> distribution = std::normal_distribution<float>(1.0,0.01);


  int nEst = 40;
  Robconf rConf[40];
};

#endif // MAP_H
