#ifndef MONTECARLO_H
#define MONTECARLO_H

#include <opencv2/opencv.hpp>
#include "laserscanner.h"

using namespace cv;

struct Robconf {
  Point2f pos;
  float dir;
  float score;
};

class Montecarlo {
 public:
  Montecarlo();
  Montecarlo(Mat m, uchar s = 4);

  void parseScan(ConstLaserScanStampedPtr &msg);
  void reDistribute(Point2f, float);
  void setConf(Point2f, float, bool = false);
  Point2f getBestPos();
  double getBestDir();

  void show();
  void showLidar();

  Mat *getMap();

 private:
  Mat map;
  int scale;

  Point2f pos = Point2f(0, 0);
  double dir = 0;

  Point2f tpos = Point2f(0, 0);
  double tdir = 0;

  LaserScanner laserScanner;

  void localize(LaserScan *ls);
  float ray(Point2f p, float r, float angle);

  std::default_random_engine generator;
  std::normal_distribution<float> distribution =
	  std::normal_distribution<float>(1.0, 4);

  int nEst = 40;
  Robconf rConf[40];
  float calcScore(Robconf rc, LaserScan *ls);
};

#endif  // MAP_H
