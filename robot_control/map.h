#ifndef MAP_H
#define MAP_H

#include <opencv2/opencv.hpp>

using namespace cv;

class Map {
public:
  Map(Mat *img, uchar scaling);
  void show();
  void updatePose(Point p, double d);
  void showLidar();

private:
  Mat map;
  Point pos = Point(0, 0);
  double dir = 0;
  Mat lidarMask;

  Mat copySafe(Point p, double d);
};

#endif // MAP_H
