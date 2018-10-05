#ifndef MAP_H
#define MAP_H

#include <opencv2/opencv.hpp>

using namespace cv;

class Map {
public:
  Map(Mat *img, uchar scaling);
  void show();
  void updatePose(Point2f p, double d);
  void showLidar();


private:
  Mat map;
  Point2f pos = Point2f(0, 0);
  double dir = 0;
  Mat lidarMask;

  Mat copySafe(Point p, double d);
  float ray(Point2f p, float r, float angle);
};

#endif // MAP_H
