#ifndef MP_H
#define MP_H

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <string>
#include <vector>

#include <area.h>

// x,y cordinates and type as in open(i)/closed(o) (belov/above 180 deg)
struct corner {
  Point P;
  Point D;
  char type;
};
using namespace std;
using namespace cv;

class mp {
public:
  mp();
  mp(Mat);
  void findCorners();
  void drawCorners();
  void findAreas();
  bool isBlackBetween(Point, Point, int);
  void displayMap();
  void drawRect();

  Mat bitmap;
  Mat display;
  Mat cornerkernel;
  Mat cornerMask;
  Mat areaMask;
  vector<corner> cnr;
  vector<Rect> area;
};

#endif // MAP_H
