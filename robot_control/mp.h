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
  int findCorners();
  void drawCorners();
  void findAreas();
  void findConnections();
  bool isBlackBetween(Mat, Point, Point, int);
  void displayMap();
  void drawRect();
  static bool largestArea(Rect const &, Rect const &);

  Mat bitmap;
  Mat bitmap_t;
  Mat display;
  Mat cornerkernel;
  Mat cornerMask;
  Mat areaMask;
  vector<corner> cnr;
  vector<corner> cnr_t;
  vector<Rect> area;
  vector<Rect> area_t;
};

#endif // MAP_H
