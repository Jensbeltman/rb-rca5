#ifndef AREA_H
#define AREA_H

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

using namespace std;
using namespace cv;

struct cordinate {
  float x;
  float y;
};
class area {
public:
  area();
  area(Point, Point);
  bool contains(cordinate);
  bool overlaps(area);

  cordinate ul;
  cordinate lr;
};

#endif // AREA_H
