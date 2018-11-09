#ifndef LINE_H
#define LINE_H
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <localizor.h>
#include <array>
#include <iomanip>
#include <iostream>
#include <string>
#include <typeinfo>
#include <vector>

using namespace std;
using namespace cv;

class Line {
 public:
  Line();
  Line(Point2f, Point2f);
  bool nearestIntersection(vector<Line>);
  void setLenth(float);
  float getAngle();
  Point2f p1, p2;
};

#endif  // LINE_H
