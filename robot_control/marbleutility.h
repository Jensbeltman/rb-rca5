#ifndef MARBLEUTILITY_H
#define MARBLEUTILITY_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>

#include <math.h>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

struct Marble {
  string name;
  float pose[3];
  float robotAngle;
  float robotDistance;
};

class MarbleUtility {
 public:
  MarbleUtility();
  void getDistrParam();
  void distributeMarbles(gazebo::transport::PublisherPtr);
  void findMarbles();

  static bool marbleDist(const Marble &, const Marble &);
  static bool marbleAng(const Marble &, const Marble &);

  bool newData = false;

  float rx, ry, rz, ro;

  Mat src;
  vector<Marble> marble;
  vector<Marble> detectedMarble;

  int numberOfMarbles, numberOfTests, angRange;
  int distL, distU;
};

#endif  // MARBLEUTILITY_H
