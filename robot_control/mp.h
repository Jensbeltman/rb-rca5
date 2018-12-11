#ifndef MP_H
#define MP_H
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <localizor.h>
#include "mclocalizor.h"
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
using namespace std;
using namespace cv;

// x,y cordinates and type as in open(i)/closed(o) (belov/above 180 deg)
struct corner {
  Point P;
  Point D;
  char type;
  int seenBy = 0;
  float score = 0;
};

class MP {
 public:
  MP(Mat);
  void cnrHeat();
  void cnrHeatC();
  void drawMap();
  void localPoseCallback(ConstPosesStampedPtr &_msg);
  void globalPoseCallback(ConstPosesStampedPtr &_msg);
  void lidarScanCallback(ConstLaserScanStampedPtr &msg);

  Localizor localizor;
  MCLocalizor mclocalizor;

 private:
  Mat bitmap;
  Mat cnrheatmap;

  int findCorners(Mat m);
  void drawCorners();

  vector<corner> cnr;

  Mat display;

  Mat cornerkernel;

  //	vector<Mat> mask;

  //	vector<corner> cnr;
};

#endif  // MP_H
