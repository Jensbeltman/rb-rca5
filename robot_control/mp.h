#ifndef MP_H
#define MP_H
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <line.h>
#include <localizor.h>
#include <algorithm>
#include <array>
#include <iomanip>
#include <iostream>
#include <string>
#include <typeinfo>
#include <vector>
using namespace std;
using namespace cv;

// x,y cordinates and type as in open(i)/closed(o) (belov/above 180 deg)
struct corner {
  Point P;
  Point D;
  char type;
  float angle;
};

class MP {
 public:
  MP(Mat);
  void findlines();
  void genVisCnr();
  void visionMap();
  void cnrHeat();
  void cnrHeatC();
  // void visionMap();//too heavy

  void drawMap();
  void localPoseCallback(ConstPosesStampedPtr &_msg);
  void poseCallback(ConstPosesStampedPtr &_msg);
  Localizor localizor;

 private:
  Mat bitmap;
  Mat cnrheatmap;
  Mat vMap;
  Rect bitmapRect;

  vector<corner *> **visCnr;

  int findCorners(Mat m, vector<corner> &);
  void drawCorners();

  vector<corner> cnr;
  vector<corner> cnrV;
  vector<Line> mapLines;

  Mat display;
  Mat cornerkernel;
  bool pixelInImage();
  // Gazebo setup
  gazebo::transport::NodePtr node;
  gazebo::transport::SubscriberPtr poseSubscriber;
  gazebo::transport::SubscriberPtr localPoseSubscriber;
  gazebo::transport::SubscriberPtr lidarSubscriber;

  //	vector<Mat> mask;

  //	vector<corner> cnr;
};

#endif  // MP_H
