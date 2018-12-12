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
#include <fstream>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

struct Marble {
  void waitForData();
  string name;
  float pose[3];
  float robotAngle;
  float robotDistance;
};

class MarbleUtility {
 public:
  MarbleUtility();
  ~MarbleUtility();

  void poseCallback(ConstPosesStampedPtr &);
  void cameraCallback(ConstImageStampedPtr &);

  void getDistrParam();
  void distributeMarbles(gazebo::transport::PublisherPtr);
  void distributeMarble(gazebo::transport::PublisherPtr, int mI, float dist,
						float ang);
  void findMarbles();
  void myFloodFill(Mat *img, int *b, int c, int r);

  void matchMarbles();
  float distance(Marble, Marble);
  float distance(Marble *, Marble *);
  static bool marbleDist(const Marble &, const Marble &);
  static bool marbleAng(const Marble &, const Marble &);
  static bool pairDist(const pair<Marble *, Marble *> &,
					   const pair<Marble *, Marble *> &);
  float safeAcos(float x);

  ofstream marbleData;
  string dataName = "marbleData";
  bool newData = false;
  bool first = true;
  float rx = -49, ry = -49, rz = 0, ro = M_PI * 0.25;
  Mat src, src_hls, display;
  vector<Mat> src_hls_channels;
  vector<Marble> marble;
  vector<Marble> detectedMarble;
  vector<pair<Marble *, Marble *>> pairs;
  float meanSquareError;

  int numberOfMarbles, numberOfTests, angRange;
  int distL, distU;
};

#endif  // MARBLEUTILITY_H
