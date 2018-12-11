#ifndef LOCALIZOR_H
#define LOCALIZOR_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <mutex>
#include <opencv2/opencv.hpp>

#include <iostream>

#define G2P ((72.0 / 25.4) * 0.5)
#define P2G ((25.4 / 72.0) * 2.0)
using namespace std;
using namespace cv;

class Localizor {
 public:
  Localizor(int = 0, int = 0, float = 0);
  void localPoseCallback(ConstPosesStampedPtr &_msg);
  void globalPoseCallback(ConstPosesStampedPtr &_msg);
  void printPose();
  double getX();
  double getY();

 private:
  float s = ((72.0 / 25.4) * 2);

  Point2f upos;
  float   uphi;

  Point2f apos; // * actual pos
  float   aphi; // * and dir

  Point2f center;

  float t = -1;
  float pl = 0;
  float pr = 0;

};

#endif  // LOCALIZOR_H
