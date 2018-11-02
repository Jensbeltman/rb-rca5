#ifndef LOCALIZOR_H
#define LOCALIZOR_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <mutex>
#include <opencv2/opencv.hpp>

#include <iostream>

#include "montecarlo.h"

#define G2P ((70.0 / 25.4) * 0.5)
#define P2G ((25.4 / 70.0) * 2.0)
using namespace std;
using namespace cv;

class Localizor {
 public:
  Localizor();
  Localizor(Mat, Mat);
  void localPoseCallback(ConstPosesStampedPtr &_msg);
  void poseCallback(ConstPosesStampedPtr &_msg);
  void printPose();
  double getX();
  double getY();
  bool cnrInrange(int threshold);
  // Montecarlo
  Montecarlo montecarlo;

 private:
  // maps
  Mat bitmap, cnrheatmap, montecarloMap;
  // position calculations
  double prev_p_l, prev_p_r, prev_t;
  double dp_l, dp_r, dt, t;
  double x, y, phi, rx, ry, rphi, mx, my;
  double rw, lw;
  double qw, qx, qy, qz, p_l, p_r, v_l, v_r;
  bool first;
};

#endif  // LOCALIZOR_H
