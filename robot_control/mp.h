#ifndef MP_H
#define MP_H
#include <opencv2/opencv.hpp>

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <line.h>
#include <localizor.h>
#include <sys/stat.h>
#include <unistd.h>
#include <algorithm>
#include <array>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <typeinfo>
#include <vector>
#include "Visibility/visibility.hpp"

using namespace std;
using namespace cv;
using namespace geometry;

using vector_type = geometry::vec2;
using segment_type = geometry::line_segment<vector_type>;
using segment_comparer_type = geometry::line_segment_dist_comparer<vector_type>;
using angle_comparer_type = geometry::angle_comparer<vector_type>;

// x,y cordinates and type as in open(i)/closed(o) (belov/above 180 deg)
struct corner {
  Point P;
  Point D;
  char type;
  float angle;
};

class MP {
 public:
  MP(Mat, string Name);

  string name;

  void genVisMap(Mat bitmap, Mat dst, int scale);
  void cnrHeatC();
  // void visionMap();//too heavy

  void localPoseCallback(ConstPosesStampedPtr &_msg);
  void poseCallback(ConstPosesStampedPtr &_msg);
  void findAreas(Mat bitmap, vector<Rect> &area);
  bool isBlackBetween(Mat image, Point a, Point d, int l);
  void drawRect(Mat m);
  static bool largestArea(Rect const &a, Rect const &b);
  static bool mostVision(pair<Point, vector<Point>> const &a,
						 pair<Point, vector<Point>> const &b);
  bool doesMapExist(const std::string &filename);
  void imwrite2(string s, Mat m);
  bool imread2(string s, Mat &m);
  float sumC1(Mat m);

  Localizor localizor;
  Mat bitmap;
  Mat cnrheatmap;
  Mat visionMap;
  Rect bitmapRect;

 private:
  corner nextCorner(corner cc, vector<corner> cnrs, bool x);
  void removeCnr(corner c, vector<corner> &cnrs);
  Point cnr2pos(corner c);
  void genVisPoly();
  void localMaxima(const Mat image, vector<Point> &maximas, bool);
  void drawPoly2(Mat image, vector<Point> &polygon, Scalar color);

  vector<corner *> **visCnr;

  int findCorners(Mat m, vector<corner> &);
  void drawCorners();

  vector<corner> cnr;
  vector<corner> cnrV;
  vector<Rect> area;
  vector<Point> mxVisPts;
  vector<pair<Point, vector<Point>>> mxVisPoly;

  int vMapScale;

  Mat display;
  Mat display2;
  Mat cornerkernel;
  // Gazebo setup
  gazebo::transport::NodePtr node;
  gazebo::transport::SubscriberPtr poseSubscriber;
  gazebo::transport::SubscriberPtr localPoseSubscriber;
  gazebo::transport::SubscriberPtr lidarSubscriber;

  //	vector<Mat> mask;

  //	vector<corner> cnr;
};

#endif  // MP_H
