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
  // -------------VisionMap Main functions------------------------
  void genVisMap(Mat bitmap, Mat &vMap, vector<segment_type> &lines, int scale);
  void genVisPoints(Mat bitmap, Mat &vMap, vector<segment_type> &lines,
					vector<Point> &pts,
					vector<pair<Point, vector<Point>>> &visPoly,
					float coverage);
  void genVisPoly(vector<segment_type> &lines, vector<Point> &pts,
				  vector<pair<Point, vector<Point>>> &visPoly);
  // visionMap helper functions
  void removeCnr(corner c, vector<corner> &cnrs);
  static bool mostVision(pair<Point, vector<Point>> const &a,
						 pair<Point, vector<Point>> const &b);
  corner nextCorner(corner cc, vector<corner> cnrs, bool x);

  Point cnr2pos(corner c);

  //----------------- Corners ---------------------------------------
  int findCorners(Mat m, vector<corner> &);
  void drawCorners();
  void cnrHeatC();

  // ------------------Rectangles -----------------------------
  void findAreas(Mat bitmap, vector<Rect> &area);
  void drawRect(Mat m);
  static bool largestArea(Rect const &a, Rect const &b);

  // general helper functions
  void imwrite2(string s, Mat m);
  void localMaxima(const Mat image, vector<Point> &maximas, bool, int kx,
				   int ky);
  void drawPoly2(Mat image, vector<Point> &polygon, Scalar color);
  float sumC1(Mat m);
  bool imread2(string s, Mat &m);
  bool isBlackBetween(Mat image, Point a, Point d, int l);

  Localizor localizor;

  // Mats
  Mat bitmap;
  Mat cnrheatmap;
  Mat visionMap;
  Rect bitmapRect;
  Mat display;
  Mat display2;
  Mat cornerkernel;

  // gazebo
  void localPoseCallback(ConstPosesStampedPtr &_msg);
  void poseCallback(ConstPosesStampedPtr &_msg);

 private:
  int vMapScale;
  vector<corner> cnr;
  vector<Rect> area;
  vector<Point> mxVisPts;
  vector<segment_type> lines;
  vector<pair<Point, vector<Point>>> mxVisPoly;

  // Gazebo setup
  gazebo::transport::NodePtr node;
  gazebo::transport::SubscriberPtr poseSubscriber;
  gazebo::transport::SubscriberPtr localPoseSubscriber;
  gazebo::transport::SubscriberPtr lidarSubscriber;
};

#endif  // MP_H
