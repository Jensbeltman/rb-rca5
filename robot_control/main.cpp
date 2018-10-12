#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <mp.h>

#include <iostream>
using namespace std;
// using namespace cv;

int main(int _argc, char **_argv) {

  cv::Mat kort =
	  imread("../models/bigworld/meshes/floor_plan.png", IMREAD_COLOR);

  mp krt(kort);

  krt.drawRect();
  while (1) {
    krt.displayMap();

	waitKey(0);
  }
}
