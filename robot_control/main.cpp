#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>

#include <iostream>

#include <localizor.h>
#include <mp.h>
#include <mutex>
#include "line.h"
using namespace std;
using namespace cv;

static boost::mutex mutex2;
static std::mutex posMutex;

void cameraCallback(ConstImageStampedPtr &msg) {
  std::size_t width = msg->image().width();
  std::size_t height = msg->image().height();
  const char *data = msg->image().data().c_str();
  cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));

  im = im.clone();
  cv::cvtColor(im, im, COLOR_BGR2RGB);

  mutex2.lock();
  cv::imshow("camera", im);
  mutex2.unlock();
}

int main(int _argc, char **_argv) {
  cv::Mat mpp1 =
	  imread("../models/bigworld/meshes/floor_plan.png", IMREAD_COLOR);
  cv::Mat mpp2 =
	  imread("../models/smallworld/meshes/floor_plan.png", IMREAD_COLOR);
  cv::Mat mpp3 =
	  imread("../models/testworld/meshes/floor_plan.png", IMREAD_COLOR);

  // Load gazebo
  gazebo::client::setup(_argc, _argv);
  // namedWindow("camera", WINDOW_FREERATIO);

  // initialise a MP object for localization
  MP mp1(mpp1, "bigworld");
  waitKey();
  MP mp2(mpp2, "smallworld");
  waitKey();
  MP mp3(mpp3, "testworld");

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  //  gazebo::transport::SubscriberPtr cameraSubscriber =
  //	  node->Subscribe("~/pioneer2dx/camera/link/camera/image",
  // cameraCallback);

  // Publish to the robot vel_cmd topic
  gazebo::transport::PublisherPtr movementPublisher =
	  node->Advertise<gazebo::msgs::Pose>("~/pioneer2dx/vel_cmd");

  // Publish a reset of the world
  gazebo::transport::PublisherPtr worldPublisher =
	  node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
  gazebo::msgs::WorldControl controlMessage;
  controlMessage.mutable_reset()->set_all(true);
  worldPublisher->WaitForConnection();
  worldPublisher->Publish(controlMessage);

  const int key_left = 81;
  const int key_up = 82;
  const int key_down = 84;
  const int key_right = 83;
  const int key_esc = 27;

  float speed = 0.0;
  float dir = 0.0;

  // Loop
  while (true) {
	mp1.localizor.printPose();
	mp1.localizor.montecarlo.show();

	gazebo::common::Time::MSleep(10);

	mutex2.lock();
	int key = cv::waitKey(1);
	mutex2.unlock();

	if (key == key_esc) break;

	if ((key == key_up) && (speed <= 1.5f))
	  speed += 0.1;
	else if ((key == key_down) && (speed >= -1.5f))
	  speed -= 0.1;
	else if ((key == key_right) && (dir <= 0.2f))
	  dir += 0.5;
	else if ((key == key_left) && (dir >= -0.2f))
	  dir -= 0.5;
	else {
	  //	 //  slow down
	  //	        speed *= 0.1;
	  //	        dir *= 0.1;
	}

	// Generate a pose
	ignition::math::Pose3d pose(double(speed), 0, 0, 0, 0, double(dir));

	// Convert to a pose message
	gazebo::msgs::Pose msg;
	gazebo::msgs::Set(&msg, pose);
	movementPublisher->Publish(msg);
  }
  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
