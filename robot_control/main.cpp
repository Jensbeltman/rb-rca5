
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>

#include <marbleutility.h>
#include <math.h>
#include <cstdlib>
#include <ctime>
#include <iostream>
using namespace cv;
using namespace std;

MarbleUtility mUtil;

void statCallback(ConstWorldStatisticsPtr &_msg) { (void)_msg; }

int main(int _argc, char **_argv) {
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo topics
  gazebo::transport::SubscriberPtr statSubscriber =
	  node->Subscribe("~/world_stats", statCallback);

  gazebo::transport::SubscriberPtr cameraSubscriber =
	  node->Subscribe("~/pioneer2dx/camera/link/camera/image",
					  &MarbleUtility::cameraCallback, &mUtil);

  gazebo::transport::PublisherPtr movePublisher =
	  node->Advertise<gazebo::msgs::Pose>("~/pose/modify");

  // Publish a reset of the world
  gazebo::transport::PublisherPtr worldPublisher =
	  node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
  gazebo::msgs::WorldControl controlMessage;
  controlMessage.mutable_reset()->set_all(true);
  worldPublisher->WaitForConnection();
  worldPublisher->Publish(controlMessage);

  int test = 0;

  const int key_left = 81;
  const int key_up = 82;
  const int key_down = 84;
  const int key_right = 83;
  const int key_esc = 27;

  gazebo::common::Time::MSleep(10);

  int key = waitKey(1);

  float dL = 3;
  float dU = 50;
  float aL = 20;
  float aU = 70;

  for (float d = dL; d <= dU; d += 0.5f) {
	for (float a = aL; a <= aU; a += 2.5f) {
	  cout << d << "," << a << endl;
	  mUtil.distributeMarble(movePublisher, 0, (float)d, a * 0.0174532925);

	  mUtil.newData = false;
	  // gazebo::common::Time::MSleep(100);
	  while (mUtil.newData == false) {
		;
	  }
	  mUtil.findMarbles();
	}
  }

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
