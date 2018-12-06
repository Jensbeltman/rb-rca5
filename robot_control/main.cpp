
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

static boost::mutex mutexx;

MarbleUtility mUtil;

void statCallback(ConstWorldStatisticsPtr &_msg) { (void)_msg; }

void poseCallback(ConstPosesStampedPtr &_msg) {
  for (int i = 0; i < _msg->pose_size(); i++) {
	if (_msg->pose(i).name() == "pioneer2dx") {
	  mUtil.rx = (float)_msg->pose(i).position().x();
	  mUtil.ry = (float)_msg->pose(i).position().y();
	  mUtil.rz = (float)_msg->pose(i).position().z();
	  mUtil.ro = M_PI * 0.25;
	}
  }
}

void cameraCallback(ConstImageStampedPtr &msg) {
  std::size_t width = msg->image().width();
  std::size_t height = msg->image().height();
  const char *data = msg->image().data().c_str();
  cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));

  mUtil.src = im.clone();
  mUtil.newData = true;
}

int main(int _argc, char **_argv) {
  mUtil.getDistrParam();

  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo topics
  gazebo::transport::SubscriberPtr statSubscriber =
	  node->Subscribe("~/world_stats", statCallback);

  gazebo::transport::SubscriberPtr poseSubscriber =
	  node->Subscribe("~/pose/info", poseCallback);

  gazebo::transport::SubscriberPtr cameraSubscriber =
	  node->Subscribe("~/pioneer2dx/camera/link/camera/image", cameraCallback);

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

  while (true) {
	gazebo::common::Time::MSleep(10);

	mutexx.lock();
	int key = waitKey(2);
	mutexx.unlock();
	if (key == key_esc) break;

	if (test < mUtil.numberOfTests && key == key_right) {
	  mUtil.distributeMarbles(movePublisher);
	  while (!mUtil.newData)
		;
	  mUtil.findMarbles();
	  mUtil.newData = false;

	  test++;

	  cout << endl;
	}
  }

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
