#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>

#include <iostream>

#include <map.h>
#include <laserscanner.h>

using namespace cv;

static boost::mutex mutex;

static Point2f gpos(0,0);
static double gdir = 0;


void cameraCallback(ConstImageStampedPtr &msg) {

  std::size_t width = msg->image().width();
  std::size_t height = msg->image().height();
  const char *data = msg->image().data().c_str();
  cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));

  im = im.clone();
  cv::cvtColor(im, im, COLOR_BGR2RGB);

  mutex.lock();
  cv::imshow("camera", im);
  mutex.unlock();
}

int main(int _argc, char **_argv) {

  Mat immap = imread("../models/bigworld/meshes/floor_plan.png", IMREAD_GRAYSCALE );
  Map map(&immap, 4);

  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  gazebo::transport::SubscriberPtr cameraSubscriber =
      node->Subscribe("~/pioneer2dx/camera/link/camera/image", cameraCallback);

  gazebo::transport::SubscriberPtr poseSubscriber =
      node->Subscribe("~/pose/info", &Map::updatePose, &map);

  gazebo::transport::SubscriberPtr lidarSubscriber2 =
      node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", &Map::parseScan, &map);

  // Publish to the robot vel_cmd topic
  gazebo::transport::PublisherPtr movementPublisher =
      node->Advertise<gazebo::msgs::Pose>("~/pioneer2dx/vel_cmd");

  gazebo::transport::PublisherPtr jointPublisher =
      node->Advertise<gazebo::msgs::JointCmd>("~/pioneer2dx/joint_cmd");

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
    gazebo::common::Time::MSleep(50);

    mutex.lock();
    int key = cv::waitKey(1);
    map.show();
    //map.showLidar();
    mutex.unlock();

    if (key == key_esc)
      break;

    if ((key == key_up) && (speed <= 1.2f))
      speed += 0.05;
    else if ((key == key_down) && (speed >= -1.2f))
      speed -= 0.05;
    else if ((key == key_right) && (dir <= 0.4f))
      dir += 0.05;
    else if ((key == key_left) && (dir >= -0.4f))
      dir -= 0.05;
    else {
      // slow down
      //      speed *= 0.1;
      //      dir *= 0.1;
    }

    // Generate a pose
    ignition::math::Pose3d pose(double(speed), 0, 0, 0, 0, double(dir));

    // Convert to a pose message
    gazebo::msgs::Pose msg;
    gazebo::msgs::Set(&msg, pose);

    //Left wheel control
    gazebo::msgs::JointCmd ljcmd;
    ljcmd.set_force(10);
    ljcmd.set_name("pioneer2dx::pioneer2dx::left_wheel_hinge");

    //Right wheel control
    gazebo::msgs::JointCmd rjcmd;
    ljcmd.set_force(10);
    ljcmd.set_name("pioneer2dx::pioneer2dx::right_wheel_hinge");

    //Publish messages
    movementPublisher->Publish(msg);
    //jointPublisher->Publish(rjcmd);
    //jointPublisher->Publish(ljcmd);
  }

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
