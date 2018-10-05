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

void statCallback(ConstWorldStatisticsPtr &_msg) {
  (void)_msg;
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();
  //  std::cout << std::flush;
}

void poseCallback(ConstPosesStampedPtr &_msg) {
  // Dump the message contents to stdout.
  //std::cout << _msg->DebugString();

  for (int i = 0; i < _msg->pose_size(); i++) {
    if (_msg->pose(i).name() == "pioneer2dx") {
      gpos.x = 6 * _msg->pose(i).position().x();
      gpos.y = - 6 * _msg->pose(i).position().y();
      gdir = 3.14 + 2 * atan2(_msg->pose(i).orientation().w(),_msg->pose(i).orientation().z());
      break;
    }
  }
}

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

void lidarCallback(ConstLaserScanStampedPtr &msg) {

  float angle_min = float(msg->scan().angle_min());
  //  double angle_max = msg->scan().angle_max();
  float angle_increment = float(msg->scan().angle_step());

  float range_min = float(msg->scan().range_min());
  float range_max = float(msg->scan().range_max());

  int sec = msg->time().sec();
  int nsec = msg->time().nsec();

  int nranges = msg->scan().ranges_size();
  int nintensities = msg->scan().intensities_size();

  assert(nranges == nintensities);

  int width = nranges - 1;
  int height = range_max * 6;

  cv::Mat im(height, width, CV_8UC1);
  im.setTo(0);
  for (int i = 0; i < nranges; i++) {
    float range = std::min(float(msg->scan().ranges(i)), range_max);
    cv::Point2f startpt(i,0);
    cv::Point2f endpt(i,range*6);
    cv::line(im, startpt, endpt, cv::Scalar(255), 1,
             cv::LINE_4, 0);
  }
  mutex.lock();
  cv::imshow("lidar", im);
  mutex.unlock();
}

int main(int _argc, char **_argv) {
  // Load gazebo
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

  gazebo::transport::SubscriberPtr lidarSubscriber1 =
      node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", lidarCallback);

  LaserScanner ls;

  gazebo::transport::SubscriberPtr lidarSubscriber2 =
        node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", &LaserScanner::parseScan, &ls);

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

  Mat immap = imread("../models/bigworld/meshes/floor_plan.png", IMREAD_GRAYSCALE );
  Map map(&immap, 4);

  const int key_left = 81;
  const int key_up = 82;
  const int key_down = 84;
  const int key_right = 83;
  const int key_esc = 27;

  float speed = 0.0;
  float dir = 0.0;

  // Loop
  while (true) {
    gazebo::common::Time::MSleep(10);

    map.updatePose(gpos,gdir);

    mutex.lock();
    int key = cv::waitKey(1);
    map.show();
    map.showLidar();
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
