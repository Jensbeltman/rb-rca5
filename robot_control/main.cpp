#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>

using namespace cv;

//#include "fuzzy1.h"
//#include "fuzzy2.h"
#include "fuzzy_control.h"
#include "ransacscanner.h"
#include "movetopoint.h"
#include "mclocalizor.h"

static boost::mutex mutex;


int main(int _argc, char **_argv) {

    cv::Mat map =
    imread("../models/bigworld/meshes/floor_plan.png", IMREAD_GRAYSCALE);

    MoveToPoint testGoal;
    laserscanner scanner;
    MCLocalizor mc(map, 4);

    gazebo::common::Time::MSleep(500);

    //std::vector<std::pair<int, int>> goals = {std::make_pair(80, 38), std::make_pair(90, 35), std::make_pair(96, 14), std::make_pair(104, 12), std::make_pair(111, 31), std::make_pair(103, 38)};
    std::vector<std::pair<int, int>> goals = {std::make_pair(90, 30), std::make_pair(96, 14), std::make_pair(104, 12), std::make_pair(111, 31), std::make_pair(103, 38)};


  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();


  //gazebo::transport::SubscriberPtr toPoint =
  //        node->Subscribe("~/pose/info", &MoveToPoint::displayGoal, &testGoal);

  //gazebo::transport::SubscriberPtr setPos =
  //        node->Subscribe("~/pose/info", &MoveToPoint::setPosition, &testGoal);

  gazebo::transport::SubscriberPtr laserScanRansac =
          node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", &laserscanner::findLines, &scanner);

  gazebo::transport::SubscriberPtr localPoseSubscriber =
          node->Subscribe("~/pose/local/info", &MCLocalizor::localPoseCallback, &mc);

  gazebo::transport::SubscriberPtr globalPoseSubscriber =
          node->Subscribe("~/pose/info", &MCLocalizor::globalPoseCallback, &mc);

  gazebo::transport::SubscriberPtr lidarSubscriber =
          node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", &MCLocalizor::lidarScanCallback, &mc);



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



  obstacle_avoidance controller1;
  controller1.init_controller();
  fuzzy_goal controller2;
  controller2.init_controller();
  take_marble controller3;
  controller2.init_controller();

  int goalIndex = 0;

  testGoal.setGoal(goals[goalIndex].first, goals[goalIndex].second);
  goalIndex++;

    bool obstacleOn = false;

  // Loop
  while (true) {
    gazebo::common::Time::MSleep(10);

    mutex.lock();
    int key = cv::waitKey(1);
    mutex.unlock();

    if (key == key_esc)
      break;

    mc.show();

    conf configuration = mc.getConfiguration();

    //std::cout << configuration.x/4 << " : " << configuration.y/4 << " : " << configuration.dir << std::endl;
    distAndAngle goal = testGoal.leftToGoal(configuration.x/4, configuration.y/4, configuration.dir);
    closestLine scan = scanner.getClosestLine();

    //std::cout << goal.distance << " : " << goal.angle << std::endl;

    if (goal.distance < 1.2) {
        testGoal.setGoal(goals[goalIndex].first, goals[goalIndex].second);
        if (++goalIndex == (int)goals.size()) {
          goalIndex = 0;
        }
    }

    if (scan.distance2 < 2) {
        obstacleOn = true;
    }
    if (scan.distance2 > 3) {
        obstacleOn = false;
    }

    if (obstacleOn) {
        //std::cout << "O" << std::endl;
        controller1.setValues(scan.distance, scan.angle);
        controller1.process();
        speed = controller1.getValues().speed;
        dir = controller1.getValues().direction;
    } else {
        //std::cout << "G" << std::endl;
        controller2.setValues(goal.distance, -goal.angle);
        controller2.process();
        speed = controller2.getValues().speed;
        dir = controller2.getValues().direction;
    }



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
    movementPublisher->Publish(msg);
  }

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
