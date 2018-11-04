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
#include "circledetect.h"
#include "laserscanner.h"

static boost::mutex mutex;


//bool marblePresent = false;
bool allBlue = false;
int marbleDist = 0;

void trackRobot(ConstPosesStampedPtr &msg) {
    Mat _map = imread("../models/bigworld/meshes/floor_plan.png", IMREAD_COLOR);
    int init_x = _map.cols;
    int init_y = _map.rows;
    float x = 0;
    float y = 0;
    Mat doubleMap(_map.rows*2, _map.cols*2, CV_8UC3, Scalar(0, 0, 0));
    resize(_map, doubleMap, doubleMap.size(), 0, 0, INTER_NEAREST);

    for (int i = 0; i < msg->pose_size(); i++) {
        if (msg->pose(i).name() == "pioneer2dx") {
            x = msg->pose(i).position().x()*72/25.4;//*0.5;
            y = -1*msg->pose(i).position().y()*72/25.4;//*0.5;
        }
    }
    doubleMap.at<Vec3b>(init_y + y , init_x + x) = {0, 0, 150};

    //cv::namedWindow("MapTracking", cv::WINDOW_FREERATIO);

    mutex.lock();
    //imshow("MapTracking", doubleMap);
    mutex.unlock();
}

void statCallback(ConstWorldStatisticsPtr &_msg) {
  (void)_msg;
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();
  //  std::cout << std::flush;
}

void poseCallback(ConstPosesStampedPtr &_msg) {
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();

  for (int i = 0; i < _msg->pose_size(); i++) {
    if (_msg->pose(i).name() == "pioneer2dx") {

      std::cout << std::setprecision(2) << std::fixed << std::setw(6)
                << _msg->pose(i).position().x() << std::setw(6)
                << _msg->pose(i).position().y() << std::setw(6)
                //<< _msg->pose(i).position().z() << std::setw(6)
                << _msg->pose(i).orientation().w() << std::setw(6)
                //<< _msg->pose(i).orientation().x() << std::setw(6)
                //<< _msg->pose(i).orientation().y() << std::setw(6)
                << _msg->pose(i).orientation().z() << std::endl;
    }
  }
}
circleDetect findCircles;
void cameraCallback(ConstImageStampedPtr &msg) {

  std::size_t width = msg->image().width();
  std::size_t height = msg->image().height();
  const char *data = msg->image().data().c_str();
  cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));

  //std::cout << "Width: " << width << " Height: " << height << std::endl;

  //im = im.clone();
  cv::cvtColor(im, im, COLOR_RGB2BGR);
    //cvtColor(im, im, COLOR_BGR2GRAY);

  // Marble detection
  //marble_detection detect;
  //detect.searchForMarbles(&im);
    /*
    GaussianBlur(im, im, Size(9, 9), 2, 2);
    std::vector<Vec3f> circles;
    HoughCircles(im, circles, HOUGH_GRADIENT, 1, im.rows/8, 50, 25, 0, 0);

    for (size_t i = 0; i < circles.size(); i++) {
        Point center(cvRound(circles[i][0]),cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        //Center
        circle(im, center, 3, Scalar(255), -1, 8, 0);
        //Outline
        circle(im, center, radius, Scalar(255), 3, 8, 0);
    }
*/

    if (findCircles.getAmountBlue(im) > 400)
        allBlue = true;
    else
        allBlue = false;
    marbleDist = findCircles.search(im);


  mutex.lock();
  cv::imshow("camera", im);
  mutex.unlock();
}

/*
//float smallest_dist;
//float smallest_dir;

void lidarCallback(ConstLaserScanStampedPtr &msg) {

  //  std::cout << ">> " << msg->DebugString() << std::endl;
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

  int width = 400;
  int height = 400;
  float px_per_m = 200 / range_max;

  cv::Mat im(height, width, CV_8UC3);
  im.setTo(0);
  for (int i = 0; i < nranges; i++) {
    float angle = angle_min + i * angle_increment;
    float range = std::min(float(msg->scan().ranges(i)), range_max);
    //    double intensity = msg->scan().intensities(i);
    cv::Point2f startpt(200.5f + range_min * px_per_m * std::cos(angle),
                        200.5f - range_min * px_per_m * std::sin(angle));
    cv::Point2f endpt(200.5f + range * px_per_m * std::cos(angle),
                      200.5f - range * px_per_m * std::sin(angle));
    cv::line(im, startpt * 16, endpt * 16, cv::Scalar(255, 255, 255, 255), 1,
             cv::LINE_AA, 4);

    //    std::cout << angle << " " << range << " " << intensity << std::endl;
  }
  cv::circle(im, cv::Point(200, 200), 2, cv::Scalar(0, 0, 255));
  cv::putText(im, std::to_string(sec) + ":" + std::to_string(nsec),
              cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.0,
              cv::Scalar(255, 0, 0));

  mutex.lock();
  cv::imshow("lidar", im);
  mutex.unlock();



  //smallest_dist = msg->scan().ranges(0);
  //smallest_dir = msg->scan().angle_min();
  //float new_angle = smallest_dir;
  //for (int i = 0; i < msg->scan().ranges_size(); i++) {
  //    smallest_dist = (smallest_dist < msg->scan().ranges(i)) ? smallest_dist : msg->scan().ranges(i);
  //    new_angle = new_angle + msg->scan().angle_step();
  //    smallest_dir = (smallest_dist < msg->scan().ranges(i)) ? smallest_dir : new_angle;
  //}
}
*/

int main(int _argc, char **_argv) {

    laserscanner scanner;
    circleDetect circDetect;


  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo topics
  //gazebo::transport::SubscriberPtr statSubscriber =
  //    node->Subscribe("~/world_stats", statCallback);

  gazebo::transport::SubscriberPtr poseSubscriber =
      node->Subscribe("~/pose/info", trackRobot);//poseCallback, trackRobot);

  //gazebo::transport::SubscriberPtr cameraSubscriber =
  //    node->Subscribe("~/pioneer2dx/camera/link/camera/image", cameraCallback);

  //gazebo::transport::SubscriberPtr lidarSubscriber =
  //    node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", lidarCallback);

  gazebo::transport::SubscriberPtr circleDetection =
          node->Subscribe("~/pioneer2dx/camera/link/camera/image", &circleDetect::detect, &circDetect);

  gazebo::transport::SubscriberPtr laserScanSubscriber =
          node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", &laserscanner::performScan, &scanner);

  gazebo::transport::SubscriberPtr laserScanRansac =
          node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", &laserscanner::findLines, &scanner);


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


  //Initialise the controller
  //fuzzy1 controller;
  //fuzzy2 controller2;

  obstacle_avoidance controller;
  controller.init_controller();
  take_marble controller2;
  controller2.init_controller();

  // Loop
  while (true) {
    gazebo::common::Time::MSleep(10);

    mutex.lock();
    int key = cv::waitKey(1);
    mutex.unlock();

    if (key == key_esc)
      break;

    //Determine if a marble is present.
    /*
    if (marbleDist) {
        controller2.setValues(scanner.getClosestScan().distance, scanner.getClosestScan().direction, marbleDist);

        controller2.process();

        speed = controller2.getValues().speed;

        dir = controller2.getValues().direction;

        //std::cout << "Marble" << std::endl;
    } else if (allBlue) {
        speed = speed;
        dir = 0;
        //std::cout << "No AI" << std::endl;
    } else {
    */
        //Set value Odir from getClosestDir and set value Odist from getClosestDist
        //controller.setValues(scanner.getClosestScan().distance, scanner.getClosestScan().direction);
        controller.setValues(scanner.getClosestLine().distance, scanner.getClosestLine().angle);

        //Engine process here.
        controller.process();

        //Speed = get value from speed
        speed = controller.getValues().speed;

        //Steer direction = get value from Sdir
        dir = controller.getValues().direction;

        //std::cout << "No marble" << std::endl;
    //}


/*
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
*/


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
