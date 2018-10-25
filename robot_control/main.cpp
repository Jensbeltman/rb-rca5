#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>

#include <iostream>
using namespace std;
using namespace cv;

static boost::mutex mutex2;

void statCallback(ConstWorldStatisticsPtr &_msg) {
  (void)_msg;
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();
  //  std::cout << std::flush;
}

void poseCallback(ConstPosesStampedPtr &_msg) {
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();

  //  for (int i = 0; i < _msg->pose_size(); i++) {
  //    if (_msg->pose(i).name() == "pioneer2dx") {

  //      std::cout << std::setprecision(2) << std::fixed << std::setw(6)
  //                << _msg->pose(i).position().x() << std::setw(6)
  //                << _msg->pose(i).position().y() << std::setw(6)
  //                << _msg->pose(i).position().z() << std::setw(6)
  //                << _msg->pose(i).orientation().w() << std::setw(6)
  //                << _msg->pose(i).orientation().x() << std::setw(6)
  //                << _msg->pose(i).orientation().y() << std::setw(6)
  //                << _msg->pose(i).orientation().z() << std::endl;
  //    }
  //  }
}
double prev_pitch_l, prev_pitch_r, prev_time;
double delta_pitch_l, delta_pitch_r, delta_time;
double px = 0;
double py = 0;
double phi = 0;

void poseCallbackLocal(ConstPosesStampedPtr &_msg) {
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();

  double time = _msg->time().sec() + 0.000000001 * _msg->time().nsec();
  double pitch_l, pitch_r;

  double vel_l, vel_r;

  for (int i = 0; i < _msg->pose_size(); i++) {
	if (_msg->pose(i).name() == "pioneer2dx::pioneer2dx::right_wheel") {
	  double y = _msg->pose(i).orientation().y();
	  double w = _msg->pose(i).orientation().w();

	  // pitch (y-axis rotation)
	  pitch_r = M_PI - atan2(_msg->pose(i).orientation().w(),
							 _msg->pose(i).orientation().y());
	}
	if (_msg->pose(i).name() == "pioneer2dx::pioneer2dx::left_wheel") {
	  double y = _msg->pose(i).orientation().y();
	  double w = _msg->pose(i).orientation().w();

	  // pitch (y-axis rotation)
	  pitch_l = M_PI - atan2(_msg->pose(i).orientation().w(),
							 _msg->pose(i).orientation().y());
	}
  }

  delta_pitch_l = pitch_l - prev_pitch_l;
  delta_pitch_r = pitch_r - prev_pitch_r;
  delta_time = time - prev_time;

  if (delta_pitch_l > 1.8 * M_PI) {
	delta_pitch_l -= M_PI;
  }
  if (delta_pitch_r > 1.8 * M_PI) {
	delta_pitch_r -= 2 * M_PI;
  }

  if (delta_pitch_l < -1.8 * M_PI) {
	delta_pitch_l += 2 * M_PI;
  }
  if (delta_pitch_r < -1.8 * M_PI) {
	delta_pitch_r += 2 * M_PI;
  }

  vel_l = delta_pitch_l / delta_time;
  vel_r = delta_pitch_r / delta_time;

  //  cout << fixed << setprecision(6) << "pl: " << setw(4) << pitch_l
  //	   << " | pr: " << setw(4) << pitch_r << " ||   vl: " << setw(4) <<
  // vel_l
  //	   << " ||   vr: " << setw(4) << vel_r << endl;

  prev_pitch_l = pitch_l;
  prev_pitch_r = pitch_r;
  prev_time = time;

  const double l = 0.34 + 0.05;
  double R, omega;

  double dpx = 0;
  double dpy = 0;
  double dphi = 0;
  double ICC_x, ICC_y;

  double vl = vel_l * (0.11 * 2);
  double vr = vel_r * (0.11 * 2);

  R = (l / 2) * (vl + vr) / (vr - vl);
  omega = (vr - vl) / l;

  dphi = omega * delta_time + phi;

  ICC_x = px - R * sin(phi);
  ICC_y = py - R * cos(phi);

  dpx = (cos(omega * delta_time) * (px - ICC_x) -
		 sin(omega * delta_time) * (py - ICC_y)) +
		ICC_x;
  dpy = (sin(omega * delta_time) * (px - ICC_x) +
		 cos(omega * delta_time) * (py - ICC_y)) +
		ICC_y;

  px = dpx;
  py = dpy;
  phi = dphi;

  cout << fixed << "X: " << setw(5) << px << ", Y: " << setw(5) << py
	   << ", phi: " << setw(5) << dphi << endl;
}

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

  mutex2.lock();
  cv::imshow("lidar", im);
  mutex2.unlock();
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

  gazebo::transport::SubscriberPtr poseSubscriberLocal =
	  node->Subscribe("~/pose/local/info", poseCallbackLocal);

  gazebo::transport::SubscriberPtr cameraSubscriber =
      node->Subscribe("~/pioneer2dx/camera/link/camera/image", cameraCallback);

  gazebo::transport::SubscriberPtr lidarSubscriber =
      node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", lidarCallback);

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
    gazebo::common::Time::MSleep(10);

	mutex2.lock();
    int key = cv::waitKey(1);
	mutex2.unlock();

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
    movementPublisher->Publish(msg);
  }

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
