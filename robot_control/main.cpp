#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>
using namespace cv;

const Point rot[] = {Point(-1, -1), Point(-1, 0), Point(-1, 1), Point(0, 1),
                     Point(1, 1),   Point(1, 0),  Point(1, -1), Point(0, -1)};

static boost::mutex mutex;

void findOutline(Mat *img, int sr, int sc, int r, int c, std::vector<Point> *pts, int dir = 5) {
  if (c == sc && r == sr && !pts->empty())
    return;
  int tdir = 0;
  int nc = 0;
  int nr = 0;
  for (int i = 0; i < 8; i++) {
    tdir = (dir + 5 + i) % 8;
    nc = c + rot[tdir].x;
    nr = r + rot[tdir].y;
    uchar h = img->at<Vec3b>(nr, nc)[0];
    if (100 < h && h < 130) {
      break;
    }
  }
  pts->push_back(Point(nc, nr));
  if (pts->size() > 1000) return;
  return findOutline(img, sr, sc, nr, nc, pts, tdir);
}

void myFloodFill(Mat *img, int *b, int c, int r) {
  uchar h = img->at<Vec3b>(r, c)[0];
  if (h == 45) return;
  if (100 > h || h > 130) return;
  img->at<Vec3b>(r, c)[0] = 45;

  if (c < b[0]) b[0] = c;
  if (c > b[1]) b[1] = c;
  if (r < b[2]) b[2] = r;
  if (r > b[3]) b[3] = r;

  myFloodFill(img, b, c + 1, r);
  myFloodFill(img, b, c - 1, r);
  myFloodFill(img, b, c, r + 1);
  myFloodFill(img, b, c    , r - 1);
  return;
}

Point2f findCentrum(Point a, Point b, Point c) {
  Point2f an(b.x - a.x, b.y - b.y);
  Point2f bn(c.x - b.x, c.y - b.y);
  Point2f aq(a.x + an.x * .5, a.y + an.y * .5);
  Point2f bq(b.x + bn.x * .5, b.y + bn.y * .5);

  float a1 = an.x;
  float b1 = an.y;
  float c1 = an.x * aq.x + an.y * aq.y;

  float a2 = bn.x;
  float b2 = bn.y;
  float c2 = bn.x * bq.x + bn.y * bq.y;

  Mat m1 = (Mat_<float>(2, 2) << c1, b1, c2, b2);
  Mat m2 = (Mat_<float>(2, 2) << a1, c1, a2, c2);
  Mat me = (Mat_<float>(2, 2) << a1, b1, a2, b2);

  float x = determinant(m1) / determinant(me);
  float y = determinant(m2) / determinant(me);

  return Point2f(x, y);
}

void circularRandSac(std::vector<Point> *cfp, std::vector<int*> *m) {
  std::vector<Point> cf = *cfp;

  std::mt19937 rng;
  rng.seed(std::random_device()());
  std::uniform_int_distribution<std::mt19937::result_type> dist(1, cf.size() - 1);

    for(int ittr = 0; ittr < 200; ittr++) {
      if (cf.size() < 10) return;
      int a, b, c;
      while (true) {
        a = dist(rng);
        b = dist(rng);
        c = dist(rng);
        if (a != b && b != c && c != a) break;
      }
      Point2f p = findCentrum(cf[a], cf[b], cf[c]);
      Point2f delta = Point2f(cf[a]) - p;
      double r2 = sqrt(delta.x * delta.x + delta.y * delta.y);

      int count = 0;
      int i = 0;
      float ra = 0;
      for (i; i < cf.size(); i++) {
        Point2f delta = Point2f(cf[i]) - p;
        double r2t = sqrt(delta.x * delta.x + delta.y * delta.y);
        float d = r2 - r2t;
        if (-0.5 < d && d < 0.5){
          count++;

        }
      }
      float per = (float)count / (float)cf.size();
      if (per > 0.3 && count > 20 || count > 100) {
          int n = 1;
          auto predicate = [&p, &r2, &ra, &n](const Point &tp) {
              Point2f delta = Point2f(tp) - p;
              double r2t = sqrt(delta.x * delta.x + delta.y * delta.y);
              float d = r2 - r2t;
              if (-3 < d && d < 3){
                  ra += r2t;
                  n++;
                  return true;
              };
              return false;
          };
           cf.erase(std::remove_if(cf.begin(), cf.end(), predicate), cf.end());
          int* ma = new int[3];
          ma[0] = p.x * 4;
          ma[1] = p.y * 4;
          ma[2] = ra/n * 4;
          m->push_back(ma);
      }
    }


}

double* findDist(int r, int x){
    double c = 160;
    double xmin = c - (x - 0.5 * r);
    double xmax = c - (x + 0.5 * r);

    double smin = sqrt(xmin*xmin + 277*277);
    double smax = sqrt(xmax*xmax + 277*277);

    double angle = .5 * acos((smin*smin + smax*smax - ((double) r*r))/(2*smin*smax));

    double d = 0.5/tan(angle);
    std::cout << d << std::setw(4) << " : ";
    return &d;
}


void searchForMarbles(Mat *img) {
  Mat tempim = Mat(img->rows, img->cols, img->channels());
  cv::cvtColor(*img, tempim, COLOR_BGR2HLS);

  int rows = tempim.rows;
  int cols = tempim.cols;

  std::vector<int *> marbels;
  std::vector<std::vector<Point>> marbelsOL;

  for (int r = 0; r < rows; r++) {
    uchar *value = tempim.ptr(r);
    for (int c = 0; c < cols; c++) {

      uchar h = *value++;
      uchar l = *value++;
      uchar s = *value++;

      if (100 < h && h < 130) {
        std::vector<Point> ol;
        findOutline(&tempim, r, c, r, c, &ol);
        marbelsOL.push_back(ol);

        int *m = new int[4];
        m[0] = m[1] = c;
        m[2] = m[3] = r;
        marbels.push_back(m);
        myFloodFill(&tempim, m, c, r);
      }
    }
  }
  std::vector<int*> mars;
  for (unsigned int i = 0; i < marbelsOL.size(); i++) {

    const Point *pts = (const Point *)Mat(marbelsOL[i]).data;
    int ntps = marbelsOL[i].size(); // Mat(ol).rows;
    polylines(tempim, &pts, &ntps, 1, false, Scalar(255, 255, 255), 1, LINE_AA);

    circularRandSac(&marbelsOL[i], &mars);
  }

  for (unsigned int i = 0; i < mars.size(); i++) {
    int* v = mars[i];
    int x = v[0],
        y = v[1],
        r = v[2];
    Point c(x,y);

    circle(*img, c, r,Scalar(255,255,0),1,LINE_8, 2);
    double* d = findDist(r, x);
    //std::cout << *d << std::setw(4) << " : ";
    std::ostringstream ss;
    ss << d;
    std::string s(ss.str());
    putText(*img, "egern", c,
            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, LINE_AA);

  }
  std::cout << std::endl;

  /*for (unsigned int i = 0; i < marbels.size(); i++) {
    rectangle(*img, Point(marbels[i][0], marbels[i][2]),
              Point(marbels[i][1], marbels[i][3]), Scalar(0, 0, 255), 1,
              LINE_AA);
    putText(*img, "Marble", Point(marbels[i][0], marbels[i][2] - 4),
            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, LINE_AA);
    rectangle(*img, Point(marbels[i][0], marbels[i][2] - 17),
              Point(marbels[i][0] + 51, marbels[i][2]), Scalar(0, 0, 255), 1,
              LINE_AA);
  }*/
  cv::cvtColor(tempim, tempim, COLOR_HLS2BGR);
  imshow("Vision", tempim);
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
      /*
     std::cout << std::setprecision(2) << std::fixed << std::setw(6)
               << _msg->pose(i).position().x() << std::setw(6)
               << _msg->pose(i).position().y() << std::setw(6)
               << _msg->pose(i).position().z() << std::setw(6)
               << _msg->pose(i).orientation().w() << std::setw(6)
               << _msg->pose(i).orientation().x() << std::setw(6)
               << _msg->pose(i).orientation().y() << std::setw(6)
               << _msg->pose(i).orientation().z() << std::endl;
               */
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
  searchForMarbles(&im);
  mutex.lock();
  cv::imshow("camera", im);
  mutex.unlock();
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

    mutex.lock();
    int key = cv::waitKey(1);
    mutex.unlock();

    if (key == key_esc)
      break;

    if ((key == key_up) && (speed <= 1.2f))
      speed += 0.4;
    else if ((key == key_down) && (speed >= -1.2f))
      speed -= 0.4;
    else if ((key == key_right) && (dir <= 0.4f))
      dir += 0.05;
    else if ((key == key_left) && (dir >= -0.4f))
      dir -= 0.05;
    else {
      // slow down
      // speed *= 0.8;
      // dir   *= 0.8;
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
