#include "marbleutility.h"

MarbleUtility::MarbleUtility() { namedWindow("camera", WINDOW_FREERATIO); }

void MarbleUtility::getDistrParam() {
  cout << "Enter number of marbles" << endl;
  cin >> numberOfMarbles;
  cout << "Enter number of tests" << endl;
  cin >> numberOfTests;
  cout << "Enter angular range" << endl;
  cin >> angRange;
  cout << "Enter lower distance range" << endl;
  cin >> distL;
  cout << "Enter upper distance range" << endl;
  cin >> distU;

  for (int i = 0; i < numberOfMarbles; i++) {
	Marble m = {"marble_clone_" + to_string(i), (0, 0, 0)};
	marble.push_back(m);
  }
}

void MarbleUtility::distributeMarbles(
	gazebo::transport::PublisherPtr movePublisher) {
  std::srand(std::time(0));

  gazebo::msgs::Pose msg;
  for (int i = 0; i < marble.size(); i++) {
	marble[i].robotDistance = static_cast<float>(
		distL + rand() / (RAND_MAX / (float)(distU - distL)));

	marble[i].robotAngle =
		M_PI / 4.0 +
		(((M_PI / 12) - static_cast<float>(rand()) /
							(static_cast<float>(RAND_MAX / (M_PI / 6)))));

	float randX = rx + cos(marble[i].robotAngle) * marble[i].robotDistance;
	float randY = ry + sin(marble[i].robotAngle) * marble[i].robotDistance;

	ignition::math::Pose3d pose(randX, randY, 0.5, 0, 0, 0);
	cout << "Pose " << i << " is " << pose << endl;
	gazebo::msgs::Set(&msg, pose);
	msg.set_name(marble[i].name);
	movePublisher->Publish(msg);
  }
}

void MarbleUtility::findMarbles() {
  detectedMarble.clear();
  Mat hls;
  cvtColor(src, hls, COLOR_BGR2HLS);
  vector<Mat> hlsChannels(3);
  split(hls, hlsChannels);

  GaussianBlur(hlsChannels[2], hlsChannels[2], Size(5, 5), 2);
  vector<Vec3f> circles;
  HoughCircles(hlsChannels[2], circles, HOUGH_GRADIENT, 1,
			   1,  // change this value to detect circles with
				   // different distances to each other
			   200, 30, 0,
			   0  // change the last two parameters
				  // (min_radius & max_radius) to detect larger circles
  );

  float imgXmid = src.cols / 2;
  float fl = 277.13;
  float flsq = fl * fl;

  Mat display = src.clone();
  for (size_t i = 0; i < circles.size(); i++) {
	Vec3i c = circles[i];
	Point center = Point(c[0], c[1]);
	// circle center
	circle(display, center, 1, Scalar(0, 100, 100), 3, LINE_AA);
	// circle outline
	float r = c[2];
	circle(display, center, (int)r, Scalar(255, 0, 255), 3, LINE_AA);

	float lmin = sqrt(pow(imgXmid - center.x - r, 2.0f) + flsq);
	float lmax = sqrt(pow(imgXmid - center.x + r, 2.0f) + flsq);

	float theta = atan(r / fl);
	float camdist = (0.5 / tan(theta));

	float cam2marbleAngle = atan(fabs(imgXmid - center.x) / fl);

	float dist = sqrt(camdist * camdist + 0.1 * 0.1 -
					  2 * camdist * 0.1 * cos(M_PI - cam2marbleAngle));
	ostringstream ss;
	ss << dist;
	string s(ss.str());
	putText(src, s, center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1,
			LINE_AA);

	Marble m;
	m.name = "detected_marble_" + to_string(i);
	m.pose[0] = rx + cos(cam2marbleAngle) * dist;
	m.pose[1] = ry + sin(cam2marbleAngle) * dist;

	detectedMarble.push_back(m);
  }

  imshow("camera", display);
}
bool marbleDist(const Marble &a, const Marble &b) {
  return (a.robotDistance > b.robotDistance);
}

bool marbleAng(const Marble &a, const Marble &b) {
  return (a.robotAngle > b.robotAngle);
}
