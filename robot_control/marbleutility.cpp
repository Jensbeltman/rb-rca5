#include "marbleutility.h"

MarbleUtility::MarbleUtility() {
  namedWindow("camera", WINDOW_FREERATIO);
  namedWindow("display", WINDOW_FREERATIO);
  namedWindow("live", WINDOW_FREERATIO);
  srand(std::time(NULL));
  src_hls_channels.resize(3);
  marbleData.open(dataName + ".txt");

  for (int i = 0; i < 20; i++) {  // hardcode 20 from smallworld file
	Marble m = {"marble_clone_" + to_string(i), (0, 0, 0)};
	marble.push_back(m);
  }
}

MarbleUtility::~MarbleUtility() {
  marbleData.close();
  cout << "Data saved as " + dataName + ".txt" << endl;
}

void MarbleUtility::cameraCallback(ConstImageStampedPtr &msg) {
  size_t width = msg->image().width();
  size_t height = msg->image().height();
  const char *data = msg->image().data().c_str();
  Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));
  cvtColor(im, im, COLOR_RGB2BGR);
  Mat adiff, im_hls;
  vector<Mat> im_hls_channels;

  cvtColor(im, im_hls, COLOR_BGR2HLS);
  split(im_hls, im_hls_channels);

  if (first) {
	first = false;
	src = im.clone();
	src_hls = im_hls.clone();
	src_hls_channels = im_hls_channels;
	display = im.clone();
  } else {
	absdiff(im_hls_channels[2], src_hls_channels[2], adiff);
	Scalar ssum = sum(adiff);
	// cout << "Diff is " << ssum << endl;
	src = im.clone();
	src_hls = im_hls.clone();
	src_hls_channels = im_hls_channels;
	display = im.clone();
	int threshold = 10;

	if (ssum[0] > threshold) newData = true;
  }

  imshow("live", src);
}

void MarbleUtility::getDistrParam() {
  cout << "Enter number of marbles" << endl;
  cin >> numberOfMarbles;
  //  cout << "Enter number of tests" << endl;
  //  cin >> numberOfTests;
  //  cout << "Enter angular range" << endl;
  //  cin >> angRange;
  //  cout << "Enter lower distance range" << endl;
  //  cin >> distL;
  //  cout << "Enter upper distance range" << endl;
  //  cin >> distU;

  for (int i = 0; i < numberOfMarbles; i++) {
	Marble m = {"marble_clone_" + to_string(i), (0, 0, 0)};
	marble.push_back(m);
  }
}

void MarbleUtility::distributeMarbles(
	gazebo::transport::PublisherPtr movePublisher) {
  gazebo::msgs::Pose msg;
  for (int i = 0; i < marble.size(); i++) {
	marble[i].robotDistance = static_cast<float>(
		distL + rand() / (RAND_MAX / (float)(distU - distL)));

	marble[i].robotAngle =
		M_PI / 12.0 + static_cast<float>(rand()) /
						  (static_cast<float>(RAND_MAX / (M_PI / 3)));

	marble[i].pose[0] =
		rx + cos(marble[i].robotAngle) * marble[i].robotDistance;
	marble[i].pose[1] =
		ry + sin(marble[i].robotAngle) * marble[i].robotDistance;

	ignition::math::Pose3d pose(marble[i].pose[0], marble[i].pose[1], 0.5, 0, 0,
								0);
	// cout << "Pose " << i << " is " << pose << endl;
	gazebo::msgs::Set(&msg, pose);
	msg.set_name(marble[i].name);
	movePublisher->Publish(msg);
  }
  // imshow("redist", src);
}

void MarbleUtility::distributeMarble(
	gazebo::transport::PublisherPtr movePublisher, int mI, float dist,
	float ang) {
  gazebo::msgs::Pose msg;

  marble[mI].robotDistance = dist;

  marble[mI].robotAngle = ang;

  marble[mI].pose[0] =
	  rx + cos(marble[mI].robotAngle) * marble[mI].robotDistance;
  marble[mI].pose[1] =
	  ry + sin(marble[mI].robotAngle) * marble[mI].robotDistance;

  ignition::math::Pose3d pose(marble[mI].pose[0], marble[mI].pose[1], 0.5, 0, 0,
							  0);
  // cout << "Pose " << mI << " is " << pose << endl;
  gazebo::msgs::Set(&msg, pose);
  msg.set_name(marble[mI].name);
  movePublisher->Publish(msg);
}

void MarbleUtility::findMarbles() {
  detectedMarble.clear();
  Mat hls;

  cvtColor(src, hls, COLOR_BGR2HLS);
  vector<int *> boxes;
  for (int r = 0; r < hls.rows; r++) {
	for (int c = 0; c < hls.cols; c++) {
	  int hue = hls.at<Vec3b>(r, c)[0];
	  if (hue > 90 && hue < 140) {
		int *box = new int[4];
		box[0] = box[1] = c;
		box[2] = box[3] = r;
		boxes.push_back(box);
		myFloodFill(&hls, box, c, r);
		imshow("floodfill", hls);
	  }
	}
  }

  vector<float *> peeks;
  for (int i = 0; i < boxes.size(); i++) {
	rectangle(display, Point(boxes[i][0], boxes[i][2]),
			  Point(boxes[i][1], boxes[i][3]), Scalar(0, 0, 255));

	int p_start, p_end, h, h_p = 0;
	bool onPeek = false;
	for (int c = boxes[i][0]; c <= boxes[i][1]; c++) {
	  h = 0;
	  for (int r = boxes[i][2]; r <= boxes[i][3]; r++) {
		if (hls.at<Vec3b>(r, c)[0] ==
			45)  // 45 is what marbles are set to in myFLoodFill
		{
		  h++;
		}
	  }
	  if (h > h_p) {
		p_start = c;
		onPeek = true;
	  }
	  if (h < h_p && onPeek) {
		onPeek = false;
		float *peek = new float[2];
		p_end = c - 1;
		peek[0] = ((float)p_start + (float)((p_end - p_start) / 2.0f)) +
				  0.5f;  // 0.5 to get the "middle of the pixel"
		peek[1] = (float)h_p;
		peeks.push_back(peek);
		line(display, Point(peek[0], boxes[i][2]), Point(peek[0], boxes[i][3]),
			 Scalar(0, 255, 0));
	  }
	  h_p = h;
	}
  }

  float imgXmid = src.cols / 2;
  float fl = 277.13;

  for (int i = 0; i < peeks.size(); i++) {
	float *peek = peeks[i];
	float mid_d = fabs(imgXmid - peek[0]);
	float pr = (peek[1] / 2);

	float cAV = atan(mid_d / fl);
	float cAH = atan(pr / (fl / cos(cAV)));
	float cD = 0.5 / tan(cAH);

	float b = cD;
	float c = 0.2;
	float bs = b * b;
	float cs = c * c;
	float A = M_PI - fabs(cAV);

	float dist = sqrt(bs + cs - 2 * b * c * cos(A));
	float a = dist;
	float as = a * a;
	float ang;
	if (peek[0] < imgXmid)
	  ang = ro + safeAcos((cs + as - bs) / (2 * a * c));
	else
	  ang = ro - safeAcos((cs + as - bs) / (2 * a * c));

	cout << "Angle is " << ang * 57.2957795 << endl;
	cout << "Dist is " << dist << endl;
	ostringstream ss;
	ss << dist;
	string s(ss.str());
	// putText(display, s, Point(peek[0], display.rows / 2),
	// FONT_HERSHEY_SIMPLEX,
	//			0.5, Scalar(0, 0, 255), 1, LINE_AA);

	Marble m;
	m.name = "detected_marble_" + to_string(i);

	m.pose[0] = rx + cos(ang) * dist;
	m.pose[1] = ry + sin(ang) * dist;

	//	cout << "DPose " << i << " is " << m.pose[0] << " " << m.pose[1] <<
	// endl; 	cout << "Diff is "
	//		 << sqrt(pow(m.pose[0] - marble[0].pose[0], 2) +
	//				 pow(m.pose[1] - marble[0].pose[1], 2))
	//		 << endl;

	marbleData << marble[0].pose[0] << "," << marble[0].pose[1] << ","
			   << marble[0].robotDistance << "," << marble[0].robotAngle << ","
			   << m.pose[0] << "," << m.pose[1] << "," << dist << "," << ang
			   << "\n";

	detectedMarble.push_back(m);
  }
  imshow("display", display);
}

void MarbleUtility::myFloodFill(Mat *img, int *b, int c, int r) {
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
  myFloodFill(img, b, c, r - 1);
  return;
}

void MarbleUtility::matchMarbles() {
  pairs.clear();
  for (int dm = 0; dm < detectedMarble.size(); dm++) {
	for (int m = 0; m < marble.size(); m++) {
	  pairs.push_back(make_pair(&detectedMarble[dm], &marble[m]));
	}
  }
  sort(pairs.begin(), pairs.end(), &pairDist);
  pairs.resize(detectedMarble.size());

  for (int i = 0; i < pairs.size(); i++) {
	//	cout << pairs[i].second->name << " found with Error "
	//		 << distance(*(pairs[i].first), *(pairs[i].second));
  }
}

float MarbleUtility::distance(Marble a, Marble b) {
  return sqrt(pow(a.pose[0] - b.pose[0], 2) + pow(a.pose[1] - b.pose[1], 2));
}
float MarbleUtility::distance(Marble *a, Marble *b) {
  return sqrt(pow(a->pose[0] - b->pose[0], 2) +
			  pow(a->pose[1] - b->pose[1], 2));
}

bool MarbleUtility::pairDist(const pair<Marble *, Marble *> &a,
							 const pair<Marble *, Marble *> &b) {
  MarbleUtility mu;
  Marble a1 = *a.first, a2 = *a.second, b1 = *b.first, b2 = *b.second;
  return (mu.distance(a1, a2) < mu.distance(b1, b2));
}

float MarbleUtility::safeAcos(float x) {
  if (x < -1.0f)
	x = -1.0f;
  else if (x > 1.0f)
	x = 1.0f;
  return acos(x);
}
bool marbleDist(const Marble &a, const Marble &b) {
  return (a.robotDistance > b.robotDistance);
}

bool marbleAng(const Marble &a, const Marble &b) {
  return (a.robotAngle > b.robotAngle);
}
