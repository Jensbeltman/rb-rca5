#include "mp.h"

MP::MP(Mat bmap) {
  namedWindow("map", WINDOW_FREERATIO);
  namedWindow("rectmap", WINDOW_FREERATIO);
  namedWindow("testmap", WINDOW_FREERATIO);
  namedWindow("mask", WINDOW_FREERATIO);
  namedWindow("testmap2", WINDOW_FREERATIO);
  cvtColor(bmap, bitmap, COLOR_BGR2GRAY);

  cnrheatmap = bmap;
  resize(bitmap, visionMap, bitmap.size(), 0, 0, INTER_NEAREST);
  display = bmap.clone();
  display2 = bmap.clone();
  cornerkernel = (Mat_<uchar>(2, 2) << 1, 3, 7, 5);
  visCnr = new vector<corner*>*[bitmap.cols];
  for (int i = 0; i < bitmap.cols; i++) {
	visCnr[i] = new vector<corner*>[bitmap.rows];
  }

  int mScale = 1;

  findCorners(bitmap, cnr);
  findAreas(bitmap, area);
  drawRect(display2);

  // cnrHeatC();
  genVisMap(bitmap, visionMap, mScale);
  //  genVisCnr();
  // visionMap();

  localizor = Localizor(bitmap, cnrheatmap);

  node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node->Init();

  poseSubscriber =
	  node->Subscribe("~/pose/info", &Localizor::poseCallback, &localizor);
  localPoseSubscriber = node->Subscribe(
	  "~/pose/local/info", &Localizor::localPoseCallback, &localizor);
  lidarSubscriber =
	  node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan",
					  &Montecarlo::parseScan, &localizor.montecarlo);
  cout << "MP created" << endl;
}

void MP::genVisMap(Mat src, Mat vMap, int mapScale) {
  resize(src, vMap, src.size() * mapScale, 0, 0, INTER_NEAREST);
  vMap.convertTo(vMap, CV_32FC1);

  const int lineScale = 1;

  //  Mat lMap;
  //  resize(src, lMap, src.size() * lineScale, 0, 0, INTER_NEAREST);

  //  vector<vector<Point>> contourArr;

  //  lMap = 255 - lMap;
  //  findContours(lMap, contourArr, RETR_TREE, CHAIN_APPROX_SIMPLE);
  vector<corner> cnr_t;
  vector<segment_type> lines;
  const Point2f offset = Point2f(0.5, 0.5);

  findCorners(bitmap, cnr_t);
  bool x;
  Point p1, p2;
  corner sc, cc, nc;
  while (!cnr_t.empty()) {
	x = true;
	sc = cnr_t[0];
	cc = nextCorner(sc, cnr_t, x);
	nc = cc;
	p1 = cnr2pos(sc);
	p2 = cnr2pos(cc);
	lines.push_back(segment_type({{p1.x, p1.y}, {p2.x, p2.y}}));

	while (nc.P != sc.P) {
	  x = !x;
	  nc = nextCorner(cc, cnr_t, x);
	  p1 = cnr2pos(cc);
	  p2 = cnr2pos(nc);
	  if (nc.P == sc.P) {
		lines.push_back(segment_type({{p1.x, p1.y}, {p2.x, p2.y}}));
		removeCnr(cc, cnr_t);
		removeCnr(nc, cnr_t);
		break;
	  } else {
		lines.push_back(segment_type({{p1.x, p1.y}, {p2.x, p2.y}}));
		removeCnr(cc, cnr_t);
		cc = nc;
	  }
	}
  }

  float m2l = 1.0 / mapScale;
  float l2m = (float)mapScale / 1.0f;

  float area = 0;

  for (int x = 0; x < vMap.cols; x += 1) {
	cout << x << endl;
	for (int y = 0; y < vMap.rows; y += 1) {
	  area = 0;
	  if (vMap.at<float>(y, x) != 0) {
		float X = x + 0.5;
		float Y = y + 0.5;
		auto XY = vec2(X, Y);
		vec2 XYS = XY * m2l;
		vector<vec2> vision =
			visibility_polygon(vector_type{XYS}, lines.begin(), lines.end());

		for (uint i = 1; i < vision.size(); i++) {
		  vec2 a{XYS - vision[i - 1]}, b{XYS - vision[i]};
		  area += abs(cross(a, b)) / 2;
		}
		vec2 a{XYS - vision[vision.size() - 1]}, b{XYS - vision[0]};

		area += abs(cross(a, b)) / 2;

		vMap.at<float>(y, x) = area;
	  }
	}
  }
  // cout << "channels" << vMap.channels();

  Mat outImg;

  // imshow("testmap", vMap);
  normalize(vMap, vMap, 0, 1, NORM_MINMAX);
  localMaxima(vMap, mxVisPts, true);
  imshow("map", vMap);

  // cout << "mxVisPts.size()=" << mxVisPts;
  mxVisPoly.resize(mxVisPts.size());
  for (uint i = 0; i < mxVisPts.size(); i++) {
	float X = mxVisPts[i].x + 0.5;
	float Y = mxVisPts[i].y + 0.5;
	auto XY = vec2(X, Y);
	vec2 XYS = XY * m2l;
	vector<vec2> vision =
		visibility_polygon(vector_type{XYS}, lines.begin(), lines.end());

	mxVisPoly[i].first = mxVisPts[i];
	mxVisPoly[i].second.resize(vision.size());
	for (uint j = 0; j < vision.size(); j++) {
	  mxVisPoly[i].second[j] = Point(vision[j].x * l2m, vision[j].y * l2m);
	}
	display.at<Vec3b>(mxVisPts[i]) = Vec3b(0, 200, 0);
  }
  sort(mxVisPoly.begin(), mxVisPoly.end(), &mostVision);

  Mat temp = display.clone();
  float coverage = 0, temp_coverage = 0;

  cout << "The amount of maximas is: " << mxVisPoly.size() << endl;
  for (int i = 0; i < mxVisPoly.size(); i++) {
	drawPoly2(temp, mxVisPoly[i].second, Scalar(200, 0, 0));
	float b = 0;
	for (int x = 0; x < temp.cols; x++) {
	  for (int y = 0; y < temp.rows; y++) {
		if (temp.at<Vec3b>(y, x) != Vec3b(255, 255, 255)) b++;
	  }
	}
	temp_coverage = b / (float)(temp.rows * temp.cols);

	if ((temp_coverage - coverage) > 0.1) {
	  drawPoly2(display, mxVisPoly[i].second, Scalar(200, 0, 0));
	  coverage = temp_coverage;
	} else
	  temp = display.clone();

	cout << "Point " << i << " coverage is " << setw(5) << coverage << endl;
	imshow("testmap", display);
	waitKey();
  }
}

corner MP::nextCorner(corner cc, vector<corner> cnrs, bool x) {
  corner c, nc;
  bool first = true;
  if (cc.type == 'o') {
	cc.D.x = cc.D.x * -1;
	cc.D.y = cc.D.y * -1;
  }

  if (x) {
	for (int i = 0; i < cnrs.size(); i++) {
	  c = cnrs[i];
	  if (cc.D.x == 1)
		if (c.P.x > cc.P.x && c.P.y == cc.P.y)
		  if (nc.P.x > c.P.x || first) {
			nc = c;
			first = false;
		  }
	  if (cc.D.x == -1)
		if (c.P.x < cc.P.x && c.P.y == cc.P.y)
		  if (nc.P.x < c.P.x || first) {
			nc = c;
			first = false;
		  }
	}
  }
  if (!x)
	for (int i = 0; i < cnrs.size(); i++) {
	  c = cnrs[i];
	  if (cc.D.y == 1)
		if (c.P.y > cc.P.y && c.P.x == cc.P.x)
		  if (nc.P.y > c.P.y || first) {
			nc = c;
			first = false;
		  }
	  if (cc.D.y == -1)
		if (c.P.y < cc.P.y && c.P.x == cc.P.x)
		  if (nc.P.y < c.P.y || first) {
			nc = c;
			first = false;
		  }
	}

  return nc;
}

void MP::removeCnr(corner c, vector<corner>& cnrs) {
  for (int i = 0; i < cnrs.size(); i++)
	if (cnrs[i].P == c.P) cnrs.erase(cnrs.begin() + i);
}

Point MP::cnr2pos(corner c) {
  Point np;
  if (c.D == Point(-1, -1))
	return (c.P + Point(1, 1));
  else if (c.D.x == -1)
	return (c.P + Point(1, 0));
  else if (c.D.y == -1)
	return (c.P + Point(0, 1));
  else
	return c.P;
}

void MP::genVisPoly() {}

void MP::localMaxima(const Mat image, vector<Point>& pts,
					 bool remove_plateaus) {
  Mat mask(image.size(), CV_32FC1);

  // find pixels that are equal to the local neighborhood not maximum
  // (including 'plateaus')
  Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(12, 12));
  dilate(image, mask, kernel);
  compare(image, mask, mask, cv::CMP_GE);
  // optionally filter out pixels that are equal to the local minimum
  // ('plateaus')
  if (remove_plateaus) {
	Mat non_plateau_mask;
	erode(image, non_plateau_mask, kernel);
	compare(image, non_plateau_mask, non_plateau_mask, cv::CMP_GT);
	bitwise_and(mask, non_plateau_mask, mask);
  }

  // normalize(mask, mask, 0, 1, NORM_MINMAX);
  imshow("mask", mask);

  mask.convertTo(mask, CV_8SC1);
  cout << "channels" << mask.channels();
  for (int x = 0; x < mask.cols; x++) {
	for (int y = 0; y < mask.rows; y++) {
	  int n = 0;
	  // cout << mask.at<float>(y, x) << ",";
	  if (mask.at<uchar>(y, x) > 0) {
		cout << mask.at<uchar>(y, x) << "," << endl;
		// cout << n++ << ",";

		pts.push_back(Point(x, y));
	  }
	}
  }
}

void MP::drawPoly2(Mat image, vector<Point>& polygon, Scalar color) {
  const Point* pts = (const Point*)Mat(polygon).data;
  int npts = Mat(polygon).rows;
  if (npts > 0) fillPoly(image, &pts, &npts, 1, color);
}

void MP::findAreas(Mat map, vector<Rect>& A) {
  Mat map_t = map.clone();
  vector<corner> cnr_t;
  vector<Rect> area_t;
  while (findCorners(map_t, cnr_t)) {
	area_t.clear();

	for (int i = 0; i < cnr_t.size(); i++) {
	  corner* c = &cnr_t[i];
	  int dx = 0;
	  int dy = 0;
	  while ((map_t.at<uchar>(c->P.y, c->P.x + (c->D.x * dx++))) != 0)
		;
	  while ((map_t.at<uchar>(c->P.y + (c->D.y * dy++), c->P.x)) != 0)
		;

	  int lx = dx;
	  int ly = dy;

	  for (int x = 0; x < dx; x++) {
		if ((isBlackBetween(map_t, c->P + Point(c->D.x * x, 0),
							Point(0, c->D.y), ly))) {
		  cout << x << "," << ly << endl;

		  Point ul;
		  if (c->D.x > 0) {
			ul.x = c->P.x;
		  } else if (c->D.x < 0) {
			ul.x = c->P.x - (x - 1);
		  }
		  if (c->D.y > 0) {
			ul.y = c->P.y;
		  } else if (c->D.y < 0) {
			ul.y = c->P.y - (ly - 1);
		  }

		  area_t.push_back(Rect(ul, Size(x, ly)));

		  while (isBlackBetween(map_t, c->P + Point(c->D.x * x, 0),
								Point(0, c->D.y), ly) &&
				 ly != 0) {
			--ly;
		  }
		}
	  }

	  for (int y = 0; y < dy; y++) {
		if ((isBlackBetween(map_t, c->P + Point(0, c->D.y * y),
							Point(c->D.x, 0), lx))) {
		  cout << y << "," << lx << endl;

		  Point ul;
		  if (c->D.x > 0) {
			ul.x = c->P.x;
		  } else if (c->D.x < 0) {
			ul.x = c->P.x - (lx - 1);
		  }
		  if (c->D.y > 0) {
			ul.y = c->P.y;
		  } else if (c->D.y < 0) {
			ul.y = c->P.y - (y - 1);
		  }

		  area_t.push_back(Rect(ul, Size(lx, y)));

		  while (isBlackBetween(map_t, c->P + Point(0, c->D.y * y),
								Point(c->D.x, 0), lx) &&
				 lx != 0) {
			--lx;
		  }
		}
	  }

	  sort(area_t.begin(), area_t.end(), &largestArea);
	  area_t.resize(1);
	}
	A.push_back(area_t[0]);
	rectangle(map_t, area_t[0], 0, FILLED);
	// rectangle(display, area_t[0], Scalar(0, 255, 0), FILLED);
	// imshow("testmap", display);
	// waitKey(0);
	cnr_t.clear();
  }
}

bool MP::isBlackBetween(Mat m, Point a, Point d, int l) {
  for (int i = 0; i < l; i++) {
	if (d.y == 0) {
	  if (m.at<uchar>(a.y, a.x + (d.x * i)) == 0) return true;
	} else {
	  if (m.at<uchar>(a.y + (d.y * i), a.x) == 0) return true;
	}
  }
  return false;
}

void MP::drawRect(Mat m) {
  for (int i = 0; i < area.size(); i++) {
	rectangle(m, area[i],
			  Scalar(rand() % 235 + 20, rand() % 235 + 20, rand() % 235 + 20),
			  FILLED);
  }
  //  imshow("rectmap", m);
  //  waitKey(0);
  for (int i = 0; i < area.size() - 1; i++) {
	Rect a = area[i];
	for (int j = i + 1; j < area.size(); j++) {
	  Rect b = area[j];
	  if (a.x + a.width == b.x && a.y + a.height > b.y &&
			  b.y + b.height > a.y ||
		  b.x + b.width == a.x && a.y + a.height > b.y &&
			  b.y + b.height > a.y ||
		  a.y + a.height == b.y && a.x + a.width > b.x && b.x + b.width > a.x ||
		  b.y + b.height == a.y && a.x + a.width > b.x && b.x + b.width > a.x) {
		line(m, Point(a.x + a.width / 2, a.y + a.height / 2),
			 Point(b.x + b.width / 2, b.y + b.height / 2), Scalar(255, 0, 0), 1,
			 LINE_8);
	  }
	}
  }
  imshow("rectmap", m);
}

bool MP::mostVision(const pair<Point, vector<Point>>& a,
					const pair<Point, vector<Point>>& b) {
  return (contourArea(a.second) > contourArea(b.second));
}

bool MP::largestArea(const Rect& a, const Rect& b) {
  return (a.area() > b.area());
}

void MP::cnrHeatC() {
  vector<pair<Point, LineIterator>> lscan;
  const int r = 14;

  for (uint c = 0; c < cnr.size(); c++) {
	corner C = cnr[c];

	int x0 = C.P.x;  // cnr[c].P.x;
	int y0 = C.P.y;  // cnr[c].P.y;
	int cx = r - 1;
	int cy = 0;
	int cdx = 1;
	int cdy = 1;
	int err = cdx - (r << 1);
	// cout << cx - cy << endl;
	while (cx >= cy) {
	  if (C.type == 'i') {
		lscan.push_back(make_pair(
			C.P, LineIterator(cnrheatmap, C.P,
							  Point(x0 + C.D.x * cy, y0 + C.D.y * cx), 4)));
		lscan.push_back(make_pair(
			C.P, LineIterator(cnrheatmap, C.P,
							  Point(x0 + C.D.x * cx, y0 + C.D.y * cy), 4)));
	  }
	  if (C.type == 'o') {
		lscan.push_back(make_pair(
			C.P, LineIterator(cnrheatmap, C.P,
							  Point(x0 + C.D.x * cy, y0 + C.D.y * cx), 4)));
		lscan.push_back(make_pair(
			C.P, LineIterator(cnrheatmap, C.P,
							  Point(x0 + C.D.x * cx, y0 + C.D.y * cy), 4)));
		lscan.push_back(make_pair(
			C.P, LineIterator(cnrheatmap, C.P,
							  Point(x0 - C.D.x * cy, y0 + C.D.y * cx), 4)));
		lscan.push_back(make_pair(
			C.P, LineIterator(cnrheatmap, C.P,
							  Point(x0 - C.D.x * cx, y0 + C.D.y * cy), 4)));
		lscan.push_back(make_pair(
			C.P, LineIterator(cnrheatmap, C.P,
							  Point(x0 + C.D.x * cy, y0 - C.D.y * cx), 4)));
		lscan.push_back(make_pair(
			C.P, LineIterator(cnrheatmap, C.P,
							  Point(x0 + C.D.x * cx, y0 - C.D.y * cy), 4)));
	  }

	  if (err <= 0) {
		cy++;
		err += cdy;
		cdy += 2;
	  }

	  if (err > 0) {
		cx--;
		cdx += 2;
		err += cdx - (r << 1);
	  }
	}
  }
  double cr = 1;
  while (lscan.size() > 0) {
	for (auto ittr = lscan.begin(); ittr != lscan.end(); ++ittr) {
	  uchar* pix = &**ittr->second;
	  if (pix[1] == 0)
		ittr = lscan.erase(ittr);
	  else {
		double nrm = norm(ittr->first - ittr->second.pos());
		int light = (255 * nrm) / (r - 2);
		if (pix[0] < light && pix[0] != 255) {
		  ittr->second++;
		  if (light > 255) ittr = lscan.erase(ittr);
		} else {
		  if (light < 255) {
			pix[0] = light;
			pix[1] = 255;
			pix[2] = light;
			if (nrm < cr) ittr->second++;
		  } else {
			ittr = lscan.erase(ittr);
		  }
		}
	  }
	}
	//	imshow("map", cnrheatmap);
	//	waitKey(20);
	cr += .8;
	if (cr >= r - 1) lscan.clear();
  }
}

void MP::localPoseCallback(ConstPosesStampedPtr& _msg) {
  localizor.localPoseCallback(_msg);
}

void MP::poseCallback(ConstPosesStampedPtr& _msg) {
  localizor.poseCallback(_msg);
}
int MP::findCorners(Mat map, vector<corner>& CNRS) {
  Mat m;
  if (map.channels() != 1) {
	cvtColor(map, m, COLOR_BGR2GRAY);
	cout << m.channels() << endl;
  } else
	m = map;

  const float a1 = 1 * 0.25 * M_PI;
  const float a2 = 3 * 0.25 * M_PI;
  const float a3 = 5 * 0.25 * M_PI;
  const float a4 = 7 * 0.25 * M_PI;

  CNRS.clear();
  corner C;
  int kVal = 0;
  for (int r = 0; r < m.rows - 1; r++) {
	for (int c = 0; c < m.cols - 1; c++) {
	  kVal = 0;

	  for (int x = 0; x < cornerkernel.cols; x++) {
		for (int y = 0; y < cornerkernel.rows; y++) {
		  kVal +=
			  cornerkernel.at<uchar>(y, x) * (m.at<uchar>(r + y, c + x) / 255);
		}
	  }

	  switch (kVal) {
		case 15:
		  C.P = Point(c + 1, r + 1);
		  C.D = Point(1, 1);
		  C.type = 'o';
		  C.angle = a1;
		  CNRS.push_back(C);
		  break;
		case 13:
		  C.P = Point(c, r + 1);
		  C.D = Point(-1, 1);
		  C.angle = a2;
		  C.type = 'o';
		  CNRS.push_back(C);
		  break;
		case 9:
		  C.P = Point(c + 1, r);
		  C.D = Point(1, -1);
		  C.type = 'o';
		  C.angle = a4;
		  CNRS.push_back(C);
		  break;
		case 11:

		  C.P = Point(c, r);
		  C.D = Point(-1, -1);
		  C.type = 'o';
		  C.angle = a3;
		  CNRS.push_back(C);
		  break;

		case 5:

		  C.P = Point(c + 1, r + 1);
		  C.D = Point(1, 1);
		  C.type = 'i';
		  C.angle = a1;
		  CNRS.push_back(C);
		  break;
		case 7:
		  C.P = Point(c, r + 1);
		  C.D = Point(-1, 1);
		  C.type = 'i';
		  C.angle = a2;
		  CNRS.push_back(C);
		  break;
		case 3:
		  C.P = Point(c + 1, r);
		  C.D = Point(1, -1);
		  C.type = 'i';
		  C.angle = a4;
		  CNRS.push_back(C);
		  break;
		case 1:
		  C.P = Point(c, r);
		  C.D = Point(-1, -1);
		  C.type = 'i';
		  C.angle = a3;
		  CNRS.push_back(C);
		  break;

		default:
		  break;
	  }
	}
  }
  return CNRS.size();
}
void MP::drawCorners() {
  namedWindow("terst", WINDOW_FREERATIO);
  for (uint i = 0; i < cnr.size(); i++) {
	if (cnr[i].type == 'i') {
	  display.at<Vec3b>(cnr[i].P) = Vec3b(0, 255, 0);
	} else {
	  display.at<Vec3b>(cnr[i].P) = Vec3b(255, 0, 0);
	}
  }
}
