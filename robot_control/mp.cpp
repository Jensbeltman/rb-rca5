#include "mp.h"

MP::MP(Mat bmap, string n) {
  name = n;
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

  int mScale = 1;

  findCorners(bitmap, cnr);
  findAreas(bitmap, area);
  drawRect(display2);

  cnrHeatC();
  genVisMap(bitmap, visionMap, lines, mScale);
  genVisPoints(bitmap, visionMap, lines, mxVisPts, mxVisPoly, 0.95f);

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
  cout << "MP " + name + " created" << endl;
}

void MP::genVisMap(Mat src, Mat& vMap, vector<segment_type>& lines,
				   int mapScale) {
  vector<corner> cnr_t;

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

  if (!imread2(name + "_vMap.png", vMap)) {
	resize(src, vMap, src.size() * mapScale, 0, 0, INTER_NEAREST);
	vMap.convertTo(vMap, CV_32FC1);
	float area = 0;

	for (int x = 0; x < vMap.cols; x += 1) {
	  cout << '\r' << std::setw(2) << std::setfill('0')
		   << "Vision Map " + name + " progress : "
		   << (int)((float)(x * 100) / (float)vMap.cols) << '%' << flush;
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

		  vMap.at<float>(y, x) = area;
		}
	  }
	}
	normalize(vMap, vMap, 0, 1, NORM_MINMAX);
	vMap.convertTo(vMap, CV_16U, 65535.0);
	imwrite2(name + "_vMap.png", vMap);
	cout << endl;
  }
}

void MP::genVisPoints(Mat bitmap, Mat& vMap, vector<segment_type>& lines,
					  vector<Point>& pts,
					  vector<pair<Point, vector<Point>>>& visPoly,
					  float cov_thresh) {
  vector<Point> tempPoints;
  vector<pair<Point, vector<Point>>> tempVisPoly;
  localMaxima(vMap, tempPoints, true, 3, 3);
  imshow("map", vMap);

  genVisPoly(lines, tempPoints, tempVisPoly);

  sort(tempVisPoly.begin(), tempVisPoly.end(), &mostVision);

  float coverage = 0;
  vector<Point> newPoints;
  Mat vMask[tempVisPoly.size()];

  for (int i = 0; i < tempVisPoly.size(); i++) {
	vMask[i] = Mat::zeros(vMap.size(), CV_8UC1);
	drawPoly2(vMask[i], tempVisPoly[i].second, Scalar(255));
	bitwise_and(bitmap, vMask[i], vMask[i]);
  }

  Mat jointVis;
  Mat tempVis = vMask[0].clone();
  newPoints.push_back(tempPoints[0]);
  float bitMapSum = sumC1(bitmap);
  float jsum;
  int points = 0;
  for (int i = 1; i < tempPoints.size(); i++) {
	bitwise_and(tempVis, vMask[i], jointVis);
	jsum = (sumC1(vMask[i]) - sumC1(jointVis)) / bitMapSum;
	if (jsum > 0.1) {
	  drawPoly2(tempVis, tempVisPoly[i].second, Scalar(255));
	  bitwise_and(bitmap, tempVis, tempVis);
	  pts.push_back(tempVisPoly[i].first);
	  visPoly.push_back(tempVisPoly[i]);
	  points++;
	}

	imshow("testmap", jointVis);
	imshow("testmap2", tempVis);
  }
  coverage = sumC1(tempVis) / bitMapSum;
  if (coverage < cov_thresh) {
	Moments Mo;
	vector<vector<Point>> cont;

	Mat ttempVis = tempVis.clone();

	ttempVis = 255 - ttempVis;
	bitwise_and(ttempVis, bitmap, ttempVis);
	// erode(ttempVis, ttempVis, getStructuringElement(MORPH_ELLIPSE, Size(3,
	// 3)));

	imshow("testmap", ttempVis);
	findContours(ttempVis, cont, RETR_TREE, CHAIN_APPROX_SIMPLE);
	drawContours(vMap, cont, -1, Scalar(255));

	tempPoints.clear();
	for (int i = 0; i < cont.size(); i++) {
	  Mo = moments(cont[i], false);

	  Point P((Mo.m10 / Mo.m00), (Mo.m01 / Mo.m00));
	  // vMap.at<ushort>(P) = 0xffff;
	  tempPoints.push_back(P);
	}

	vector<pair<Point, vector<Point>>> contVis;
	for (int i = 0; i < cont.size(); i++)
	  contVis.push_back(make_pair(tempPoints[i], cont[i]));
	sort(contVis.begin(), contVis.end(), &mostVision);

	tempVisPoly.clear();
	tempPoints.clear();
	for (int i = 0; i < contVis.size(); i++)
	  tempPoints.push_back(contVis[i].first);
	genVisPoly(lines, tempPoints, tempVisPoly);

	for (int i = 0; i < contVis.size(); i++) {
	  if (coverage > cov_thresh) break;
	  vector<Point> approxPoly;
	  approxPolyDP(contVis[i].second, approxPoly, 4, false);
	  if (isContourConvex(approxPoly)) {
		drawPoly2(tempVis, contVis[i].second, Scalar(255));
		bitwise_and(bitmap, tempVis, tempVis);
		coverage = sumC1(tempVis) / bitMapSum;
		pts.push_back(contVis[i].first);
		points++;
		waitKey();
		imshow("testmap2", tempVis);
	  }
	}
  }
  cout << "In generating vision Point for " + name << endl;
  cout << points + 1 << "/" << tempPoints.size() << " points where drawn";
  cout << coverage << " total coverage" << endl;

  imshow("map", vMap);
}

void MP::genVisPoly(vector<segment_type>& lines, vector<Point>& pts,
					vector<pair<Point, vector<Point>>>& visPoly) {
  int offset = visPoly.size();
  visPoly.resize(visPoly.size() + pts.size());
  for (uint i = offset; i < pts.size() + offset; i++) {
	float X = pts[i].x + 0.5;
	float Y = pts[i].y + 0.5;
	auto XY = vec2(X, Y);
	vec2 XYS = XY;
	vector<vec2> vision =
		visibility_polygon(vector_type{XYS}, lines.begin(), lines.end());

	visPoly[i].first = pts[i];
	visPoly[i].second.resize(vision.size());
	for (uint j = 0; j < vision.size(); j++) {
	  visPoly[i].second[j] = Point(vision[j].x, vision[j].y);
	}
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

void MP::localMaxima(const Mat image, vector<Point>& pts, bool remove_plateaus,
					 int kx, int ky) {
  Mat mask(image.size(), CV_16U);

  // find pixels that are equal to the local neighborhood not maximum
  // (including 'plateaus')
  Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(kx, ky));
  dilate(image, mask, kernel);

  imshow("map", mask);
  waitKey();
  waitKey();
  waitKey();
  compare(image, mask, mask, cv::CMP_GE);
  // optionally filter out pixels that are equal to the local minimum
  // ('plateaus')
  if (remove_plateaus) {
	Mat non_plateau_mask;
	erode(image, non_plateau_mask, kernel);
	compare(image, non_plateau_mask, non_plateau_mask, cv::CMP_GT);
	bitwise_and(mask, non_plateau_mask, mask);
  }

  imshow("mask", mask);

  mask.convertTo(mask, CV_8SC1);
  for (int x = 0; x < mask.cols; x++) {
	for (int y = 0; y < mask.rows; y++) {
	  int n = 0;
	  if (mask.at<uchar>(y, x) > 0) {
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

void MP::imwrite2(string s, Mat m) {
  int result;
  try {
	result = imwrite(s, m);
  } catch (const cv::Exception& ex) {
	fprintf(stderr, "Exception converting image to PNG format: %s\n",
			ex.what());
  }
  if (result)
	cout << "Succesfully saved image as" + s << endl;
  else
	cout << "Error occured when trying to save image as " + s << endl;
}

bool MP::imread2(string s, Mat& m) {
  {
	Mat result;

	result = imread(s, IMREAD_ANYDEPTH);

	if (!result.data) {
	  cout << "Loading " + s + " failed" << endl;
	  return false;
	} else {
	  cout << "Loading image " + s + " succeded." << endl;
	  result.copyTo(m);
	}

	return true;
  }
}

float MP::sumC1(Mat m) {
  int sum = 0;
  for (int x = 0; x < m.cols; x++) {
	for (int y = 0; y < m.rows; y++) {
	  if (m.at<uchar>(y, x) != 0) sum++;
	}
  }
  return (float)sum;
}

bool MP::largestArea(const Rect& a, const Rect& b) {
  return (a.area() > b.area());
}

void MP::cnrHeatC() {
  vector<pair<Point, LineIterator>> lscan;
  const int r = 5;

  for (uint c = 0; c < cnr.size(); c++) {
	corner C = cnr[c];

	int x0 = C.P.x;
	int y0 = C.P.y;
	int cx = r - 1;
	int cy = 0;
	int cdx = 1;
	int cdy = 1;
	int err = cdx - (r << 1);
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
	cout << "Corner image has " << m.channels() << "channels" << endl;
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
  for (uint i = 0; i < cnr.size(); i++) {
	if (cnr[i].type == 'i') {
	  display.at<Vec3b>(cnr[i].P) = Vec3b(0, 255, 0);
	} else {
	  display.at<Vec3b>(cnr[i].P) = Vec3b(255, 0, 0);
	}
  }
}
