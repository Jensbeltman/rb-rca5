#include "mp.h"

MP::MP(Mat bmap) {
  namedWindow("map", WINDOW_FREERATIO);
  namedWindow("testmap", WINDOW_FREERATIO);
  namedWindow("testmap2", WINDOW_FREERATIO);
  namedWindow("testmap4", WINDOW_FREERATIO);
  cvtColor(bmap, bitmap, COLOR_BGR2GRAY);

  cnrheatmap = bmap;
  resize(bitmap, vMap, bitmap.size() * 4, 0, 0, INTER_NEAREST);
  display = bmap.clone();
  bitmapRect = Rect(Point(), bitmap.size());
  cornerkernel = (Mat_<uchar>(2, 2) << 1, 3, 7, 5);
  visCnr = new vector<corner*>*[bitmap.cols];
  for (int i = 0; i < bitmap.cols; i++) {
	visCnr[i] = new vector<corner*>[bitmap.rows];
  }

  findCorners(bitmap, cnr);
  findCorners(vMap, cnrV);
  cnrHeatC();
  findlines();

  genVisCnr();
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

void MP::findlines() {
  Mat lMap;

  resize(bitmap, lMap, vMap.size(), 0, 0, INTER_NEAREST);
  vector<vector<Point>> contourArr;
  //  lMap = 255 - lMap;
  findContours(lMap, contourArr, RETR_TREE, CHAIN_APPROX_SIMPLE);
  cvtColor(lMap, lMap, COLOR_GRAY2BGR);
  drawContours(lMap, contourArr, -1, Scalar(0, 255, 255));
  //  lMap = Scalar(255, 255, 255) - lMap;
  imshow("testmap", lMap);

  for (int con = 1; con < contourArr.size(); con++) {
	int conSize = contourArr[con].size();
	for (int i = 1; i < conSize; i++) {
	  mapLines.push_back(Line(
		  Point2f((float)contourArr[con][i - 1].x,
				  (float)contourArr[con][i - 1].y),
		  Point2f((float)contourArr[con][i].x, (float)contourArr[con][i].y)));
	  if (i == conSize - 1) {
		mapLines.push_back(Line(
			Point2f((float)contourArr[con][i].x, (float)contourArr[con][i].y),
			Point2f((float)contourArr[con][0].x, (float)contourArr[con][0].y)));
	  }
	}
  }

  Mat lMapCOPY;
  resize(bitmap, lMapCOPY, vMap.size(), 0, 0, INTER_NEAREST);
  cvtColor(lMapCOPY, lMapCOPY, COLOR_GRAY2BGR);
  for (int i = 0; i < mapLines.size(); i++) {
	line(lMapCOPY, mapLines[i].p1, mapLines[i].p2, Scalar(255, 0, 0));
  }
  imshow("testmap2", lMapCOPY);
}

void MP::genVisCnr() {
  int rows = bitmap.rows;
  int cols = bitmap.cols;

  for (int x = 0; x < cols; x++) {
	for (int y = 0; y < rows; y++) {
	  if (bitmap.at<uchar>(y, x) != 0) {
		//		for (int i = 0; i < cnr.size(); i++) {
		//		  LineIterator li(bitmap, Point(x, y), cnr[i].P, 4);
		//		  bool connected = true;
		//		  for (int j = 0; j < li.count; j++, li++) {
		//			if (**li == 0) {
		//			  connected = false;
		//			  break;
		//			}
		//		  }
		//		  if (connected) {
		//			visCnr[x][y].push_back(&cnrV[i]);
		//		  }
	  }
	}
  }
}

bool angleComp(Line a, Line b) { return (a.angle < b.angle); }

void MP::visionMap() {
  float area;
  float maxArea = 0;

  float r = sqrt(pow(vMap.cols, 2) * pow(vMap.rows, 2));
  for (int x = 0; x < vMap.cols; x++) {
	cout << x << endl;
	for (int y = 0; y < vMap.rows; y++) {
	  if (vMap.at<uchar>(y, x) != 0) {
		vector<Line> cnrLines;

		for (int i = 0; i < visCnr[x / 4][y / 4].size(); i++) {
		  corner* C = visCnr[x / 4][y / 4][i];
		  Line l(Point2f(x, y), Point(C->P));
		  if (C->type == 'i') {
			cnrLines.push_back(l);
		  }
		  if (C->type == 'o') {
			if (C->D == Point(1, 1)) {
			  if (x > C->P.x && y > C->P.y) cnrLines.push_back(l);
			} else if (C->D == Point(-1, 1)) {
			  if (x < C->P.x && y > C->P.y) cnrLines.push_back(l);
			} else if (C->D == Point(1, -1)) {
			  if (x > C->P.x && y > C->P.y) cnrLines.push_back(l);
			} else if (C->D == Point(-1, -1)) {
			  if (x < C->P.x && y < C->P.y) cnrLines.push_back(l);
			} else {
			  //			  l.setLenth(r);
			  l.nearestIntersection(mapLines);
			  cnrLines.push_back(l);
			}
		  }
		}
		float tArea = 0;
		sort(cnrLines.begin(), cnrLines.end(), angleComp);
		for (int i = 1; i < cnrLines.size(); i++) {
		  Line l1 = cnrLines[i - 1];
		  Line l2 = cnrLines[i];
		  Mat aM =
			  (Mat_<float>(2, 2) << (l1.p2.x - l1.p1.x), (l2.p2.x - l2.p1.x),
			   (l1.p2.y - l1.p1.y), (l2.p2.y - l2.p1.y));
		  tArea += fabs(determinant(aM) / 2.0);

		  //		for (int i = 0; i < cnrLines.size(); i++)
		  //		  line(vMap, cnrLines[i].p1, cnrLines[i].p2, 127);
		}
		Line l1 = cnrLines[cnrLines.size() - 1];
		Line l2 = cnrLines[0];
		Mat aM = (Mat_<float>(2, 2) << (l1.p2.x - l1.p1.x), (l2.p2.x - l2.p1.x),
				  (l1.p2.y - l1.p1.y), (l2.p2.y - l2.p1.y));
		tArea += fabs(determinant(aM) / 2.0);

		if (tArea > maxArea) maxArea = tArea;
		cout << tArea << endl;
		vMap.at<uchar>(y, x) = (uchar)(tArea / 280);
	  }
	}
  }
  cout << "maxarea" << maxArea;
  Mat tmp;

  imshow("testmap4", vMap);
}
void MP::cnrHeat() {
  cout << "creating cnrheatmap" << endl;
  vector<Point> pts(cnr.size());
  for (int i = 0; i < cnr.size(); i++) {
	pts[i] = cnr[i].P;
  }

  int color = 255;

  vector<Point> pts_t;
  while (pts.size() > 0 && color > 0) {
	for (int i = 0; i < pts.size(); i++) {
	  for (int dx = -1; dx < 2; dx++) {
		for (int dy = -1; dy < 2; dy++) {
		  if (dx != 0 || dy != 0) {
			Point p(pts[i].x + dx, pts[i].y + dy);
			auto pixel = cnrheatmap.at<Vec3b>(p)[0];
			if (pixel == 255) {
			  cnrheatmap.at<Vec3b>(p) = Vec3b(255 - color, 255, 255 - color);
			  pts_t.push_back(p);
			}
		  }
		}
	  }
	}
	color -= 25;
	pts = pts_t;
	pts_t.clear();
	//	imshow("map", cnrheatmap);
	//	waitKey(0);
  }
  cout << "cnrheatmap created" << endl;
  // imshow("map", cnrheatmap);
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

void MP::drawMap() {
  // cvtColor(bitmap, display, COLOR_GRAY2BGR);
  display = cnrheatmap.clone();
  display.at<Vec3b>(localizor.getY(), localizor.getX()) = Vec3b(255, 0, 0);
  imshow("map", display);
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
  for (int i = 0; i < cnr.size(); i++) {
	if (cnr[i].type == 'i') {
	  display.at<Vec3b>(cnr[i].P) = Vec3b(0, 255, 0);
	} else {
	  display.at<Vec3b>(cnr[i].P) = Vec3b(255, 0, 0);
	}
  }
}
