#include "mp.h"

MP::MP(Mat bmap) {
  namedWindow("map", WINDOW_FREERATIO);
  namedWindow("testmap", WINDOW_FREERATIO);
  namedWindow("testmap2", WINDOW_FREERATIO);
  cvtColor(bmap, bitmap, COLOR_BGR2GRAY);
  cnrheatmap = bmap;
  resize(bitmap, vMap, bitmap.size() * 4, 0, 0, INTER_NEAREST);
  display = bmap.clone();
  bitmapRect = Rect(Point(), bitmap.size());
  cornerkernel = (Mat_<uchar>(2, 2) << 1, 3, 7, 5);
  findCorners(bitmap, cnr);
  findCorners(vMap, cnrV);
  // cnrHeatC();
  findlines();
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
  lMap = 255 - lMap;
  findContours(lMap, contourArr, RETR_TREE, CHAIN_APPROX_SIMPLE);
  cvtColor(lMap, lMap, COLOR_GRAY2BGR);
  drawContours(lMap, contourArr, -1, Scalar(0, 255, 255));
  lMap = Scalar(255, 255, 255) - lMap;
  //  imshow("testmap", lMap);

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

void MP::visionMap() {
  float r = sqrt(pow(vMap.cols, 2) * pow(vMap.rows, 2));
  for (int x = 0; x < vMap.cols; x++) {
	cout << x << endl;
	for (int y = 0; y < vMap.rows; y++) {
	  if (vMap.at<uchar>(y, x) != 0) {
		vector<Line> cnrLines;

		for (int i = 0; i < cnrV.size(); i++) {
		  corner C = cnrV[i];
		  if (C.type == 'o') {
			Line l(Point2f(x, y), Point(C.P));
			// l.setLenth(r);
			cnrLines.push_back(l);

		  } else
			cnrLines.push_back(Line(Point2f(x, y), Point(C.P)));
		}

		for (int i = 0; i < cnrLines.size(); i++) {
		  cnrLines[i].nearestIntersection(mapLines);
		}

		//		for (int i = 0; i < cnrLines.size(); i++)
		//		  line(vMap, cnrLines[i].p1, cnrLines[i].p2, 127);
		// imshow("testmap", vMap);
	  }
	}
  }
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

// void MP::visionMap() {
//  int r = ceil(sqrt(pow(vMap.rows, 2) + pow(vMap.cols, 2))) / 4;

//  vector<pair<Point2i, LineIterator>> cnrVision;

//  cout << "1" << endl;
//  char seenPix[vMap.cols][vMap.rows] = {0};
//  int visebility[vMap.cols][vMap.rows] = {0};
//  int mx = 0;
//  for (uint x = 0; x < vMap.cols; x++) {
//	cout << x << endl;
//	for (uint y = 0; y < vMap.rows; y++) {
//	  Point P = Point(x, y);

//	  int x0 = x;  // cnr[c].P.20;  //x;
//	  int y0 = y;  // cnr[c].P.y;
//	  int cx = r - 1;
//	  int cy = 0;
//	  int cdx = 1;
//	  int cdy = 1;
//	  int err = cdx - (r << 1);
//	  // cout << cx - cy << endl;
//	  int ii = 0;
//	  while (cx >= cy) {
//		// ii++;
//		cnrVision.push_back(
//			make_pair(P, LineIterator(vMap, P, Point(x0 + cy, y0 +
// cx),
// 8))); 		cnrVision.push_back( 			make_pair(P,
// LineIterator(vMap, P, Point(x0 + cx,
// y0 + cy), 8))); 		cnrVision.push_back(
// make_pair(P, LineIterator(vMap, P, Point(x0 + cy, y0 - cx), 8)));
// cnrVision.push_back( 			make_pair(P, LineIterator(vMap,
// P, Point(x0 + cx, y0 - cy), 8))); 		cnrVision.push_back(
//			make_pair(P, LineIterator(vMap, P, Point(x0 - cy, y0 +
// cx),
// 8))); 		cnrVision.push_back( 			make_pair(P,
// LineIterator(vMap, P, Point(x0 - cx,
// y0 + cy), 8))); 		cnrVision.push_back(
// make_pair(P, LineIterator(vMap, P, Point(x0 - cy, y0 - cx), 8)));
// cnrVision.push_back( 			make_pair(P, LineIterator(vMap,
// P, Point(x0 - cx, y0 - cy), 8)));

//		if (err <= 0) {
//		  cy++;
//		  err += cdy;
//		  cdy += 2;
//		}

//		if (err > 0) {
//		  cx--;
//		  cdx += 2;
//		  err += cdx - (r << 1);
//		}
//	  }
//	  // cout << ii << endl;
//	  for (int cV = 0; cV < cnrVision.size(); cV++) {
//		for (auto ittr = cnrVision.begin(); ittr != cnrVision.end();
//++ittr) { 		  Point2i Pix; 		  int i = 0; while (i <
// ittr->second.count) { 			Pix = ittr->second.pos();
// uchar* pix = &**ittr->second++; 			if (pix[0] == 0) {
// i =
// 999999; 			} else {
// seenPix[Pix.x][Pix.y] = 1; i++;
//			}
//		  }
//		}

//		int vis = 0;
//		for (int i = 0; i < vMap.cols; i++) {
//		  for (int j = 0; j < vMap.rows; j++) {
//			if (seenPix[i][j] == 1) {
//			  vis++;
//			  seenPix[i][j] = 0;
//			}
//		  }
//		}
//		// cout << setw(6) << vis << endl;
//		mx = max(mx, vis);
//		visebility[x][y] = vis;

//		cnrVision.clear();
//	  }
//	}
//  }
//  float s = (float)255 / (float)mx;
//  // << mx << ", " << s << endl;

//  for (uint x = 0; x < vMap.cols; x++) {
//	for (uint y = 0; y < vMap.rows; y++) {
//	  vMap.at<Vec3b>(y, x)[0] = visebility[x][y] * s;
//	  vMap.at<Vec3b>(y, x)[1] = visebility[x][y] * s;
//	  vMap.at<Vec3b>(y, x)[2] = visebility[x][y] * s;
//	}
//  }

//  imshow("testmap", vMap);
//}

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
		  CNRS.push_back(C);
		  break;
		case 13:
		  C.P = Point(c, r + 1);
		  C.D = Point(-1, 1);
		  C.type = 'o';
		  CNRS.push_back(C);
		  break;
		case 9:
		  C.P = Point(c + 1, r);
		  C.D = Point(1, -1);
		  C.type = 'o';
		  CNRS.push_back(C);
		  break;
		case 11:

		  C.P = Point(c, r);
		  C.D = Point(-1, -1);
		  C.type = 'o';
		  CNRS.push_back(C);
		  break;

		case 5:

		  C.P = Point(c + 1, r + 1);
		  C.D = Point(1, 1);
		  C.type = 'i';
		  CNRS.push_back(C);
		  break;
		case 7:
		  C.P = Point(c, r + 1);
		  C.D = Point(-1, 1);
		  C.type = 'i';
		  CNRS.push_back(C);
		  break;
		case 3:
		  C.P = Point(c + 1, r);
		  C.D = Point(1, -1);
		  C.type = 'i';
		  CNRS.push_back(C);
		  break;
		case 1:
		  C.P = Point(c, r);
		  C.D = Point(-1, -1);
		  C.type = 'i';
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
