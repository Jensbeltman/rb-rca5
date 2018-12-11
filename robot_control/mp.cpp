#include "mp.h"

MP::MP(Mat bmap) {
  namedWindow("map", WINDOW_FREERATIO);
  cvtColor(bmap, bitmap, COLOR_BGR2GRAY);
  cnrheatmap = bmap;
  display = bmap;
  cornerkernel = (Mat_<uchar>(2, 2) << 1, 3, 7, 5);
  findCorners(bitmap);
  cnrHeatC();
  localizor = Localizor(bitmap.cols, bitmap.rows);
  mclocalizor = MCLocalizor(bitmap,4);

  cout << "MP created" << endl;
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
	imshow("map", cnrheatmap);
	waitKey(0);
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
	cout << cx - cy << endl;
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
		int light = (255 * nrm) / (r - 1);
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
  cvtColor(bitmap, display, COLOR_GRAY2BGR);
  //display = cnrheatmap.clone();
  display.at<Vec3b>(localizor.getY(), localizor.getX()) = Vec3b(255, 0, 0);
  imshow("map", display);


}

void MP::localPoseCallback(ConstPosesStampedPtr& _msg) {
  localizor.localPoseCallback(_msg);
  mclocalizor.localPoseCallback(_msg);
}

void MP::globalPoseCallback(ConstPosesStampedPtr& _msg) {
    localizor.globalPoseCallback(_msg);
    mclocalizor.globalPoseCallback(_msg);
}

void MP::lidarScanCallback(ConstLaserScanStampedPtr &_msg)
{
    mclocalizor.lidarScanCallback(_msg);
}

int MP::findCorners(Mat m) {
  cnr.clear();
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
		  cnr.push_back(C);
		  break;
		case 13:
		  C.P = Point(c, r + 1);
		  C.D = Point(-1, 1);
		  C.type = 'o';
		  cnr.push_back(C);
		  break;
		case 9:
		  C.P = Point(c + 1, r);
		  C.D = Point(1, -1);
		  C.type = 'o';
		  cnr.push_back(C);
		  break;
		case 11:

		  C.P = Point(c, r);
		  C.D = Point(-1, -1);
		  C.type = 'o';
		  cnr.push_back(C);
		  break;

		case 5:

		  C.P = Point(c + 1, r + 1);
		  C.D = Point(1, 1);
		  C.type = 'i';
		  cnr.push_back(C);
		  break;
		case 7:
		  C.P = Point(c, r + 1);
		  C.D = Point(-1, 1);
		  C.type = 'i';
		  cnr.push_back(C);
		  break;
		case 3:
		  C.P = Point(c + 1, r);
		  C.D = Point(1, -1);
		  C.type = 'i';
		  cnr.push_back(C);
		  break;
		case 1:
		  C.P = Point(c, r);
		  C.D = Point(-1, -1);
		  C.type = 'i';
		  cnr.push_back(C);
		  break;

		default:
		  break;
	  }
	}
  }
  return cnr.size();
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
