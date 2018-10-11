#include "mp.h"

mp::mp() {}

mp::mp(Mat src) {
  cornerkernel = (Mat_<uchar>(2, 2) << 1, 3, 7, 5);

  cvtColor(src, bitmap, COLOR_BGR2GRAY);
  display = src;
  cornerMask = Mat::zeros(display.rows, display.cols, CV_8UC3);
  areaMask = cornerMask.clone();
  namedWindow("map", WINDOW_FREERATIO);
  resizeWindow("map", display.cols, display.rows);
}

void mp::findCorners() {

  corner C;
  int kVal = 0;
  int mapVal = 0;
  for (int c = 0; c < bitmap.cols - 1; c++) {
	for (int r = 0; r < bitmap.rows - 1; r++) {
	  kVal = 0;

	  for (int x = 0; x < cornerkernel.cols; x++) {
		for (int y = 0; y < cornerkernel.rows; y++) {
		  kVal += cornerkernel.at<uchar>(y, x) *
				  (bitmap.at<uchar>(r + y, c + x) / 255);
		}
	  }

	  switch (kVal) {
	  case 5:
		//		arrowedLine(display, Point(c + 1, r + 1), Point(c + aL,
		// r
		//+ aL), 					Scalar(0, 255, 0));
		C.P = Point(c + 1, r + 1);
		C.D = Point(1, 1);
		C.type = 'i';
		break;
	  case 7:
		//		arrowedLine(display, Point(c, r + 1), Point(c - aL, r +
		// aL), 					Scalar(0, 255, 0));
		C.P = Point(c, r + 1);
		C.D = Point(-1, 1);
		C.type = 'i';
		break;
	  case 3:
		//		arrowedLine(display, Point(c + 1, r), Point(c + aL, r -
		// aL), 					Scalar(0, 255, 0));
		C.P = Point(c + 1, r);
		C.D = Point(1, -1);
		C.type = 'i';
		break;
	  case 1:
		//		arrowedLine(display, Point(c, r), Point(c - aL, r - aL),
		//					Scalar(0, 255, 0));
		C.P = Point(c, r);
		C.D = Point(-1, -1);
		C.type = 'i';
		break;
	  case 15:
		//		arrowedLine(display, Point(c + 1, r + 1), Point(c + aL,
		// r
		//+ aL), 					Scalar(0, 255, 0));
		C.P = Point(c + 1, r + 1);
		C.D = Point(1, 1);
		C.type = '0';
		break;
		break;
	  case 13:
		//		arrowedLine(display, Point(c, r + 1), Point(c - aL, r +
		// aL), 					Scalar(0, 255, 0));
		C.P = Point(c, r + 1);
		C.D = Point(-1, 1);
		C.type = 'o';
		break;
	  case 9:
		//		arrowedLine(display, Point(c + 1, r), Point(c + aL, r -
		// aL), 					Scalar(0, 255, 0));
		C.P = Point(c + 1, r);
		C.D = Point(1, -1);
		C.type = 'o';
		break;
	  case 11:
		//		arrowedLine(display, Point(c, r), Point(c - aL, r - aL),
		//					Scalar(0, 255, 0));
		C.P = Point(c, r);
		C.D = Point(-1, -1);
		C.type = 'o';
		break;
	  default:
		break;
	  }
	  cnr.push_back(C);
	}
  }
}

void mp::drawCorners() {
  for (int i = 0; i < cnr.size(); i++) {
	if (cnr[i].type == 'i') {
	  display.at<Vec3b>(cnr[i].P) = Vec3b(0, 255, 0);
	} else {
	  display.at<Vec3b>(cnr[i].P) = Vec3b(255, 0, 0);
	}
  }
}

void mp::findAreas() {
  for (int i = 0; i < 20; i++) {

	int dy = 1;
	int dx = 0;
	corner *a = &cnr[i];
	while ((bitmap.at<uchar>(a->P + Point(a->D.x * dx++, 0))) != 0)
	  ;

	while (true) {
	  for (int j)
	}
	/*!isBlackBetween(a->P + Point(0, a->D.y * dy++), Point(a->D.x, 0), dx))*/
	;

	Point ul;
	ul.x = min(a->P.x, a->P.x + (a->D.x * dx));
	ul.y = min(a->P.y, a->P.y + (a->D.y * dy));

	area.push_back(Rect(ul, Point(dx, dy)));
  }
}

bool mp::isBlackBetween(Point a, Point d, int l) {
  for (int i = 0; i < l; i++) {

	if (bitmap.at<uchar>(a.y + d.y, a.x + d.x) == 0)
	  return true;
  }
  return false;
}

void mp::displayMap() { imshow("map", display); }

void mp::drawRect() {
  for (int i = 0; i < area.size(); i++) {
	rectangle(display, area[i],
			  Scalar(rand() % 255, rand() % 255, rand() % 255));
  }
}
