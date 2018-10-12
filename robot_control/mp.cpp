#include "mp.h"

mp::mp() {}

mp::mp(Mat src) {
  cornerkernel = (Mat_<uchar>(2, 2) << 1, 3, 7, 5);

  cvtColor(src, bitmap, COLOR_BGR2GRAY);
  bitmap.copyTo(bitmap_t);
  display = src;
  cornerMask = Mat::zeros(display.rows, display.cols, CV_8UC3);
  areaMask = cornerMask.clone();
  namedWindow("map", WINDOW_FREERATIO);
  resizeWindow("map", display.cols, display.rows);
}

int mp::findCorners(Mat m, vector<corner> &cnR) {

  cnR.clear();
  corner C;
  int kVal = 0;
  int mapVal = 0;
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
	  case 5:
		//		arrowedLine(display, Point(c + 1, r + 1), Point(c + aL,
		// r
		//+ aL), 					Scalar(0, 255, 0));
		C.P = Point(c + 1, r + 1);
		C.D = Point(1, 1);
		C.type = 'i';
		cnR.push_back(C);
		break;
	  case 7:
		//		arrowedLine(display, Point(c, r + 1), Point(c - aL, r +
		// aL), 					Scalar(0, 255, 0));
		C.P = Point(c, r + 1);
		C.D = Point(-1, 1);
		C.type = 'i';
		cnR.push_back(C);
		break;
	  case 3:
		//		arrowedLine(display, Point(c + 1, r), Point(c + aL, r -
		// aL), 					Scalar(0, 255, 0));
		C.P = Point(c + 1, r);
		C.D = Point(1, -1);
		C.type = 'i';
		cnR.push_back(C);
		break;
	  case 1:
		//		arrowedLine(display, Point(c, r), Point(c - aL, r - aL),
		//					Scalar(0, 255, 0));
		C.P = Point(c, r);
		C.D = Point(-1, -1);
		C.type = 'i';
		cnR.push_back(C);
		break;
		//	  case 15:
		//		//		arrowedLine(display, Point(c + 1, r +
		// 1), Point(c + aL,
		//		// r
		//		//+ aL), Scalar(0, 255,
		// 0)); 		C.P = Point(c + 1, r + 1); 		C.D =
		// Point(1,
		// 1); 		C.type = '0'; 		cnR.push_back(C);
		// break; 		break; 	  case 13:
		//		//		arrowedLine(display, Point(c, r + 1),
		// Point(c - aL, r +
		//		// aL), Scalar(0, 255,
		// 0)); 		C.P = Point(c, r + 1); 		C.D = Point(-1,
		// 1);
		// C.type = 'o'; 		cnR.push_back(C); 		break;
		// case 9:
		//		//		arrowedLine(display, Point(c + 1, r),
		// Point(c + aL, r -
		//		// aL), Scalar(0, 255,
		// 0)); 		C.P = Point(c + 1, r); 		C.D = Point(1,
		// -1); C.type = 'o'; 		cnR.push_back(C);
		// break; 	  case 11:
		//		//		arrowedLine(display, Point(c, r),
		// Point(c
		//- aL, r - aL),
		//		//					Scalar(0, 255,
		// 0)); 		C.P = Point(c, r); 		C.D = Point(-1,
		// -1); C.type = 'o'; 		cnR.push_back(C); 		break;
	  default:
		break;
	  }
	}
  }
  return cnR.size();
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

void mp::findAreas(Mat m, vector<corner> &cnR, vector<Rect> &A) {
  while (findCorners(bitmap_t, cnr_t)) {

	for (int i = 0; i < cnR.size(); i++) {

	  int dy = 1;
	  int dx = 0;
	  corner *a = &cnR[i];
	  while ((m.at<uchar>(a->P.y, a->P.x + (a->D.x * dx++))) != 0)
		;

	  while (!isBlackBetween(m, a->P + Point(0, a->D.y * dy++),
							 Point(a->D.x, 0), dx - 1))
		;

	  Point ul;
	  if (a->D.x > 0) {
		ul.x = a->P.x;
	  } else if (a->D.x < 0) {
		ul.x = a->P.x - (dx - 2);
	  }
	  if (a->D.y > 0) {
		ul.y = a->P.y;
	  } else if (a->D.y < 0) {
		ul.y = a->P.y - (dy - 2);
	  }

	  cout << i << ":  --------------------------------------" << endl;
	  cout << "dxdy: " << dx << " ," << dy << endl;
	  cout << "point a: " << a->P.x << " ," << a->P.y << endl;

	  cout << "ul: " << ul.x << " ;" << ul.y << endl;
	  A.push_back(Rect(ul, Size(dx - 1, dy - 1)));

	  //--------------------------------------------------------------------------------
	  dy = 0;
	  dx = 1;

	  while (m.at<uchar>(a->P.y + (a->D.y * dy++), a->P.x) != 0)
		;

	  while (!isBlackBetween(m, a->P + Point(a->D.x * dx++, 0),
							 Point(0, a->D.y), dy - 1))
		;

	  if (a->D.x > 0) {
		ul.x = a->P.x;
	  } else if (a->D.x < 0) {
		ul.x = a->P.x - (dx - 2);
	  }
	  if (a->D.y > 0) {
		ul.y = a->P.y;
	  } else if (a->D.y < 0) {
		ul.y = a->P.y - (dy - 2);
	  }

	  A.push_back(Rect(ul, Size(dx - 1, dy - 1)));
	}

	sort(A.begin(), A.end(), &largestArea);
	rectangle(bitmap_t, A[0], Scalar(0, 0, 0), FILLED);
	area.push_back(A[0]);
	A.clear();
  }
}

bool mp::isBlackBetween(Mat m, Point a, Point d, int l) {
  for (int i = 0; i < l; i++) {
	if (d.y == 0) {
	  if (m.at<uchar>(a.y, a.x + (d.x * i)) == 0)
		return true;
	} else {
	  if (m.at<uchar>(a.y + (d.y * i), a.x) == 0)
		return true;
	}
  }
  return false;
}

void mp::displayMap() { imshow("map", display); }

void mp::drawRect() {
  for (int i = 0; i < area.size(); i++) {
	rectangle(display, area[i],
			  Scalar(rand() % 235 + 20, rand() % 235 + 20, rand() % 235 + 20),
			  FILLED);
  }
}

bool mp::largestArea(Rect const &a, Rect const &b) {
  return (a.area() > b.area());
}
