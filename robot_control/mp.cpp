#include "mp.h"

mp::mp() {}

mp::mp(Mat src) {
  cornerkernel = (Mat_<uchar>(2, 2) << 1, 3, 7, 5);

  cvtColor(src, bitmap, COLOR_BGR2GRAY);
  bitmap.copyTo(bitmap_t);
  resize(src,display,src.size()*4,0,0,INTER_NEAREST);
  cornerMask = Mat::zeros(display.rows, display.cols, CV_8UC3);
  areaMask = cornerMask.clone();
  namedWindow("map", WINDOW_FREERATIO);
  resizeWindow("map", display.cols, display.rows);

  findAreas();
}

int mp::findCorners() {

  cnr_t.clear();
  corner C;
  int kVal = 0;
  int mapVal = 0;
  for (int r = 0; r < bitmap_t.rows - 1; r++) {
    for (int c = 0; c < bitmap_t.cols - 1; c++) {
      kVal = 0;

      for (int x = 0; x < cornerkernel.cols; x++) {
        for (int y = 0; y < cornerkernel.rows; y++) {
          kVal +=
              cornerkernel.at<uchar>(y, x) * (bitmap_t.at<uchar>(r + y, c + x) / 255);
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
        cnr_t.push_back(C);
        break;
      case 7:
        //		arrowedLine(display, Point(c, r + 1), Point(c - aL, r +
        // aL), 					Scalar(0, 255, 0));
        C.P = Point(c, r + 1);
        C.D = Point(-1, 1);
        C.type = 'i';
        cnr_t.push_back(C);
        break;
      case 3:
        //		arrowedLine(display, Point(c + 1, r), Point(c + aL, r -
        // aL), 					Scalar(0, 255, 0));
        C.P = Point(c + 1, r);
        C.D = Point(1, -1);
        C.type = 'i';
        cnr_t.push_back(C);
        break;
      case 1:
        //		arrowedLine(display, Point(c, r), Point(c - aL, r - aL),
        //					Scalar(0, 255, 0));
        C.P = Point(c, r);
        C.D = Point(-1, -1);
        C.type = 'i';
        cnr_t.push_back(C);
        break;

      default:
        break;
      }
    }
  }
  return cnr_t.size();
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
  while (findCorners()) {
    area_t.clear();

    for (int i = 0; i < cnr_t.size(); i++) {

      int dy = 1;
      int dx = 0;
      corner *a = &cnr_t[i];
      while ((bitmap_t.at<uchar>(a->P.y, a->P.x + (a->D.x * dx++))) != 0)
        ;

      while (!isBlackBetween(bitmap_t, a->P + Point(0, a->D.y * dy++),
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

      /*cout << i << ":  --------------------------------------" << endl;
      cout << "dxdy: " << dx << " ," << dy << endl;
      cout << "point a: " << a->P.x << " ," << a->P.y << endl;

      cout << "ul: " << ul.x << " ;" << ul.y << endl;*/
      area_t.push_back(Rect(ul, Size(dx - 1, dy - 1)));

      //--------------------------------------------------------------------------------
      dy = 0;
      dx = 1;

      while (bitmap_t.at<uchar>(a->P.y + (a->D.y * dy++), a->P.x) != 0)
        ;

      while (!isBlackBetween(bitmap_t, a->P + Point(a->D.x * dx++, 0),
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
      area_t.push_back(Rect(ul , Size(dx - 1, dy - 1)));
    }

    sort(area_t.begin(), area_t.end(), &largestArea);
    rectangle(bitmap_t, area_t[0], Scalar(0, 0, 0), FILLED);
    area.push_back(Rect(Point(area_t[0].x, area_t[0].y)*4, area_t[0].size()*4 ));

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
              FILLED, LINE_8);
    //imshow("map", display);
    //waitKey(0);
  }
}

void mp::findConnections(){

    for(int i = 0; i < area.size() -1; i++){
        Rect a = area[i];
        for(int j = i + 1; j < area.size(); j++){
            Rect b = area[j];
            if(     a.x + a.width  == b.x && a.y + a.height > b.y && b.y + b.height > a.y ||
                    b.x + b.width  == a.x && a.y + a.height > b.y && b.y + b.height > a.y ||
                    a.y + a.height == b.y && a.x + a.width  > b.x && b.x + b.width  > a.x ||
                    b.y + b.height == a.y && a.x + a.width  > b.x && b.x + b.width  > a.x
                    ){

                line(display,Point(a.x + a.width/2, a.y + a.height/2),Point(b.x + b.width/2, b.y + b.height/2),Scalar(255,0,0),1,LINE_AA);
            }
        }
    }
}


bool mp::largestArea(Rect const &a, Rect const &b) {
  return (a.area() > b.area());
}
