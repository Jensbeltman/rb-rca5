#include "map.h"

Map::Map(Mat *m, uchar s = 4) {
  int rows = m->rows;
  int cols = m->cols;
  map = Mat(rows * s, cols * s, CV_8UC1);

  for (int r = 0; r < rows; r++) {
    uchar *value = m->ptr(r);
    for (int c = 0; c < cols; c++) {
      uchar g = (*value++ < 128) ? 0 : 255;
      for (int j = 0; j < s; j++) {
        for (int i = 0; i < s; i++) {
          map.at<uchar>(r * s + j, c * s + i) = g;
        }
      }
    }
  }
}

void Map::show() {
  Mat m = Mat(map.rows, map.cols, CV_8UC3);
  cvtColor(map, m, COLOR_GRAY2BGR);
  arrowedLine(m, pos, pos + Point(8 * cos(dir), 8 * sin(dir)),
              Scalar(0, 0, 255), 1, LINE_AA, 0, .5);
  //Rect r(pos.x - 20, pos.y - 20, 40, 40);
  //Mat cut = m(r);
  imshow("lid", copySafe(pos,dir));
  imshow("Map", m);
}

void Map::updatePose(Point p, double d) {
  pos = p + Point(map.cols / 2, map.rows / 2);
  dir = d;
}

void Map::showLidar() {}

Mat Map::copySafe(Point p, double d)
{
    int rows = map.rows;
    int cols = map.cols;
    Mat cut(40, 40, CV_8UC1);
    for(int c = 0; c < 40; c++){
        for(int r = 0; r < 40; r++){
            int tc = c+p.x-20;
            int tr = r+p.y-20;
            if( 0 <= tr && tr < rows && 0 <= tc && tc < cols )
                cut.at<uchar>(r,c) = map.at<uchar>(tr,tc);
            else
                cut.at<uchar>(r,c) = 0;
        }
    }

    Mat r2d = getRotationMatrix2D(Point(20,20),57.2957795 * d,1);
    warpAffine(cut,cut,r2d,Size(40,40));
    return cut;
}
