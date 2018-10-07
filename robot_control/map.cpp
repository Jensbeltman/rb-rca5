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
  lidarMask = Mat(120,120,CV_8UC1, Scalar(0));
  circle(lidarMask,Point(60,60),60,Scalar(255),-1);
  const Point* pts = new const Point[3] {
    Point(60+100*cos(-2.268), 60+100*sin(-2.268)),
    Point(60+100*cos( 2.268), 60+100*sin( 2.268)),
    Point(60,60)
  };
  const int ntps = 3;
  fillPoly(lidarMask,&pts,&ntps,1,Scalar(0));

}

void Map::show() {
  Mat m = Mat(map.rows, map.cols, CV_8UC3);
  cvtColor(map, m, COLOR_GRAY2BGR);
  arrowedLine(m, pos, pos + Point2f(8 * cos(dir), 8 * sin(dir)),
              Scalar(0, 0, 255), 1, LINE_AA, 0, .5);

  //imshow("lid", copySafe(pos,dir));
  imshow("Map", m);
}

void Map::updatePose(Point2f p, double d) {
  pos = p + Point2f(map.cols / 2., map.rows / 2.);
  dir = d;
}

void Map::showLidar() {
    float scale = (72. / 25.4) * 2;
    float angle_min = -2.2688899;
    float angle_inc = 0.00710139284;
    float r_max = 10 * scale;
    Point2f p = pos + 0.8 * Point2f(cos(dir), sin(dir));

    cv::Mat im(r_max , 640, CV_8UC1);
    im.setTo(255);
    for(int i = 0; i < 640; i++){
        float range = ray(pos, r_max , dir + angle_min + angle_inc * i);
        cv::Point2f startpt(639 - i,r_max);
        cv::Point2f endpt(639 - i,range);
        cv::line(im, startpt, endpt, cv::Scalar(0), 1,
                 cv::LINE_4, 0);
    }
    imshow("maplidar",im);

}

float Map::ray(Point2f p, float r, float angle){
    Point2f delta(cos(angle),sin(angle));
    for(float i = 0; i < r; i+=.5){
        if(map.at<uchar>(p+i*delta) == 0)
            return i;
    }
    return r;
}

Mat Map::copySafe(Point p, double d)
{
    int rows = map.rows;
    int cols = map.cols;
    Mat cut(120, 120, CV_8UC1);
    for(int c = 0; c < 120; c++){
        for(int r = 0; r < 120; r++){
            int tc = c+p.x-60;
            int tr = r+p.y-60;
            if( 0 <= tr && tr < rows && 0 <= tc && tc < cols )
                cut.at<uchar>(r,c) = map.at<uchar>(tr,tc);
            else
                cut.at<uchar>(r,c) = 0;
        }
    }

    Mat r2d = getRotationMatrix2D(Point(60,60),57.2957795 * d,1);
    warpAffine(cut,cut,r2d,Size(120,120));
    Mat masked(120, 120, CV_8UC1, Scalar(0));
    cut.copyTo(masked,lidarMask);
    //Mat masked
    return masked;
}
