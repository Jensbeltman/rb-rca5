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

  //std::default_random_engine gen;
  std::normal_distribution<double> dist(0.0, .01);

  tpos.x = cols/2. * s;
  tpos.y = rows/2. * s;
  tdir   = 0.;

  for(int i = 0; i < nEst; i++){
    rConf[i].dir = tdir;// + 0.005 * dist(generator);
    rConf[i].pos.x = tpos.x;// + dist(generator);
    rConf[i].pos.y = tpos.y;// + dist(generator);
  }

}

void Map::parseScan(ConstLaserScanStampedPtr &msg)
{
    laserScanner.parseScan(msg);
    localize(laserScanner.getScan());
}

void Map::show() {
  Mat m = Mat(map.rows, map.cols, CV_8UC3);
  cvtColor(map, m, COLOR_GRAY2BGR);
  for(int i = 0; i < nEst; i++){
      arrowedLine(m, rConf[i].pos, rConf[i].pos + Point2f(8 * cos(rConf[i].dir), 8 * sin(rConf[i].dir)), Scalar(100, 100, 200), 1, LINE_AA, 0, .5);
  }
  arrowedLine(m, rConf[0].pos, rConf[0].pos + Point2f(8 * cos(rConf[0].dir), 8 * sin(rConf[0].dir)), Scalar(0, 255, 0), 1, LINE_AA, 0, .5);
  arrowedLine(m, pos, pos + Point2f(8 * cos(dir), 8 * sin(dir)), Scalar(0, 0, 255), 1, LINE_AA, 0, .5);

  imshow("Real Lidar", laserScanner.visualizeScan(laserScanner.getScan()));
  imshow("Map Lidar", laserScanner.visualizeScan(laserScanner.generateScan(&map,pos,dir)));

  imshow("Map", m);
}

void Map::updatePose(ConstPosesStampedPtr &_msg) {
    float scale = (72. / 25.4) * 2;
    for (int i = 0; i < _msg->pose_size(); i++) {
      if (_msg->pose(i).name() == "pioneer2dx") {
        pos.x = _msg->pose(i).position().x() * scale + map.cols / 2.;
        pos.y = -_msg->pose(i).position().y() * scale + map.rows / 2.;
        dir = -3.14 + 2 * atan2(_msg->pose(i).orientation().w(),_msg->pose(i).orientation().z());
        break;
      }
    }
}

Mat* Map::getMap(){ return &map; }

bool robconf_sorter(Robconf const& lhs, Robconf const& rhs) {
   return lhs.score < rhs.score;
}

float Map::calcScore(Robconf rc, LaserScan *ls){

    LaserScan* gls = laserScanner.generateScan(&map, rc.pos, rc.dir);
    float score = 0;
    for(int i = 0; i < ls->ntps; i++){
        score += std::abs(ls->pts[i] - gls->pts[i])/ls->ntps;
    }
    return score;
}

void Map::localize(LaserScan *ls)
{
    std::default_random_engine gen;
    std::normal_distribution<double> dist(0.0, nEst * 0.3);

    std::array<Robconf, 80> newrConf;

    Point2f dpos = pos - tpos;
    float   ddir = dir - tdir;
    float  ddist = sqrt(dpos.x*dpos.x + dpos.y*dpos.y);

    tpos = pos;
    tdir = dir;


    for(int i = 0; i < nEst; i++){
        int sel = std::abs(dist(gen));
        if( sel >= nEst) sel = nEst - 1;
        float ndir = rConf[sel].dir + ddir * distribution(generator);
        newrConf[i].dir = ndir;
        newrConf[i].pos.x = rConf[sel].pos.x + ddist * cos( ndir ) * distribution(generator);
        newrConf[i].pos.y = rConf[sel].pos.y + ddist * sin( ndir ) * distribution(generator);
        newrConf[i].score = calcScore(newrConf[i], ls);
    }
    std::sort(newrConf.begin(), newrConf.end(), &robconf_sorter );
    for(int i = 0; i < nEst; i++){
        rConf[i] = newrConf[i];
    }
}
