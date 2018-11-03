#include "montecarlo.h"

Montecarlo::Montecarlo() {}

Montecarlo::Montecarlo(Mat m, uchar s) {
  scale = s;
  int rows = m.rows;
  int cols = m.cols;
  map = Mat(rows * scale, cols * scale, CV_8UC1);

  for (int r = 0; r < rows; r++) {
	uchar *value = m.ptr(r);
	for (int c = 0; c < cols; c++) {
	  uchar g = (*value++ < 128) ? 0 : 255;
	  for (int j = 0; j < scale; j++) {
		for (int i = 0; i < scale; i++) {
		  map.at<uchar>(r * scale + j, c * scale + i) = g;
		}
	  }
	}
  }
  tpos.x = (cols / 2.) * scale;
  tpos.y = (rows / 2.) * scale;
  tdir = 0.;
  dir = 0.;
  reDistribute(tpos, tdir);
}

void Montecarlo::parseScan(ConstLaserScanStampedPtr &msg) {
  laserScanner.parseScan(msg);
  localize(laserScanner.getScan());
}

void Montecarlo::reDistribute(Point2f p, float d) {
  std::normal_distribution<double> dist(0.0, .15);
  pos = p;
  dir = d;
  tpos = p;
  tdir = d;

  for (int i = 0; i < nEst; i++) {
    rConf[i].dir = tdir  + 0.01 * dist(generator);
    rConf[i].pos.x = tpos.x + dist(generator);
    rConf[i].pos.y = tpos.y + dist(generator);
  }
}

void Montecarlo::setConf(Point2f p, float d, bool override) {
  pos = p;
  dir = d;
  if(override){
      tpos = p;
      tdir = d;
  }
}

Point2f Montecarlo::getBestPos() {
  Point2f tp(0,0);
  int n = nEst/4;
  for(int i = 0; i < n; i++){
      tp += rConf[0].pos;
  }
  tp /= (float) n;
  return Point2f((tp.x - map.cols / 2) / 4,
                 -(tp.y - map.rows / 2) / 4);
}

double Montecarlo::getBestDir() { return rConf[0].dir; }

void Montecarlo::show() {
  Mat m = Mat(map.rows, map.cols, CV_8UC3);
  cvtColor(map, m, COLOR_GRAY2BGR);
  for (int i = 0; i < nEst; i++) {
	arrowedLine(
		m, rConf[i].pos,
		rConf[i].pos + Point2f(8 * cos(rConf[i].dir), 8 * sin(rConf[i].dir)),
		Scalar(120, 120, 255), 1, LINE_AA, 0, .5);
  }
  arrowedLine(
	  m, rConf[0].pos,
	  rConf[0].pos + Point2f(8 * cos(rConf[0].dir), 8 * sin(rConf[0].dir)),
	  Scalar(0, 255, 0), 1, LINE_AA, 0, .5);
  arrowedLine(m, pos, pos + Point2f(8 * cos(dir), 8 * sin(dir)),
			  Scalar(255, 0, 0), 1, LINE_AA, 0, .5);

  // imshow("Real Lidar", laserScanner.visualizeScan(laserScanner.getScan()));
  // imshow("Map Lidar",
  // laserScanner.visualizeScan(laserScanner.generateScan(map, pos, dir)));

  imshow("Map", m);
}

Mat *Montecarlo::getMap() { return &map; }

bool robconf_sorter(Robconf const &lhs, Robconf const &rhs) {
  return lhs.score < rhs.score;
}

float Montecarlo::calcScore(Robconf rc, LaserScan *ls) {
  LaserScan *gls = laserScanner.generateScan(map, rc.pos, rc.dir);
  float score = 0;
  for (int i = 0; i < ls->ntps; i++) {
	score += std::abs(ls->pts[i] - gls->pts[i]) / ls->ntps;
  }
  return score;
}

void Montecarlo::localize(LaserScan *ls) {
  std::default_random_engine gen;
  std::normal_distribution<double> dist(0.0, nEst * 0.3);

  std::array<Robconf, 40> newrConf;

  Point2f dpos = pos - tpos;
  float ddir = dir - tdir;
  float ddist = sqrt(dpos.x * dpos.x + dpos.y * dpos.y);

  tpos = pos;
  tdir = dir;

  for (int i = 0; i < nEst; i++) {
	int sel = std::abs(dist(gen));
	// if (sel >= nEst) sel = nEst - 1;
	float ndir = rConf[sel].dir + ddir * distribution(generator);
	newrConf[i].dir = ndir;
	newrConf[i].pos.x =
		rConf[sel].pos.x + ddist * cos(ndir) * distribution(generator);
	newrConf[i].pos.y =
		rConf[sel].pos.y + ddist * sin(ndir) * distribution(generator);
	newrConf[i].score = calcScore(newrConf[i], ls);
  }
  std::sort(newrConf.begin(), newrConf.end(), &robconf_sorter);
  for (int i = 0; i < nEst; i++) {
	rConf[i] = newrConf[i];
  }
}
