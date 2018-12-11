#include "laserscanner.h"

LaserScanner::LaserScanner() {}

void LaserScanner::parseScan(ConstLaserScanStampedPtr& msg) {
  if (!param_set) {
	_angle_min = float(msg->scan().angle_min());
	_angle_increment = float(msg->scan().angle_step());

	_range_min = float(msg->scan().range_min());
	_range_max = float(msg->scan().range_max());

	_nranges = msg->scan().ranges_size();
	param_set = true;
  }

  float* rays = new float[_nranges];

  for (int i = 0; i < _nranges; i++)
	rays[i] = std::min(float(msg->scan().ranges(i)), _range_max);

  if (_current_scan != nullptr) {
	delete _current_scan;
  }
  _current_scan = new LaserScan(rays, _nranges);
}

LaserScan* LaserScanner::getScan() { return _current_scan; }

LaserScan* LaserScanner::generateScan(Mat& map, Point2f pos, float dir) {
  Point2f p = pos + 0.8 * Point2f(cos(dir), sin(dir));

  float* rays = new float[_nranges];

  for (int i = 0; i < _nranges; i++) {
	rays[_nranges - i - 1] =
		ray(map, p, dir + _angle_min + _angle_increment * i);
  }
  return new LaserScan(rays, _nranges);
}

float LaserScanner::ray(Mat& map, Point2f p, float angle) {
  Point2f delta(cos(angle), sin(angle));
  float scale = (72. / 25.4) * 2;
  for (float r = 0; r < _range_max * scale; r += 2) {
	if (map.at<uchar>(p + r * delta) == 0) return r / scale;
  }
  return _range_max;
}

cv::Mat LaserScanner::visualizeScan(LaserScan* ls) {
  // TODO: remake visualization of scanner
  int width = 200;
  int height = 200;
  float px_per_m = 100 / _range_max;

  cv::Mat im(height, width, CV_8UC3);
  im.setTo(0);
  for (int i = 0; i < _nranges; i++) {
	float angle = _angle_min + i * _angle_increment;
	float range = std::min(float(ls->pts[i]), _range_max);
	cv::Point2f startpt(100.5f + _range_min * px_per_m * std::cos(angle),
						100.5f - _range_min * px_per_m * std::sin(angle));
	cv::Point2f endpt(100.5f + range * px_per_m * std::cos(angle),
					  100.5f - range * px_per_m * std::sin(angle));
	cv::line(im, startpt * 16, endpt * 16, cv::Scalar(255, 255, 255, 255), 1,
             cv::LINE_AA, 4);
  }
  cv::circle(im, cv::Point(100, 100), 2, cv::Scalar(0, 0, 255));

  return im;
}
