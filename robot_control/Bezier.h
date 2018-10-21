#pragma once

#include <opencv2/core.hpp>

using namespace cv;

class Bezier
{
public:
		static std::vector<Point> curve(Point* pts, int npts);
};

