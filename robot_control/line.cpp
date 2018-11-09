#include "line.h"

Line::Line() {}

Line::Line(Point2f p1, Point2f p2) : p1{p1}, p2{p2} {}

bool Line::nearestIntersection(vector<Line> v) {
  Point2f nearestI;
  float shortestD = 99999;
  for (int i = 0; i < v.size(); i++) {
	Point2f p3 = v[i].p1;
	Point2f p4 = v[i].p2;
	float t = ((p1.x - p3.x) * (p3.y - p4.y) - (p1.y - p3.y) * (p3.x - p4.x)) /
			  ((p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x));
	float u = -((p1.x - p2.x) * (p1.y - p3.y) - (p1.y - p2.y) * (p1.x - p3.x)) /
			  ((p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x));
	if (0 < t && t < 1 && 0 < u && u < 1) {
	  Point2f I(p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y));
	  float l = norm(p1 - I);
	  if (l < shortestD) {
		shortestD = l;
		nearestI = I;
	  }
	}
  }

  if (shortestD == 99999999) {
	return false;
  } else {
	p2 = nearestI;
	return true;
  }
}

void Line::setLenth(float l) { p2 = p1 + l * (p1 - p2) / norm(p1 - p2); }

float Line::getAngle() { return atan2(p2.y - p1.y, p2.x - p1.x); }
