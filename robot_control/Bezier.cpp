#include "Bezier.h"

inline unsigned long Fac(int x) {
	return (x <= 1 ? 1 : x * Fac(x - 1));
}

std::vector<Point> Bezier::curve(Point * pts, int npts)
{
	int n = npts - 1;
	std::vector<Point> path;

	int* bincoef = new int[npts];
	for (int i = 0; i <= n; i++)
		bincoef[i] = (Fac(n)) / (Fac(n - i) * Fac(i));

	for (float t = 0; t <= 1; t += 0.01) {
		double  x = 0,
			    y = 0;
		for (int i = 0; i <= n; i++) {
			double c = (double)((double)bincoef[i]) * powf(1 - t, n - i) * powf(t, i);
			x += c * pts[i].x;
			y += c * pts[i].y;
		}
		path.push_back(Point(x * 4, y * 4));
	}

	return path;
}
