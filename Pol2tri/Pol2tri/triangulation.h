#ifndef POLY2TRI_H
#define POLY2TRI_H

#define _USE_MATH_DEFINES

#include "cdt.h"
#include <exception>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327
#endif

namespace cdt
{
	const double PI_3div4 = 3 * M_PI / 4;
	const double PI_div2 = 1.57079632679489661923;
	const double EPSILON = 1e-12;
	enum Orientation { CW, CCW, COLLINEAR };

	Orientation Orient2d(const Point& pa, const Point& pb, const Point& pc)
	{
		double detleft = (pa.xyz.x() - pc.xyz.x()) * (pb.xyz.y() - pc.xyz.y());
		double detright = (pa.xyz.y() - pc.xyz.y()) * (pb.xyz.x() - pc.xyz.x());
		double val = detleft - detright;
		if (val > -EPSILON && val < EPSILON)
		{
			return COLLINEAR;
		}
		else if (val > 0)
		{
			return CCW;
		}
		return CW;
	}

	bool InScanArea(const Point& pa, const Point& pb, const Point& pc, const Point& pd)
	{
		double oadb = (pa.xyz.x() - pb.xyz.x())*(pd.xyz.y() - pb.xyz.y()) - (pd.xyz.x() - pb.xyz.x())*(pa.xyz.y() - pb.xyz.y());
		if (oadb >= -EPSILON)
		{
			return false;
		}

		double oadc = (pa.xyz.x() - pc.xyz.x())*(pd.xyz.y() - pc.xyz.y()) - (pd.xyz.x() - pc.xyz.x())*(pa.xyz.y() - pc.xyz.y());
		if (oadc <= EPSILON)
		{
			return false;
		}
		return true;
	}
}

#endif