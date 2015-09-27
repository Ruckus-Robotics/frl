#include "Point3d.hpp"
#include <math.h>
#include <algorithm>

namespace geometry_utilities
{

Point3d::Point3d(const double &x, const double &y, const double &z) : Tuple3d(x, y, z)
{

}

Point3d::Point3d(const std::vector<double> &vector) : Tuple3d(vector)
{

}

Point3d::Point3d(const Point3d &point)
{
	this->x = point.x;
	this->y = point.y;
	this->z = point.z;
}

Point3d::Point3d(const Tuple3d &tuple) : Tuple3d(tuple)
{

}

Point3d::Point3d() : Tuple3d()
{

}

double Point3d::distanceSquared(const Point3d &point)
{
	double dx, dy, dz;

	dx = this->x - point.x;
	dy = this->y - point.y;
	dz = this->z - point.z;
	return (dx * dx + dy * dy + dz * dz);
}

double Point3d::distance(const Point3d point)
{
	double dx, dy, dz;

	dx = this->x - point.x;
	dy = this->y - point.y;
	dz = this->z - point.z;
	return sqrt(dx * dx + dy * dy + dz * dz);
}

double Point3d::distanceL1(const Point3d &point)
{
	return (fabs(this->x - point.x) + fabs(this->y - point.y) +
	        fabs(this->z - point.z));
}

double Point3d::distanceLinf(const Point3d &point)
{
	double tmp;
	tmp = std::max( fabs(this->x - point.x), fabs(this->y - point.y));

	return std::max(tmp, fabs(this->z - point.z));
}

}