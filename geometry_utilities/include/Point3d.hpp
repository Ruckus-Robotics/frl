#ifndef POINT_3D_HPP
#define POINT_3D_HPP

#include <eigen3/Eigen/Eigen>
#include <Tuple3d.hpp>
#include <vector>

namespace geometry_utilities
{
class Point3d : public Tuple3d
{
	public:
		Point3d(const double &x, const double &y, const double &z);
		Point3d(const std::vector<double> &vector);
		Point3d(const Point3d & point);
		Point3d(const Tuple3d & tuple);
		Point3d();

		double distanceSquared(const Point3d &point);
		double distance(const Point3d point);
		double distanceL1(const Point3d &point);
		double distanceLinf(const Point3d &point);
};
}

#endif