#ifndef POINT_3D_HPP
#define POINT_3D_HPP

#include <eigen3/Eigen/Eigen>
#include <Tuple3d.hpp>

namespace geometry_utilities
{
class Point3d : public Tuple3d
{
	public:
		Point3d();
		Point3d(const Eigen::Vector3d vector);
};
}

#endif