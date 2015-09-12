#ifndef TUPLE_3D_HPP
#define TUPLE_3D_HPP

#include <vector>
#include <stdexcept>

namespace geometry_utilities
{
class Tuple3d
{
	public:
		Tuple3d();
		Tuple3d(const double &x, const double &y, const double &z);
		Tuple3d(const Tuple3d &tupleToCopy);
		Tuple3d(double array[3]);
		Tuple3d(std::vector<double> vector);

	private:
		double x, y, z;
};
}

#endif