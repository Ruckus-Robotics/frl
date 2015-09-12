#include "Tuple3d.hpp"

namespace geometry_utilities
{
Tuple3d::Tuple3d()
{
	this->x = 0;
	this->y = 0;
	this->z = 0;
}

Tuple3d::Tuple3d(const Tuple3d &tupleToCopy)
{
	this->x = tupleToCopy.x;
	this->y = tupleToCopy.y;
	this->z = tupleToCopy.z;
}

Tuple3d::Tuple3d(const double &x, const double &y, const double &z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}

Tuple3d::Tuple3d(double array[3])
{
	this->x = array[0];
	this->y = array[1];
	this->z = array[2];
}

Tuple3d::Tuple3d(std::vector<double> vector)
{
	if (vector.size() != 3)
	{
		throw std::runtime_error("Vector size is not equal to 3!");
	}

	this->x = vector[0];
	this->y = vector[1];
	this->z = vector[2];
}

}