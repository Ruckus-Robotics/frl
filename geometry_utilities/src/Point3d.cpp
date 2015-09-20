#include "Point3d.hpp"

Point3d::Point3d() : Tuple3d()
{

}

Point3d::Point3d(const Eigen::Vector &vector) : Tuple3d(vector)
{

}