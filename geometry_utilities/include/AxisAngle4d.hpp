
#include <eigen3/Eigen/Eigen>

namespace geometry_utilities
{

class AxisAngle4d
{
	public:
		AxisAngle4d();
		AxisAngle4d(const double &x, const double &y, const double &z, const double &angle);
		AxisAngle4d(const Eigen::Vector3d &axis, const double &angle);
		AxisAngle4d(const AxisAngle4d &axisAngle);

		double x, y, z, angle;
};

}