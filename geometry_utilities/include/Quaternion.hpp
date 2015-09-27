#ifndef __QUATERNION__HPP
#define __QUATERNION__HPP

#include <eigen3/Eigen/Eigen>
#include <tf2/LinearMath/Quaternion.h>
#include "AxisAngle.hpp"

namespace geometry_utilities
{
class Quaternion
{
	public:
		Quaternion(const tf2::Quaternion &q);
		Quaternion(const Eigen::Matrix3d matrix);
		Quaternion(const Eigen::Matrix4d matrix);
		Quaternion(const double &x, const double &y, const double &z, const double &w);
		Quaternion(const std::vector<double> &q);
		Quaternion(const Quaternion &q1);
		Quaternion();
		void conjugate(const Quaternion &q1);
		void conjugate();
		void multiply(const Quaternion &q1, const Quaternion &q2);
		void multiply(const Quaternion &q1);
		void multiplyInverse(const Quaternion &q1, const Quaternion &q2);
		void multiplyInverse(const Quaternion &q1);
		void inverse(const Quaternion &q1);
		void inverse();
		void normalize(const Quaternion &q1);
		void normalize();
		void set(const double &x, const double &y, const double &z, const double &w);
		void set(const Eigen::Matrix4d &m1);
		void set(const Eigen::Matrix3d &m1);
		void set(const AxisAngle &a);
		void get(Eigen::Matrix3d &matrix) const;
		Eigen::Matrix3d getAsMatrix3d() const;
		double getX() const;
		double getY() const;
		double getZ() const;
		double getW() const;

		friend std::ostream &operator<<( std::ostream &os, const Quaternion &q )
		{
			os << "(" << q.x << "," << q.y << "," << q.z << "," << q.w << ")";
			return os;
		}

	private:
		double x, y, z, w;
};
}

#endif