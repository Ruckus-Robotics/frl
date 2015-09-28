#include <math.h>
#include <vector>
#include "geometry_utilities/Quaternion.hpp"

static double EPS = 1.0e-12;
static double EPS2 = 1.0e-30;
static double PIO2 = 1.57079632679;

namespace geometry_utilities
{

Quaternion::Quaternion(const tf2::Quaternion &q)
{
	this->x = q.getAxis().getX();
	this->y = q.getAxis().getY();
	this->z = q.getAxis().getZ();
	this->w = q.getW();
}

Quaternion::Quaternion(const Eigen::Matrix3d matrix)
{
	set(matrix);
}

Quaternion::Quaternion(const Eigen::Matrix4d matrix)
{
	set(matrix);
}

Quaternion::Quaternion(const double &x, const double &y, const double &z, const double &w)
{
	double mag;
	mag = 1.0 / sqrt( x * x + y * y + z * z + w * w );
	this->x =  x * mag;
	this->y =  y * mag;
	this->z =  z * mag;
	this->w =  w * mag;

}

Quaternion::Quaternion(const std::vector<double> &q)
{
	double mag;
	mag = 1.0 / sqrt( q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3] );
	this->x =  q[0] * mag;
	this->y =  q[1] * mag;
	this->z =  q[2] * mag;
	this->w =  q[3] * mag;

}

Quaternion::Quaternion(const Quaternion &q1)
{
	this->x = q1.x;
	this->y = q1.y;
	this->z = q1.z;
	this->w = q1.w;
}

/**
 * Constructs and initializes a Quaternion to (0,0,0,0).
 */
Quaternion::Quaternion()
{
	this->x = 0.0;
	this->y = 0.0;
	this->z = 0.0;
	this->w = 1.0;
}


/**
 * Sets the value of this quaternion to the conjugate of quaternion q1.
 * @param q1 the source vector
 */
void Quaternion::conjugate(const Quaternion &q1)
{
	this->x = -q1.x;
	this->y = -q1.y;
	this->z = -q1.z;
	this->w = q1.w;
}

void Quaternion::conjugate()
{
	this->x = -this->x;
	this->y = -this->y;
	this->z = -this->z;
}

void Quaternion::multiply(const Quaternion &q1, const Quaternion &q2)
{
	this->w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
	this->x = q1.w * q2.x + q2.w * q1.x + q1.y * q2.z - q1.z * q2.y;
	this->y = q1.w * q2.y + q2.w * q1.y - q1.x * q2.z + q1.z * q2.x;
	this->z = q1.w * q2.z + q2.w * q1.z + q1.x * q2.y - q1.y * q2.x;
	// else
	// {
	//  double    x, y, w;

	//  w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
	//  x = q1.w * q2.x + q2.w * q1.x + q1.y * q2.z - q1.z * q2.y;
	//  y = q1.w * q2.y + q2.w * q1.y - q1.x * q2.z + q1.z * q2.x;
	//  this->z = q1.w * q2.z + q2.w * q1.z + q1.x * q2.y - q1.y * q2.x;
	//  this->w = w;
	//  this->x = x;
	//  this->y = y;
	// }
}

void Quaternion::multiply(const Quaternion &q1)
{
	double     x, y, w;

	w = this->w * q1.w - this->x * q1.x - this->y * q1.y - this->z * q1.z;
	x = this->w * q1.x + q1.w * this->x + this->y * q1.z - this->z * q1.y;
	y = this->w * q1.y + q1.w * this->y - this->x * q1.z + this->z * q1.x;
	this->z = this->w * q1.z + q1.w * this->z + this->x * q1.y - this->y * q1.x;
	this->w = w;
	this->x = x;
	this->y = y;
}

void Quaternion::multiplyInverse(const Quaternion &q1, const Quaternion &q2)
{
	Quaternion tempQuat = q2;

	tempQuat.inverse();
	this->multiply(q1, tempQuat);
}

void Quaternion::multiplyInverse(const Quaternion &q1)
{
	Quaternion tempQuat = q1;

	tempQuat.inverse();
	this->multiply(tempQuat);
}

void Quaternion::inverse(const Quaternion &q1)
{
	double norm;

	norm = 1.0 / (q1.w * q1.w + q1.x * q1.x + q1.y * q1.y + q1.z * q1.z);
	this->w =  norm * q1.w;
	this->x = -norm * q1.x;
	this->y = -norm * q1.y;
	this->z = -norm * q1.z;
}

void Quaternion::inverse()
{
	double norm;

	norm = 1.0 / (this->w * this->w + this->x * this->x + this->y * this->y + this->z * this->z);
	this->w *=  norm;
	this->x *= -norm;
	this->y *= -norm;
	this->z *= -norm;
}

void Quaternion::normalize(const Quaternion &q1)
{
	double norm;

	norm = (q1.x * q1.x + q1.y * q1.y + q1.z * q1.z + q1.w * q1.w);

	if (norm > 0.0)
	{
		norm = 1.0 / sqrt(norm);
		this->x = norm * q1.x;
		this->y = norm * q1.y;
		this->z = norm * q1.z;
		this->w = norm * q1.w;
	}
	else
	{
		this->x =  0.0;
		this->y =  0.0;
		this->z =  0.0;
		this->w =  0.0;
	}
}

void Quaternion::normalize()
{
	double norm;

	norm = (this->x * this->x + this->y * this->y + this->z * this->z + this->w * this->w);

	if (norm > 0.0)
	{
		norm = 1.0 / sqrt(norm);
		this->x *= norm;
		this->y *= norm;
		this->z *= norm;
		this->w *= norm;
	}
	else
	{
		this->x =  0.0;
		this->y =  0.0;
		this->z =  0.0;
		this->w =  0.0;
	}
}

void Quaternion::set(const double &x, const double &y, const double &z, const double &w)
{
	this->x = x;
	this->y = y;
	this->z = z;
	this->w = w;
}

void Quaternion::set(const Eigen::Matrix4d &m1)
{
	double ww = 0.25 * (m1(0, 0) + m1(1, 1) + m1(2, 2) + m1(3, 3));

	if (ww >= 0)
	{
		if (ww >= EPS2)
		{
			this->w = sqrt(ww);
			ww =  0.25 / this->w;
			this->x =  ((m1(2, 1) - m1(1, 2)) * ww);
			this->y =  ((m1(0, 2) - m1(2, 0)) * ww);
			this->z =  ((m1(1, 0) - m1(0, 1)) * ww);
			return;
		}
	}
	else
	{
		this->w = 0;
		this->x = 0;
		this->y = 0;
		this->z = 1;
		return;
	}

	this->w = 0;
	ww = -0.5 * (m1(1, 1) + m1(2, 2));
	if (ww >= 0)
	{
		if (ww >= EPS2)
		{
			this->x =  sqrt(ww);
			ww = 1.0 / (2.0 * this->x);
			this->y = (m1(1, 0) * ww);
			this->z = (m1(2, 0) * ww);
			return;
		}
	}
	else
	{
		this->x = 0;
		this->y = 0;
		this->z = 1;
		return;
	}

	this->x = 0;
	ww = 0.5 * (1.0 - m1(2, 2));
	if (ww >= EPS2)
	{
		this->y = sqrt(ww);
		this->z = (m1(2, 1)) / (2.0 * this->y);
		return;
	}

	this->y = 0;
	this->z = 1;
}

void Quaternion::set(const Eigen::Matrix3d &m1)
{
	double ww = 0.25 * (m1(0, 0) + m1(1, 1) + m1(2, 2) + 1.0);

	if (ww >= 0)
	{
		if (ww >= EPS2)
		{
			this->w = sqrt(ww);
			ww = 0.25 / this->w;
			this->x = (m1(2, 1) - m1(1, 2)) * ww;
			this->y = (m1(0, 2) - m1(2, 0)) * ww;
			this->z = (m1(1, 0) - m1(0, 1)) * ww;
			return;
		}
	}
	else
	{
		this->w = 0;
		this->x = 0;
		this->y = 0;
		this->z = 1;
		return;
	}

	this->w = 0;
	ww = -0.5 * (m1(1, 1) + m1(2, 2));
	if (ww >= 0)
	{
		if (ww >= EPS2)
		{
			this->x =  sqrt(ww);
			ww = 0.5 / this->x;
			this->y = m1(1, 0) * ww;
			this->z = m1(2, 0) * ww;
			return;
		}
	}
	else
	{
		this->x = 0;
		this->y = 0;
		this->z = 1;
		return;
	}

	this->x = 0;
	ww = 0.5 * (1.0 - m1(2, 2));
	if (ww >= EPS2)
	{
		this->y =  sqrt(ww);
		this->z = m1(2, 1) / (2.0 * this->y);
		return;
	}

	this->y = 0;
	this->z = 1;
}

void Quaternion::set(const AxisAngle &a)
{
	double mag, amag;

	amag = sqrt( a.x * a.x + a.y * a.y + a.z * a.z);
	if ( amag < EPS )
	{
		w = 0.0;
		x = 0.0;
		y = 0.0;
		z = 0.0;
	}
	else
	{
		amag = 1.0 / amag;
		mag = sin(a.angle / 2.0);
		w = cos(a.angle / 2.0);
		x = a.x * amag * mag;
		y = a.y * amag * mag;
		z = a.z * amag * mag;
	}

}

Eigen::Matrix3d Quaternion::getAsMatrix3d() const
{
	Eigen::Matrix3d matrix;
	get(matrix);
	return matrix;
}

void Quaternion::get(Eigen::Matrix3d &matrix) const
{
	double qx = this->x;
	double qy = this->y;
	double qz = this->z;
	double qw = this->w;

	double yy2 = 2.0 * qy * qy;
	double zz2 = 2.0 * qz * qz;
	double xx2 = 2.0 * qx * qx;
	double xy2 = 2.0 * qx * qy;
	double wz2 = 2.0 * qw * qz;
	double xz2 = 2.0 * qx * qz;
	double wy2 = 2.0 * qw * qy;
	double yz2 = 2.0 * qy * qz;
	double wx2 = 2.0 * qw * qx;

	matrix(0, 0) = (1.0 - yy2 - zz2);
	matrix(0, 1) = (xy2 - wz2);
	matrix(0, 2) = (xz2 + wy2);
	matrix(1, 0) = (xy2 + wz2);
	matrix(1, 1) = (1.0 - xx2 - zz2);
	matrix(1, 2) = (yz2 - wx2);
	matrix(2, 0) = (xz2 - wy2);
	matrix(2, 1) = (yz2 + wx2);
	matrix(2, 2) = (1.0 - xx2 - yy2);

}

double Quaternion::getX() const
{
	return this->x;
}
double Quaternion::getY() const
{
	return this->y;
}
double Quaternion::getZ() const
{
	return this->z;
}
double Quaternion::getW() const
{
	return this->w;
}
}