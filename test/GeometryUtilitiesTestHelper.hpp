#ifndef __GEOMETRY_UTILITIES_TEST_HELPER_HPP__
#define __GEOMETRY_UTILITIES_TEST_HELPER_HPP__

#include <random>
#include <memory>
#include <math.h>
#include <eigen3/Eigen/Eigen>
#include <unistd.h>

namespace frl
{

	namespace geometry
	{

		class GeometryUtilitiesTestHelper
		{
		public:
			static double getRandomDouble()
			{
				// Generates random double between -1000 & 1000
				return (2000.0 * rand() / RAND_MAX - 1000.0);
			}

			static double getRandomDoubleBetween1AndMinus1()
			{
				return 2.0 * rand() / RAND_MAX - 1.0;
			}

			static double getRandomAngle()
			{
				return 2 * (M_PI - 0.01) * rand() / RAND_MAX - (M_PI - 0.01);
			}

			static std::vector<double> getRandom3dVector()
			{
				std::vector<double> vector(3);
				vector[0] = getRandomDouble();
				vector[1] = getRandomDouble();
				vector[2] = getRandomDouble();

				return vector;
			}

			template<typename T>
			static Point3d<T> getRandomPoint3d()
			{
				Point3d<T> point;
				point.setX(getRandomDouble());
				point.setY(getRandomDouble());
				point.setZ(getRandomDouble());

				return point;
			}

			static Eigen::Matrix3d createRandomRotationMatrix()
			{
				Eigen::Matrix3d rotX = createRandomRotationMatrixX();
				Eigen::Matrix3d rotY = createRandomRotationMatrixY();
				Eigen::Matrix3d rotZ = createRandomRotationMatrixZ();

				return rotX * rotY * rotZ;
			}

			static Eigen::Matrix3d createRandomRotationMatrixX()
			{
				Eigen::Matrix3d rotX;
				rotX(0, 0) = 1;
				rotX(0, 1) = 0;
				rotX(0, 2) = 0;
				rotX(1, 0) = 0;
				rotX(2, 0) = 0;

				double angle = getRandomAngle();
				rotX(1, 1) = cos(angle);
				rotX(2, 2) = cos(angle);
				rotX(1, 2) = -sin(angle);
				rotX(2, 1) = sin(angle);

				return rotX;
			}

			static Eigen::Matrix3d createRandomRotationMatrixY()
			{
				Eigen::Matrix3d rotY;
				rotY(2, 1) = 0;
				rotY(0, 1) = 0;
				rotY(1, 0) = 0;
				rotY(1, 1) = 1;
				rotY(1, 2) = 0;

				double angle = getRandomAngle();
				rotY(0, 0) = cos(angle);
				rotY(2, 2) = cos(angle);
				rotY(2, 0) = -sin(angle);
				rotY(0, 2) = sin(angle);

				return rotY;
			}

			static Eigen::Matrix3d createRandomRotationMatrixZ()
			{
				Eigen::Matrix3d rotZ;
				rotZ(0, 2) = 0;
				rotZ(1, 2) = 0;
				rotZ(2, 0) = 0;
				rotZ(2, 1) = 0;
				rotZ(2, 2) = 1;

				double angle = getRandomAngle();
				rotZ(0, 0) = cos(angle);
				rotZ(1, 1) = cos(angle);
				rotZ(0, 1) = -sin(angle);
				rotZ(1, 0) = sin(angle);

				return rotZ;
			}

			static Eigen::Matrix4d createRandomTransformationMatrix()
			{
				Eigen::Matrix3d rotationMatrix = createRandomRotationMatrix();

				Eigen::Matrix4d transform;

				transform(0, 0) = rotationMatrix(0, 0);
				transform(0, 1) = rotationMatrix(0, 1);
				transform(0, 2) = rotationMatrix(0, 2);
				transform(1, 0) = rotationMatrix(1, 0);
				transform(1, 1) = rotationMatrix(1, 1);
				transform(1, 2) = rotationMatrix(1, 2);
				transform(2, 0) = rotationMatrix(2, 0);
				transform(2, 1) = rotationMatrix(2, 1);
				transform(2, 2) = rotationMatrix(2, 2);

				transform(0, 3) = getRandomDouble();
				transform(1, 3) = getRandomDouble();
				transform(2, 3) = getRandomDouble();

				transform(3, 0) = 0.0;
				transform(3, 1) = 0.0;
				transform(3, 2) = 0.0;
				transform(3, 3) = 1.0;

				return transform;
			}

			static Eigen::Matrix4d createRandomMatrix4d()
			{
				Eigen::Matrix4d matrix;

				matrix(0, 0) = getRandomDoubleBetween1AndMinus1();
				matrix(0, 1) = getRandomDoubleBetween1AndMinus1();
				matrix(0, 2) = getRandomDoubleBetween1AndMinus1();
				matrix(0, 3) = getRandomDoubleBetween1AndMinus1();
				matrix(1, 0) = getRandomDoubleBetween1AndMinus1();
				matrix(1, 1) = getRandomDoubleBetween1AndMinus1();
				matrix(1, 2) = getRandomDoubleBetween1AndMinus1();
				matrix(1, 3) = getRandomDoubleBetween1AndMinus1();
				matrix(2, 0) = getRandomDoubleBetween1AndMinus1();
				matrix(2, 1) = getRandomDoubleBetween1AndMinus1();
				matrix(2, 2) = getRandomDoubleBetween1AndMinus1();
				matrix(2, 3) = getRandomDoubleBetween1AndMinus1();
				matrix(3, 0) = 0;
				matrix(3, 1) = 0;
				matrix(3, 2) = 0;
				matrix(3, 3) = 1;

				return matrix;
			}

			static bool checkOrthogonality(Eigen::Matrix4d matrix)
			{
				bool xMag = (1.0 - (sqrt(pow(matrix(0, 0), 2) + pow(matrix(1, 0), 2) + pow(matrix(2, 0), 2))) < 1e-8);
				bool yMag = (1.0 - (sqrt(pow(matrix(0, 1), 2) + pow(matrix(1, 1), 2) + pow(matrix(2, 1), 2))) < 1e-8);
				bool zMag = (1.0 - (sqrt(pow(matrix(0, 2), 2) + pow(matrix(1, 2), 2) + pow(matrix(2, 2), 2))) < 1e-8);

				return (xMag && yMag && zMag);
			}

			static Eigen::AngleAxis<double> createRandomAxisAngle()
			{
				double x, y, z;
				x = getRandomDouble();
				y = getRandomDouble();
				z = getRandomDouble();

				double mag = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));

				x *= 1 / mag;
				y *= 1 / mag;
				z *= 1 / mag;

				double angle = getRandomAngle();

				Eigen::AngleAxis<double> ret;
				ret.axis()[0] = x;
				ret.axis()[1] = y;
				ret.axis()[2] = z;
				ret.angle() = angle;
				return ret;
			}

			static Eigen::Quaternion<double> createRandomQuaternion()
			{
				Eigen::Quaternion<double> quaternion(getRandomDouble(), getRandomDouble(), getRandomDouble(), getRandomDouble());

				quaternion.normalize();

				return quaternion;
			}

			static bool areAxisAngleEpsilonEqual(const Eigen::AngleAxis<double> &a1, const Eigen::AngleAxis<double> &a2, const double &eps)
			{
				if ((fabs(a1.axis()[0] - a2.axis()[0]) < eps && fabs(a1.axis()[1] - a2.axis()[1]) < eps && fabs(a1.axis()[2] - a2.axis()[2]) < eps && fabs(a1.angle() - a2.angle()) < eps) ||
					(fabs(-a1.axis()[0] - a2.axis()[0]) < eps && fabs(-a1.axis()[1] - a2.axis()[1]) < eps && fabs(-a1.axis()[2] - a2.axis()[2]) < eps && fabs(-a1.angle() - a2.angle()) < eps))
				{
					return true;
				}

				if ((fabs(a1.axis()[0] - a2.axis()[0]) < eps && fabs(a1.axis()[1] - a2.axis()[1]) < eps && fabs(a1.axis()[2] - a2.axis()[2]) < eps))
				{
					if (M_PI - fabs(a1.angle()) < 1e-4 && M_PI - fabs(a2.angle()) < 1e-4)
					{
						return true;
					}
					else
					{
						return false;
					}
				}
			}

			static bool areQuaternionsEpsilonEqual(const Eigen::Quaternion<double> &q1, const Eigen::Quaternion<double> &q2, const double &eps)
			{
				if ((fabs(q1.x() - q2.x()) < eps && fabs(q1.y() - q2.y()) < eps && fabs(q1.z() - q2.z()) < eps && fabs(q1.w() - q2.w()) < eps) ||
					(fabs(-q1.x() - q2.x()) < eps && fabs(-q1.y() - q2.y()) < eps && fabs(-q1.z() - q2.z()) < eps && fabs(-q1.w() - q2.w()) < eps))
				{
					return true;
				}

				return false;
			}

			static bool areVector3dsEpsilonEqual(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, const double &eps)
			{
				return (fabs(v1(0) - v2(0)) < eps && fabs(v1(1) - v2(1)) < eps && fabs(v1(2) - v2(2)) < eps);
			}

			static bool areVector4dsEpsilonEqual(const Eigen::Vector4d &v1, const Eigen::Vector4d &v2, const double &eps)
			{
				return (fabs(v1(0) - v2(0)) < eps && fabs(v1(1) - v2(1)) < eps && fabs(v1(2) - v2(2)) < eps && fabs(v1(3) - v2(3)) < eps);
			}

			static Eigen::Vector3d createRandomVector3d()
			{
				Eigen::Vector3d vector;

				vector(0) = getRandomDouble();
				vector(1) = getRandomDouble();
				vector(2) = getRandomDouble();

				return vector;
			}

			static bool areMatrix3dEpsilonEqual(const Eigen::Matrix3d &m1, const Eigen::Matrix3d &m2, double epsilon)
			{
				for (int i = 0; i < 3; i++)
				{
					for (int j = 0; j < 3; j++)
					{
						bool tmp = fabs(m1(i, j) - m2(i, j)) < epsilon;
						if (!tmp)
						{
							return false;
						}
					}
				}

				return true;
			}

			static bool areMatrix4dEpsilonEqual(const Eigen::Matrix4d &m1, const Eigen::Matrix4d &m2, double epsilon)
			{
				for (int i = 0; i < 4; i++)
				{
					for (int j = 0; j < 4; j++)
					{
						bool tmp = fabs(m1(i, j) - m2(i, j)) < epsilon;
						if (!tmp)
						{
							return false;
						}
					}
				}

				return true;
			}
		};

	}
}
#endif