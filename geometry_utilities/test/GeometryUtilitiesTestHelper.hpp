#include "Tuple3d.hpp"
#include <random>
#include <memory>
#include <math.h>
#include <eigen3/Eigen/Eigen>
#include "AxisAngle.hpp"

namespace geometry_utilities
{

class GeometryUtilitiesTestHelper
{
	public:
		static double getRandomDouble()
		{
			// Generates random double between -1000 & 1000
			return (2000.0 * rand() / RAND_MAX - 1000);
		}

		static std::vector<double> getRandom3dVector()
		{
			std::vector<double> vector(3);
			vector[0] = getRandomDouble();
			vector[1] = getRandomDouble();
			vector[2] = getRandomDouble();

			return vector;
		}

		static Tuple3d getRandomTuple3d()
		{
			Tuple3d tuple;
			tuple.setX(getRandomDouble());
			tuple.setY(getRandomDouble());
			tuple.setZ(getRandomDouble());

			return tuple;
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

			double angle = (2 * M_PI * rand() / RAND_MAX - M_PI);
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

			double angle = (2 * M_PI * rand() / RAND_MAX - M_PI);
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

			double angle = (2 * M_PI * rand() / RAND_MAX - M_PI);
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

			transform(3, 3) = 1.0;

			return transform;
		}

		static Eigen::Matrix4d createRandomMatrix4d()
		{
			Eigen::Matrix4d matrix;

			matrix(0, 0) = rand() / RAND_MAX;
			matrix(0, 1) = rand() / RAND_MAX;
			matrix(0, 2) = rand() / RAND_MAX;
			matrix(0, 3) = rand() / RAND_MAX;
			matrix(1, 0) = rand() / RAND_MAX;
			matrix(1, 1) = rand() / RAND_MAX;
			matrix(1, 2) = rand() / RAND_MAX;
			matrix(1, 3) = rand() / RAND_MAX;
			matrix(2, 0) = rand() / RAND_MAX;
			matrix(2, 1) = rand() / RAND_MAX;
			matrix(2, 2) = rand() / RAND_MAX;
			matrix(2, 3) = rand() / RAND_MAX;
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

		static AxisAngle createRandomAxisAngle()
		{
			double x, y, z;
			x = getRandomDouble();
			y = getRandomDouble();
			z = getRandomDouble();

			double mag = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));

			x *= 1 / mag;
			y *= 1 / mag;
			z *= 1 / mag;

			double angle = (2 * M_PI * rand() / RAND_MAX - M_PI);

			AxisAngle ret(x, y, z, angle);
			return ret;
		}

		static bool areAxisAngleEpsilonEqual(const AxisAngle &a1, const AxisAngle &a2, const double &eps)
		{
			if ((fabs(a1.x - a2.x) < eps && fabs(a1.y - a2.y) < eps && fabs(a1.z - a2.z) < eps && fabs(a1.angle - a2.angle) < eps) ||
			        (fabs(-a1.x - a2.x) < eps && fabs(-a1.y - a2.y) < eps && fabs(-a1.z - a2.z) < eps && fabs(-a1.angle - a2.angle) < eps))
			{
				return true;
			}

			if ((fabs(a1.x - a2.x) < eps && fabs(a1.y - a2.y) < eps && fabs(a1.z - a2.z) < eps))
			{
				if (M_PI - fabs(a1.angle) < 1e-4 && M_PI - fabs(a2.angle) < 1e-4)
				{
					return true;
				}
				else
				{
					return false;
				}
			}
		}

		static Eigen::Vector3d createRandomVector3d()
		{
			Eigen::Vector3d vector;

			vector(0) = getRandomDouble();
			vector(1) = getRandomDouble();
			vector(2) = getRandomDouble();

			return vector;
		}
};

}