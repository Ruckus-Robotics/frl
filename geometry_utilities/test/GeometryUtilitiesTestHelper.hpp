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

		static bool areAxisAngleEpsilonEqual(const AxisAngle &a1, const AxisAngle &a2, const double &eps)
		{
			return (fabs(a1.x - a2.x) < eps && fabs(a1.y - a2.y) < eps && fabs(a1.z - a2.z) < eps && fabs(a1.angle - a2.angle) < eps);
		}

		// static RigidBodyTransform generateRandomTransform(Random random)
		// {
		// 	RigidBodyTransform ret = new RigidBodyTransform();
		// 	ret.setRotationAndZeroTranslation(RandomTools.generateRandomRotation(random));
		// 	ret.setTranslation(RandomTools.generateRandomVector(random));

		// 	return ret;
		// }
};

}