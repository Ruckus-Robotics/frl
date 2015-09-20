#include "Tuple3d.hpp"
#include <random>
#include <memory>
#include <math.h>

namespace geometry_utilities
{

class GeometryUtilitiesTestHelper
{
	public:
		static double getRandomDouble()
		{
			// Generates random double between -1000 & 1000
			return ((rand() % 2000) - 1000);
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

		// static RigidBodyTransform generateRandomTransform(Random random)
		// {
		// 	RigidBodyTransform ret = new RigidBodyTransform();
		// 	ret.setRotationAndZeroTranslation(RandomTools.generateRandomRotation(random));
		// 	ret.setTranslation(RandomTools.generateRandomVector(random));

		// 	return ret;
		// }
};

}