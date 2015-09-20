#include "FrameTuple.hpp"
#include <random>
#include <memory>
#include <math.h>

namespace frame_utilities
{

class FrameUtilitiesTestHelper
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

		static FrameTuple getRandomFrameTuple(const std::string &name, ReferenceFrame* referenceFrame)
		{
			FrameTuple frameTuple(name, referenceFrame);
			frameTuple.setX(getRandomDouble());
			frameTuple.setY(getRandomDouble());
			frameTuple.setZ(getRandomDouble());

			return frameTuple;
		}
};

}