#include "ReferenceFrame.hpp"
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <random>


class ReferenceFrameTestHelper
{
	public:
		ReferenceFrameTestHelper();

		ReferenceFrame createRandomUnchangingFrame(const std::string &frameName, ReferenceFrame* const parentFrame)
		{
			tf::Transform randomTransform = createRandomTransformationMatrix();

			ReferenceFrame frame = ReferenceFrame::createFrameWithUnchangingTransformToParent(frameName, parentFrame, randomTransform,
			                       false, false);
		}

		tf::Transform createRandomTransformationMatrix()
		{
			tf::Quaternion quaternion = generateRandomQuaternion();
			tf::Vector3 translation = generateRandomTranslation();
			tf::Transform transform(quaternion, translation);
		}

	private:
		tf::Vector3 generateRandomTranslation()
		{
			tf::Vector3 vector;
			vector.setX(rand() % 10 - 5);
			vector.setY(rand() % 10 - 5);
			vector.setZ(rand() % 10 - 5);

			return vector;
		}

		tf::Quaternion generateRandomQuaternion()
		{
			tf::Quaternion quaternion;
			quaternion.setRPY(generateRandomAngle(), generateRandomAngle(), generateRandomAngle());

			return quaternion;
		}

		double generateRandomAngle()
		{
			std::random_device randomDevice;
			std::mt19937 mt(randomDevice());
			std::uniform_real_distribution<double> dist(0, 1);

			return (dist(mt) * 6.28 - 3.14);
		}
};