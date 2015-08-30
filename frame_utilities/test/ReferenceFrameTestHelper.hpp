#include "ReferenceFrame.hpp"
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <random>
#include <memory>

class RandomlyChangingFrame : public ReferenceFrame
{
	public:
		RandomlyChangingFrame() : ReferenceFrame() {};
		RandomlyChangingFrame(const std::string &frameName, ReferenceFrame* parentFrame) : ReferenceFrame(frameName, parentFrame, false, false)
		{

		}

		static std::shared_ptr<RandomlyChangingFrame> create(const std::string &frameName, ReferenceFrame* parentFrame)
		{
			std::shared_ptr<RandomlyChangingFrame> frame(new RandomlyChangingFrame(frameName, parentFrame));
			return frame;
		}

	protected:
		void updateTransformToParent(tf::Transform &transformToParent)
		{

		}
};

class RandomUnchangingFrame : public ReferenceFrame
{
	public:
		RandomUnchangingFrame() : ReferenceFrame() {};
		RandomUnchangingFrame(const std::string &frameName, ReferenceFrame* parentFrame) : ReferenceFrame(frameName, parentFrame, false, false)
		{

		}

		static std::shared_ptr<RandomUnchangingFrame> create(const std::string &frameName, ReferenceFrame* parentFrame)
		{
			std::shared_ptr<RandomUnchangingFrame> frame(new RandomUnchangingFrame(frameName, parentFrame));
			return frame;
		}

	protected:
		void updateTransformToParent(tf::Transform &transformToParent)
		{

		}
};

class ReferenceFrameTestHelper
{
	public:
		static tf::Transform createRandomTransformationMatrix()
		{
			tf::Quaternion quaternion = generateRandomQuaternion();
			tf::Vector3 translation = generateRandomTranslation();
			tf::Transform transform(quaternion, translation);

			return transform;
		}

	private:
		static tf::Vector3 generateRandomTranslation()
		{
			tf::Vector3 vector;
			vector.setX(rand() % 10 - 5);
			vector.setY(rand() % 10 - 5);
			vector.setZ(rand() % 10 - 5);

			return vector;
		}

		static tf::Quaternion generateRandomQuaternion()
		{
			tf::Quaternion quaternion;
			quaternion.setRPY(generateRandomAngle(), generateRandomAngle(), generateRandomAngle());

			return quaternion;
		}

		static double generateRandomAngle()
		{
			std::random_device randomDevice;
			std::mt19937 mt(randomDevice());
			std::uniform_real_distribution<double> dist(0, 1);

			return (dist(mt) * 6.28 - 3.14);
		}
};