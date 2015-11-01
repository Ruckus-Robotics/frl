#ifndef REFERENCE_FRAME_TEST_HELPER_HPP
#define REFERENCE_FRAME_TEST_HELPER_HPP

#include "frame_utilities/ReferenceFrame.hpp"
#include "GeometryUtilitiesTestHelper.hpp"
#include <iostream>
#include <random>
#include <memory>
#include <math.h>

using namespace frame_utilities;

class ReferenceFrameTestHelper
{
	public:
		static geometry_utilities::RigidBodyTransform createRandomTransformationMatrix()
		{
			geometry_utilities::Quaternion quaternion = geometry_utilities::GeometryUtilitiesTestHelper::createRandomQuaternion();
			Eigen::Vector3d translation = generateRandomTranslation();
			geometry_utilities::RigidBodyTransform transform(quaternion, translation);

			return transform;
		}

		static ReferenceFrame* getARandomFrame(std::vector<ReferenceFrame*> frames)
		{
			int index = rand() % frames.size();

			return frames[index];
		}

		static void updateAllFrames(std::vector<ReferenceFrame*> frames)
		{
			for (int i = 0; i < frames.size(); i++)
			{
				ReferenceFrame* tmpFrame = frames[i];
				tmpFrame->update();
			}
		}

		static geometry_utilities::RigidBodyTransform getTransformToRootByClimbingTree(ReferenceFrame* frame)
		{
			const std::vector<ReferenceFrame*> framesStartingWithRootEndingWithFrame = frame->getFramesStartingWithRootEndingWithThis();

			geometry_utilities::RigidBodyTransform transform;

			if (frame->getParentFrame() != nullptr)
			{
				for (int i = 0; i < framesStartingWithRootEndingWithFrame.size() ; i++)
				{
					transform *= framesStartingWithRootEndingWithFrame[i]->getTransformToParent();
				}
			}

			return transform;
		}

		static bool isVectorZeroWithinEpsilon(Eigen::Vector3d vector, double epsilon)
		{
			return (fabs(vector(0)) < epsilon && fabs(vector(1)) < epsilon && fabs(vector(2)) < epsilon) ? true : false;
		}

		static bool isIdentityQuaternionWithinEpsilon(geometry_utilities::Quaternion q, double epsilon)
		{
			return (fabs(q.getX()) < epsilon && fabs(q.getY()) < epsilon && fabs(q.getZ()) < epsilon && (fabs(q.getW()) - 1) < epsilon) ? true : false;
		}

		static ReferenceFrame* getRandomFrameFromVectorOfFrames(std::vector<ReferenceFrame*> vectorOfFrames)
		{
			return vectorOfFrames[rand() % 10];
		}

	private:
		static Eigen::Vector3d generateRandomTranslation()
		{
			Eigen::Vector3d vector;
			vector(0) = (rand() % 10 - 5);
			vector(1) = (rand() % 10 - 5);
			vector(2) = (rand() % 10 - 5);

			return vector;
		}

		static double generateRandomAngle()
		{
			std::random_device randomDevice;
			std::mt19937 mt(randomDevice());
			std::uniform_real_distribution<double> dist(0, 1);

			return (dist(mt) * 6.2 - 3.1);
			// return 0.0;
		}
};

class RandomlyChangingFrame : public ReferenceFrame
{
	public:
		RandomlyChangingFrame() : ReferenceFrame() {}
		RandomlyChangingFrame(const std::string& frameName, ReferenceFrame* parentFrame) : ReferenceFrame(frameName, parentFrame, false, false)
		{

		}

		RandomlyChangingFrame(const std::string& frameName, ReferenceFrame* parentFrame, geometry_utilities::RigidBodyTransform transformToParent) : ReferenceFrame(frameName, parentFrame, transformToParent, false, false)
		{

		}

		static std::shared_ptr<RandomlyChangingFrame> create(const std::string& frameName, ReferenceFrame* parentFrame)
		{
			geometry_utilities::RigidBodyTransform randomTransform = ReferenceFrameTestHelper::createRandomTransformationMatrix();
			std::shared_ptr<RandomlyChangingFrame> frame(new RandomlyChangingFrame(frameName, parentFrame, randomTransform));
			return frame;
		}

	protected:
		void updateTransformToParent(geometry_utilities::RigidBodyTransform& transformToParent)
		{
			geometry_utilities::RigidBodyTransform randomTransform = ReferenceFrameTestHelper::createRandomTransformationMatrix();
			// std::cout << "RandomlyChangingFrame updating!!" << std::endl;
			transformToParent = randomTransform;
		}
};

class RandomUnchangingFrame : public ReferenceFrame
{
	public:
		RandomUnchangingFrame() : ReferenceFrame() {}

		RandomUnchangingFrame(const std::string& frameName, ReferenceFrame* parentFrame, geometry_utilities::RigidBodyTransform transformToParent) : ReferenceFrame(frameName, parentFrame, transformToParent, false, false)
		{

		}

		static std::shared_ptr<RandomUnchangingFrame> create(const std::string& frameName, ReferenceFrame* parentFrame)
		{
			geometry_utilities::RigidBodyTransform randomTransform = ReferenceFrameTestHelper::createRandomTransformationMatrix();

			std::shared_ptr<RandomUnchangingFrame> frame(new RandomUnchangingFrame(frameName, parentFrame, randomTransform));
			return frame;
		}

	protected:
		void updateTransformToParent(geometry_utilities::RigidBodyTransform& transformToParent)
		{
			// std::cout << "RandomUnchangingFrame updating!!" << std::endl;
		}
};

#endif