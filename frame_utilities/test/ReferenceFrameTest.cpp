#include <gtest/gtest.h>
#include "ReferenceFrame.hpp"

class ReferenceFrameTest : public ::testing::Test 
{
protected:

	virtual void SetUp() 
	{
		std::unique_ptr<ReferenceFrame> rootFrame = ReferenceFrame::createARootFrame("root");
	}
	virtual void TearDown()
	{
	}

	geometry_msgs::Transform createRandomTransformationMatrix()
	{
		geometry_msgs::Transform transform;
		tf::Quaternion quaternion = generateRandomQuaternion();
	}

	tf::Quaternion generateRandomQuaternion()
	{
		tf::Quaternion quaternion;
		quaternion.setRPY(generateRandomAngle(),generateRandomAngle(),generateRandomAngle());

		return quaternion;
	}

	double generateRandomAngle()
	{
		std::random_device randomDevice;
	    std::mt19937 mt(randomDevice());
	    std::uniform_real_distribution<double> dist(0, 1);

	    return (dist(mt)*6.28 - 3.14);
	}
};

TEST(ReferenceFrameTest,testWorldFramePointerStuff)
{
	const ReferenceFrame* worldFrame1 = ReferenceFrame::getWorldFrame();
	const ReferenceFrame* worldFrame2 = ReferenceFrame::getWorldFrame();

	ASSERT_TRUE(worldFrame1==worldFrame2);
}