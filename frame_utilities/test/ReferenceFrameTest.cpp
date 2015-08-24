#include <gtest/gtest.h>
#include "ReferenceFrame.hpp"
#include "ReferenceFrameTestHelper.hpp"

class ReferenceFrameTest : public ::testing::Test
{
	protected:

		virtual void SetUp()
		{
			std::unique_ptr<ReferenceFrame> root = ReferenceFrame::createARootFrame("root1");
			// RandomlyChangingFrame frame1("frame1", root.get());

			std::unique_ptr<ReferenceFrame> root2 = ReferenceFrame::createARootFrame("root2");
		}
		virtual void TearDown()
		{
		}
};

class RandomlyChangingFrame : public ReferenceFrame
{
	public:
		RandomlyChangingFrame(const std::string &frameName, ReferenceFrame* const parentFrame) : ReferenceFrame(frameName, parentFrame, false, false)
		{

		}

	protected:
		void updateTransformToParent(tf::Transform &transformToParent)
		{

		}
};

TEST(ReferenceFrameTest, testWorldFramePointerStuff)
{
	const ReferenceFrame* worldFrame1 = ReferenceFrame::getWorldFrame();
	const ReferenceFrame* worldFrame2 = ReferenceFrame::getWorldFrame();

	ASSERT_TRUE(worldFrame1 == worldFrame2);
}

TEST(ReferenceFrameTest, testRootFramesArentTheSame)
{
	std::unique_ptr<ReferenceFrame> testRoot1 = ReferenceFrame::createARootFrame("TestRoot1");
	std::unique_ptr<ReferenceFrame> testRoot2 = ReferenceFrame::createARootFrame("TestRoot2");

	ASSERT_FALSE(testRoot1.get() == testRoot2.get());
}