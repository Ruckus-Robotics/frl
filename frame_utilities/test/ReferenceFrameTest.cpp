#include <gtest/gtest.h>
#include "ReferenceFrame.hpp"
#include "ReferenceFrameTestHelper.hpp"

class ReferenceFrameTest : public ::testing::Test
{
	protected:

		virtual void SetUp()
		{
			// this->root = ReferenceFrame::createARootFrame("root1");

		}
		virtual void TearDown()
		{
		}

		// Single chain of frames
		std::unique_ptr<ReferenceFrame> root1 = ReferenceFrame::createARootFrame("root1");
		// RandomlyChangingFrame frame1("frame1", root.get());
		// ReferenceFrame frame2 = ReferenceFrameTestHelper::createRandomUnchangingFrame("frame2", &frame1);
		// RandomlyChangingFrame frame3("frame3", &frame2);
		// ReferenceFrame frame4 = ReferenceFrameTestHelper::createRandomUnchangingFrame("frame4", &frame3);

		// // Chain with 2 branches
		// std::unique_ptr<ReferenceFrame> root2 = ReferenceFrame::createARootFrame("root2");
		// RandomlyChangingFrame frame5("frame5", root2.get());
		// ReferenceFrame frame6 = ReferenceFrameTestHelper::createRandomUnchangingFrame("frame6", &frame5);
		// RandomlyChangingFrame frame7("frame7", root2.get());
		// ReferenceFrame frame8 = ReferenceFrameTestHelper::createRandomUnchangingFrame("frame8", &frame7);
		// RandomlyChangingFrame frame9("frame9", &frame8);

	private:
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
};

TEST_F(ReferenceFrameTest, testWorldFramePointerStuff)
{
	const ReferenceFrame* worldFrame1 = ReferenceFrame::getWorldFrame();
	const ReferenceFrame* worldFrame2 = ReferenceFrame::getWorldFrame();

	ASSERT_TRUE(worldFrame1 == worldFrame2);
}

TEST_F(ReferenceFrameTest, testRootFramesArentTheSame)
{
	std::unique_ptr<ReferenceFrame> testRoot1 = ReferenceFrame::createARootFrame("TestRoot1");
	std::unique_ptr<ReferenceFrame> testRoot2 = ReferenceFrame::createARootFrame("TestRoot2");

	ASSERT_FALSE(testRoot1.get() == testRoot2.get());
}

TEST_F(ReferenceFrameTest, testFrameParents)
{
	root1.get()->getParentFrame();
}