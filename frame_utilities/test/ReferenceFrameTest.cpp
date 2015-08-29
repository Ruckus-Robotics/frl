#include <gtest/gtest.h>
#include "ReferenceFrame.hpp"
#include "ReferenceFrameTestHelper.hpp"

class ReferenceFrameTest : public ::testing::Test
{
	protected:

		virtual void SetUp()
		{
			RandomlyChangingFrame tmpFrame1("frame1", root1.get());
			frame1 = tmpFrame1;
			RandomlyChangingFrame tmpFrame3("frame3", &frame2);
			frame3 = tmpFrame3;

			RandomlyChangingFrame tmpFrame5("frame5", root2.get());
			frame5 = tmpFrame5;
			RandomlyChangingFrame tmpFrame7("frame7", &frame6);
			frame7 = tmpFrame7;
			RandomlyChangingFrame tmpFrame9("frame9", &frame8);
			frame9 = tmpFrame9;

		}
		virtual void TearDown()
		{
		}

		// Single chain of frames
		std::unique_ptr<ReferenceFrame> root1 = ReferenceFrame::createARootFrame("root1");
		RandomlyChangingFrame frame1;
		ReferenceFrame frame2 = ReferenceFrameTestHelper::createRandomUnchangingFrame("frame2", &frame1);
		RandomlyChangingFrame frame3;
		ReferenceFrame frame4 = ReferenceFrameTestHelper::createRandomUnchangingFrame("frame4", &frame3);

		// Chain with 2 branches
		std::unique_ptr<ReferenceFrame> root2 = ReferenceFrame::createARootFrame("root2");
		RandomlyChangingFrame frame5;
		ReferenceFrame frame6 = ReferenceFrameTestHelper::createRandomUnchangingFrame("frame6", &frame5);
		RandomlyChangingFrame frame7;
		ReferenceFrame frame8 = ReferenceFrameTestHelper::createRandomUnchangingFrame("frame8", &frame7);
		RandomlyChangingFrame frame9;

	private:

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
	const ReferenceFrame* test = root1.get()->getParentFrame();
	ASSERT_TRUE(test == NULL);

}