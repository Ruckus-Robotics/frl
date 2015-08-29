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
			RandomlyChangingFrame tmpFrame7("frame7", root2.get());
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

		int nTests = 100;

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
	for (int i = 0; i < nTests; i++)
	{
		const ReferenceFrame* root1ParentFrame = root1.get()->getParentFrame();
		ASSERT_TRUE(root1ParentFrame == NULL);

		const ReferenceFrame* frame1ParentFrame = frame1.getParentFrame();
		ASSERT_TRUE(frame1ParentFrame == root1.get());

		const ReferenceFrame* frame2ParentFrame = frame2.getParentFrame();
		ASSERT_TRUE(frame2ParentFrame == &frame1	);

		const ReferenceFrame* frame3ParentFrame = frame3.getParentFrame();
		ASSERT_TRUE(frame3ParentFrame == &frame2);

		const ReferenceFrame* frame4ParentFrame = frame4.getParentFrame();
		ASSERT_TRUE(frame4ParentFrame == &frame3);

		const ReferenceFrame* root2ParentFrame = root2.get()->getParentFrame();
		ASSERT_TRUE(root2ParentFrame == NULL);

		const ReferenceFrame* frame5ParentFrame = frame5.getParentFrame();
		ASSERT_TRUE(frame5ParentFrame == root2.get());

		const ReferenceFrame* frame6ParentFrame = frame6.getParentFrame();
		ASSERT_TRUE(frame6ParentFrame == &frame5);

		const ReferenceFrame* frame7ParentFrame = frame7.getParentFrame();
		ASSERT_TRUE(frame7ParentFrame == root2.get());

		const ReferenceFrame* frame8ParentFrame = frame8.getParentFrame();
		ASSERT_TRUE(frame8ParentFrame == &frame7);

		const ReferenceFrame* frame9ParentFrame = frame9.getParentFrame();
		ASSERT_TRUE(frame9ParentFrame == &frame8);
	}
}