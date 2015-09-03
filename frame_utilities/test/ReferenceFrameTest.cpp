#include <gtest/gtest.h>
#include "ReferenceFrame.hpp"
#include "ReferenceFrameTestHelper.hpp"

class ReferenceFrameTest : public ::testing::Test
{
	protected:

		virtual void SetUp()
		{
			allFrames.push_back(root1.get());
			allFrames.push_back(frame1.get());
			allFrames.push_back(frame2.get());
			allFrames.push_back(frame3.get());

			allFrames.push_back(root2.get());
			allFrames.push_back(frame4.get());
			allFrames.push_back(frame5.get());
			allFrames.push_back(frame6.get());
			allFrames.push_back(frame7.get());
		}
		virtual void TearDown()
		{
			allFrames.clear();
		}

		std::unique_ptr<ReferenceFrame> root1 = ReferenceFrame::createARootFrame("root1");
		std::unique_ptr<ReferenceFrame> root2 = ReferenceFrame::createARootFrame("root2");

		std::shared_ptr<RandomUnchangingFrame> frame1 = RandomUnchangingFrame::create("frame1", root1.get());
		std::shared_ptr<RandomUnchangingFrame> frame2 = RandomUnchangingFrame::create("frame2", frame1.get());
		std::shared_ptr<RandomUnchangingFrame> frame3 = RandomUnchangingFrame::create("frame3", frame2.get());

		std::shared_ptr<RandomlyChangingFrame> frame4 = RandomlyChangingFrame::create("frame4", root2.get());
		std::shared_ptr<RandomlyChangingFrame> frame5 = RandomlyChangingFrame::create("frame5", frame4.get());
		std::shared_ptr<RandomUnchangingFrame> frame6 = RandomUnchangingFrame::create("frame6", root2.get());
		std::shared_ptr<RandomlyChangingFrame> frame7 = RandomlyChangingFrame::create("frame7", frame6.get());

		std::vector<ReferenceFrame*> allFrames;

		int nTests = 100;

	private:

};

TEST_F(ReferenceFrameTest, testRootsHaveNullParent)
{
	ASSERT_TRUE(root1->getParentFrame() == nullptr);
	ASSERT_TRUE(root2->getParentFrame() == nullptr);
}

TEST_F(ReferenceFrameTest, testWorldFramePointerStuff)
{
	const ReferenceFrame* worldFrame1 = ReferenceFrame::getWorldFrame();
	const ReferenceFrame* worldFrame2 = ReferenceFrame::getWorldFrame();
}

TEST_F(ReferenceFrameTest, testRootFramesArentTheSame)
{
	ASSERT_FALSE(root1 == root2);
}

TEST_F(ReferenceFrameTest, testGetRootFrame)
{
	ASSERT_TRUE(frame2->getRootFrame() == root1.get());
	ASSERT_TRUE(frame7->getRootFrame() == frame5->getRootFrame());

	//Throws error if these frames dont have the same root frames.
	frame7.get()->verifyFramesHaveSameRoot(frame6.get());

	try
	{
		frame7.get()->verifyFramesHaveSameRoot(frame1.get());
		ASSERT_TRUE(false);
	}
	catch ( ... )
	{
		ASSERT_TRUE(true);
	}
}

TEST_F(ReferenceFrameTest, testGetTransformToDesiredFrame)
{
	tf::Transform transformFrom7To6 = frame7.get()->getTransformToDesiredFrame(frame6.get());
	tf::Transform transformFrom6To7 = frame6.get()->getTransformToDesiredFrame(frame7.get());

	tf::Transform shouldBeIdentity = transformFrom7To6;

	shouldBeIdentity *= transformFrom6To7;

	EXPECT_TRUE(ReferenceFrameTestHelper::isTransformIdentityWithinEpsilon(shouldBeIdentity, 1e-5));

	tf::Transform transformFrom4To7 = frame4.get()->getTransformToDesiredFrame(frame7.get());
	tf::Transform transformFrom7To4 = frame7.get()->getTransformToDesiredFrame(frame4.get());

	shouldBeIdentity = transformFrom4To7;
	shouldBeIdentity *= transformFrom7To4;

	EXPECT_TRUE(ReferenceFrameTestHelper::isTransformIdentityWithinEpsilon(shouldBeIdentity, 1e-5));
}

TEST_F(ReferenceFrameTest, testGetTransformToParent)
{
	for (int i = 1; i < allFrames.size(); i++)
	{
		ReferenceFrame* tmpFrame2 = allFrames[i];

		ReferenceFrame* parentFrame = tmpFrame2->getParentFrame();

		ReferenceFrameTestHelper::printQuaternion(tmpFrame2->getTransformToDesiredFrame(parentFrame));
		// ReferenceFrameTestHelper::areTransformsEpsilonEqual(tmpFrame2->getTransformToParent(), tmpFrame2->getTransformToDesiredFrame(parentFrame), 1e-5);
	}
}