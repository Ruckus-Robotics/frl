#include "FrameTuple.hpp"
#include "ReferenceFrameTestHelper.hpp"
#include <gtest/gtest.h>

namespace frame_utilities
{

class FrameTupleTest : public ::testing::Test
{
	protected:

		virtual void SetUp()
		{
			allFrames.push_back(root1.get());
			allFrames.push_back(frame1.get());
			allFrames.push_back(frame2.get());
			allFrames.push_back(frame3.get());
		}
		virtual void TearDown()
		{
			allFrames.clear();
		}

		std::unique_ptr<ReferenceFrame> root1 = ReferenceFrame::createARootFrame("root1");

		std::shared_ptr<RandomUnchangingFrame> frame1 = RandomUnchangingFrame::create("frame1", root1.get());
		std::shared_ptr<RandomUnchangingFrame> frame2 = RandomUnchangingFrame::create("frame2", frame1.get());
		std::shared_ptr<RandomUnchangingFrame> frame3 = RandomUnchangingFrame::create("frame3", frame2.get());

		std::vector<ReferenceFrame*> allFrames;

		int nTests = 1000;
};

}