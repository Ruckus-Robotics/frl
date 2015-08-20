#include <gtest/gtest.h>
#include "ReferenceFrame.hpp"

class ReferenceFrameTest : public ::testing::Test 
{
protected:

	virtual void SetUp() 
	{
		// std::unique_ptr<ReferenceFrame> ReferenceFrame::worldFrame = ReferenceFrame::createAWorldFrame("World");
		std::unique_ptr<ReferenceFrame> rootFrame = ReferenceFrame::createARootFrame("root");
	}
	virtual void TearDown()
	{
	}
};

TEST(ReferenceFrameTest,testCreateWorldFrame)
{

}