#include <gtest/gtest.h>
#include "ReferenceFrame.hpp"

class ReferenceFrameTest : public ::testing::Test 
{
protected:

	virtual void SetUp() 
	{
		ReferenceFrame *rootFrame = ReferenceFrame::createARootFrame("root");
	}
	virtual void TearDown()
	{
	}
};

TEST(ReferenceFrameTest,testCreateWorldFrame)
{

}