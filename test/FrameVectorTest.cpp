#include <gtest/gtest.h>
#include "frl/frame_utilities/FrameVector.hpp"
#include "ReferenceFrameTestHelper.hpp"
#include "GeometryUtilitiesTestHelper.hpp"

using namespace frame_utilities;

class FrameVectorTest : public ::testing::Test
{
protected:

    virtual void SetUp()
    {
        std::srand( time(NULL) );
    }

    virtual void TearDown()
    {
    }

    std::unique_ptr<ReferenceFrame> root1 = ReferenceFrame::createARootFrame("root1");

    int nTests = 1000;

private:

};

TEST_F(FrameVectorTest, testBlah)
{

}