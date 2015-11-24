#include <gtest/gtest.h>
#include "frame_utilities/ReferenceFrame.hpp"
#include "frame_utilities/FramePoint.hpp"
#include "ReferenceFrameTestHelper.hpp"
#include "GeometryUtilitiesTestHelper.hpp"

using namespace frame_utilities;

class FramePointTest : public ::testing::Test
{
protected:

    virtual void SetUp()
    {
    }

    virtual void TearDown()
    {
    }

    std::unique_ptr <ReferenceFrame> root1 = ReferenceFrame::createARootFrame("root1");
    std::unique_ptr <ReferenceFrame> root2 = ReferenceFrame::createARootFrame("root2");

    int nTests = 1000;

private:

};

TEST_F(FramePointTest, testChangeFrameToThisFrameDoesNothing)
{
    geometry_utilities::RigidBodyTransform transform1;
    for(int i = 0; i<nTests;i++)
    {
        transform1.setIdentity();
        Eigen::Vector3d rpy = geometry_utilities::GeometryUtilitiesTestHelper::createRandomVector3d();
        Eigen::Vector3d translation = geometry_utilities::GeometryUtilitiesTestHelper::createRandomVector3d();
        transform1.setEuler(rpy);
        transform1.setTranslation(translation);

        std::shared_ptr<ReferenceFrame> frameA(new RandomUnchangingFrame("A", root1.get(), transform1));

        double x = geometry_utilities::GeometryUtilitiesTestHelper::getRandomDouble();
        double y = geometry_utilities::GeometryUtilitiesTestHelper::getRandomDouble();
        double z = geometry_utilities::GeometryUtilitiesTestHelper::getRandomDouble();

        FramePoint framePoint("FramePointA",frameA.get(),x,y,z);

        EXPECT_TRUE(framePoint.getX() - x < 1e-8);
        EXPECT_TRUE(framePoint.getY() - y < 1e-8);
        EXPECT_TRUE(framePoint.getZ() - z < 1e-8);

    }
}