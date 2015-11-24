#include <gtest/gtest.h>
#include "frame_utilities/ReferenceFrame.hpp"
#include "frame_utilities/FramePoint.hpp"
#include "ReferenceFrameTestHelper.hpp"

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

    std::unique_ptr<ReferenceFrame> root1 = ReferenceFrame::createARootFrame("root1");
    std::unique_ptr<ReferenceFrame> root2 = ReferenceFrame::createARootFrame("root2");

    int nTests = 1000;

private:

};

TEST_F(FramePointTest, testChangeFrameToThisFrameDoesNothing)
{
    geometry_utilities::RigidBodyTransform transform1;

    transform1.setIdentity();
    Eigen::Vector3d rpy(0, 0, 0);
    Eigen::Vector3d translation(1, 2, 3);
    transform1.setEuler(rpy);
    transform1.setTranslation(translation);

    std::shared_ptr<ReferenceFrame> frameA(new RandomUnchangingFrame("A", root1.get(), transform1));

    transform1.setIdentity();
    rpy.setZero();
    rpy << 0, 0, M_PI;
    translation << 0, 0, 0;
    transform1.setEuler(rpy);
    transform1.setTranslation(translation);

    std::shared_ptr<ReferenceFrame> frameB(new RandomUnchangingFrame("B", frameA.get(), transform1));

    transform1.setIdentity();
    rpy.setZero();
    rpy << M_PI, 0, 0;
    translation << 0, 0, 0;
    transform1.setEuler(rpy);
    transform1.setTranslation(translation);

    std::shared_ptr<ReferenceFrame> frameC(new RandomUnchangingFrame("C", frameB.get(), transform1));

    transform1.setIdentity();
    rpy.setZero();
    rpy << 0, M_PI, 0;
    translation << 0, 0, 0;
    transform1.setEuler(rpy);
    transform1.setTranslation(translation);

    std::shared_ptr<ReferenceFrame> frameD(new RandomUnchangingFrame("D", frameC.get(), transform1));

    double x = 3;
    double y = 1;
    double z = 4;

    FramePoint framePoint("FramePointA", frameA.get(), x, y, z);

    EXPECT_TRUE(framePoint.getX() - x < 1e-8);
    EXPECT_TRUE(framePoint.getY() - y < 1e-8);
    EXPECT_TRUE(framePoint.getZ() - z < 1e-8);

    framePoint.changeFrame(frameA.get());

    EXPECT_TRUE(framePoint.getX() - x < 1e-8);
    EXPECT_TRUE(framePoint.getY() - y < 1e-8);
    EXPECT_TRUE(framePoint.getZ() - z < 1e-8);

}


