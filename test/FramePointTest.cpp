#include <gtest/gtest.h>
#include "frl/frame_utilities/ReferenceFrame.hpp"
#include "frl/frame_utilities/FramePoint.hpp"
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

    int nTests = 1000;

private:

};

TEST_F(FramePointTest, testChangeFrame)
{
    geometry_utilities::RigidBodyTransform transform1;

    transform1.setIdentity();
    Eigen::Vector3d rpy(M_PI/2, 0, 0);
    Eigen::Vector3d translation(5.0, 0.0, 0.0);
    transform1.setEuler(rpy);
    transform1.setTranslation(translation);

    std::shared_ptr<ReferenceFrame> frameA(new RandomUnchangingFrame("A", root1.get(), transform1));

    transform1.setIdentity();
    rpy.setZero();
    rpy << 0, M_PI/2, 0.0;
    translation << 5.0, 0, 0;
    transform1.setEuler(rpy);
    transform1.setTranslation(translation);

    std::shared_ptr<ReferenceFrame> frameB(new RandomUnchangingFrame("B", frameA.get(), transform1));

    transform1.setIdentity();
    rpy.setZero();
    rpy << 0, 0, M_PI/2;
    translation << 5.0, 0, 0;
    transform1.setEuler(rpy);
    transform1.setTranslation(translation);

    std::shared_ptr<ReferenceFrame> frameC(new RandomUnchangingFrame("C", frameB.get(), transform1));

    double x = 4.0;
    double y = 7.0;
    double z = 2.0;

    FramePoint framePoint("FramePoint", frameC.get(), x, y, z);

    EXPECT_TRUE(fabs(framePoint.getX() - x) < 1e-8);
    EXPECT_TRUE(fabs(framePoint.getY() - y) < 1e-8);
    EXPECT_TRUE(fabs(framePoint.getZ() - z) < 1e-8);

    framePoint.changeFrame(frameB.get());

    EXPECT_TRUE(fabs(framePoint.getX() + 2) < 1e-8);
    EXPECT_TRUE(fabs(framePoint.getY() - 4) < 1e-8);
    EXPECT_TRUE(fabs(framePoint.getZ() - 2) < 1e-8);

    framePoint.changeFrame(frameA.get());

    EXPECT_TRUE(fabs(framePoint.getX() - 7) < 1e-8);
    EXPECT_TRUE(fabs(framePoint.getY() - 4) < 1e-8);
    EXPECT_TRUE(fabs(framePoint.getZ() - 2) < 1e-8);

    framePoint.changeFrame(root1.get());

    EXPECT_TRUE(fabs(framePoint.getX() - 12) < 1e-8);
    EXPECT_TRUE(fabs(framePoint.getY() + 2) < 1e-8);
    EXPECT_TRUE(fabs(framePoint.getZ() - 4) < 1e-8);


}


