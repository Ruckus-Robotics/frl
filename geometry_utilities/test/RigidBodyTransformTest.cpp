#include <gtest/gtest.h>
#include <AxisAngle.hpp>
#include <eigen3/Eigen/Eigen>
#include <RigidBodyTransform.hpp>

namespace geometry_utilities
{

class RigidBodyTransformTest : public ::testing::Test
{
	protected:

		virtual void SetUp()
		{

		}
		virtual void TearDown()
		{

		}

		int nTests = 1000;
};

TEST_F(RigidBodyTransformTest, testSetRotationAndZeroTranslationWithAxisAngle)
{
	Eigen::Vector3d vector;
	vector << 0, 0, 0;

	for (int i = 0; i < nTests; i++)
	{
		AxisAngle axisAngle = GeometryUtilitiesTestHelper::createRandomAxisAngle();

		RigidBodyTransform transform(axisAngle, vector);

		AxisAngle axisAngleToCheck;

		transform.getRotation(axisAngleToCheck);

		ASSERT_TRUE(axisAngle.epsilonEquals(axisAngleToCheck));
	}
}

}