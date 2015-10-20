#include "geometry_utilities/Quaternion.hpp"
#include "GeometryUtilitiesTestHelper.hpp"
#include <gtest/gtest.h>

namespace geometry_utilities
{

class QuaternionTest : public ::testing::Test
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

TEST_F(QuaternionTest, testConstructors1)
{
	Eigen::Matrix3d rot1 = GeometryUtilitiesTestHelper::createRandomRotationMatrix();
	Eigen::Matrix4d trans1 = GeometryUtilitiesTestHelper::createRandomTransformationMatrix();

	Quaternion q1(rot1);
	Quaternion q2(trans1);

	Eigen::Vector4d v1;
	Eigen::Vector4d v2;

	q1.get(v1);
	q2.get(v2);

	EXPECT_TRUE(GeometryUtilitiesTestHelper::areVector4dsEpsilonEqual(v1, v2, 1e-5));
}

}