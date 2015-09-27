#include <gtest/gtest.h>
#include <AxisAngle.hpp>
#include <RigidBodyTransform.hpp>
#include <eigen3/Eigen/Eigen>
#include "GeometryUtilitiesTestHelper.hpp"
#include <ctime>

namespace geometry_utilities
{

class AxisAngleTest : public ::testing::Test
{
	protected:

		virtual void SetUp()
		{
			std::srand( time(NULL) );
		}
		virtual void TearDown()
		{

		}

		int nTests = 1000;
};

TEST_F(AxisAngleTest, testSetFromMatrix4d_1)
{
	Eigen::Matrix4d transform;

	transform = GeometryUtilitiesTestHelper::createRandomTransformationMatrix();

	AxisAngle axisAngle;
	RigidBodyTransform rigidTransform(transform);

	axisAngle.set(transform);

	AxisAngle axisAngleToCheck;
	rigidTransform.getRotation(axisAngleToCheck);

	bool equal = GeometryUtilitiesTestHelper::areAxisAngleEpsilonEqual(axisAngle, axisAngleToCheck, 1e-5);

	EXPECT_TRUE(equal);
	EXPECT_TRUE(axisAngle.epsilonEquals(axisAngleToCheck, 1e-5));
}

TEST_F(AxisAngleTest, testSetFromMatrix4d_2)
{
	Eigen::Matrix4d transform;
	transform << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

	AxisAngle axisAngle;

	axisAngle.set(transform);

	AxisAngle axisAngleToCheck(0, 1, 0, 0);

	bool equal = GeometryUtilitiesTestHelper::areAxisAngleEpsilonEqual(axisAngle, axisAngleToCheck, 1e-6);
	EXPECT_TRUE(axisAngle.equals(axisAngleToCheck));
}

TEST_F(AxisAngleTest, testSetFromMatrix3d_1)
{
	Eigen::Matrix3d rot;

	rot = GeometryUtilitiesTestHelper::createRandomRotationMatrix();

	AxisAngle axisAngle;
	RigidBodyTransform rigidTransform;
	rigidTransform.setRotation(rot);

	axisAngle.set(rot);

	AxisAngle axisAngleToCheck;
	rigidTransform.getRotation(axisAngleToCheck);

	bool equal = GeometryUtilitiesTestHelper::areAxisAngleEpsilonEqual(axisAngle, axisAngleToCheck, 1e-5);

	EXPECT_TRUE(equal);
	EXPECT_TRUE(axisAngle.epsilonEquals(axisAngleToCheck, 1e-5));
}

}