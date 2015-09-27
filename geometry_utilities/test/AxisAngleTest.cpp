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
	Eigen::Matrix3d rotationMatrix;
	Eigen::Matrix4d transform;

	rotationMatrix = GeometryUtilitiesTestHelper::createRandomRotationMatrix();

	transform(0, 0) = rotationMatrix(0, 0);
	transform(0, 1) = rotationMatrix(0, 1);
	transform(0, 2) = rotationMatrix(0, 2);
	transform(1, 0) = rotationMatrix(1, 0);
	transform(1, 1) = rotationMatrix(1, 1);
	transform(1, 2) = rotationMatrix(1, 2);
	transform(2, 0) = rotationMatrix(2, 0);
	transform(2, 1) = rotationMatrix(2, 1);
	transform(2, 2) = rotationMatrix(2, 2);

	AxisAngle axisAngle;
	RigidBodyTransform rigidTransform(transform);

	axisAngle.set(rotationMatrix);

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

}