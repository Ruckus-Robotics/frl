#include <gtest/gtest.h>
#include <AxisAngle.hpp>
#include <eigen3/Eigen/Eigen>
#include <RigidBodyTransform.hpp>
#include "GeometryUtilitiesTestHelper.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "Quaternion.hpp"

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

	for (int i = 0; i < nTests; i++)
	{
		vector << 0, 0, 0;

		for (int i = 0; i < nTests; i++)
		{
			AxisAngle axisAngle = GeometryUtilitiesTestHelper::createRandomAxisAngle();

			RigidBodyTransform transform(axisAngle, vector);

			AxisAngle axisAngleToCheck;

			transform.getRotation(axisAngleToCheck);

			if (!(GeometryUtilitiesTestHelper::areAxisAngleEpsilonEqual(axisAngle, axisAngleToCheck, 1e-5)))
			{
				std::cout << axisAngle << std::endl;
				std::cout << axisAngleToCheck << std::endl;
			}

			ASSERT_TRUE(GeometryUtilitiesTestHelper::areAxisAngleEpsilonEqual(axisAngle, axisAngleToCheck, 1e-5));
		}
	}
}

TEST_F(RigidBodyTransformTest, testCreateTransformWithAxisAngle4dAndRandomVector3d)
{
	Eigen::Vector3d vector;

	for (int i = 0; i < nTests; i++)
	{
		vector = GeometryUtilitiesTestHelper::createRandomVector3d();

		for (int i = 0; i < nTests; i++)
		{
			AxisAngle axisAngle = GeometryUtilitiesTestHelper::createRandomAxisAngle();

			RigidBodyTransform transform(axisAngle, vector);

			AxisAngle axisAngleToCheck;

			transform.getRotation(axisAngleToCheck);

			if (!(GeometryUtilitiesTestHelper::areAxisAngleEpsilonEqual(axisAngle, axisAngleToCheck, 1e-5)))
			{
				std::cout << axisAngle << std::endl;
				std::cout << axisAngleToCheck << std::endl;
			}

			ASSERT_TRUE(GeometryUtilitiesTestHelper::areAxisAngleEpsilonEqual(axisAngle, axisAngleToCheck, 1e-5));
		}
	}
}

TEST_F(RigidBodyTransformTest, testNormalize)
{
	for (int i = 0; i < nTests; i++)
	{
		Eigen::Matrix4d matrix = GeometryUtilitiesTestHelper::createRandomMatrix4d();

		RigidBodyTransform transform(matrix);

		transform.normalize();

		Eigen::Matrix4d matrixToCheck;
		transform.get(matrixToCheck);

		if (!GeometryUtilitiesTestHelper::checkOrthogonality(matrixToCheck))
		{
			std::cout << matrixToCheck << std::endl;
			std::cout << matrix << std::endl;
			ASSERT_TRUE(false);
		}
	}
}

TEST_F(RigidBodyTransformTest, testUseQuaternions_1)
{
	for (int i = 0; i < nTests; i++)
	{
		Quaternion quat1 = GeometryUtilitiesTestHelper::createRandomQuaternion();

		RigidBodyTransform transform;
		transform.setRotationAndZeroTranslation(quat1);

		Quaternion quatToCheck;
		transform.getRotation(quatToCheck);

		if (!GeometryUtilitiesTestHelper::areQuaternionsEpsilonEqual(quat1, quatToCheck, 1e-5))
		{
			std::cout << quat1 << std::endl;
			std::cout << quatToCheck << std::endl;
			ASSERT_TRUE(false);
		}
	}
}

TEST_F(RigidBodyTransformTest, testCreateTransformWithQuaternionAndVector3d)
{
	for (int i = 0; i < nTests; i++)
	{
		Eigen::Vector3d vector = GeometryUtilitiesTestHelper::createRandomVector3d();
		Quaternion quat1 = GeometryUtilitiesTestHelper::createRandomQuaternion();

		RigidBodyTransform transform(quat1, vector);

		Quaternion quatCheck;
		Eigen::Vector3d vecCheck;

		transform.getRotation(quatCheck);
		transform.get(vecCheck);

		EXPECT_TRUE(GeometryUtilitiesTestHelper::areQuaternionsEpsilonEqual(quat1, quatCheck, 1e-5));
		EXPECT_TRUE(GeometryUtilitiesTestHelper::areVector3dsEpsilonEqual(vector, vecCheck, 1e-5));
	}
}

}