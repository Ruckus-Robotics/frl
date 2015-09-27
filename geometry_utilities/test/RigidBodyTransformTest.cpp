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
		Quaternion quat1(0.4219074509830524, 0.16571163801219513, 0.8514918861954333, 0.26361965703574125);

		RigidBodyTransform transform;
		transform.setRotationAndZeroTranslation(quat1);

		Quaternion quatToCheck;
		transform.getRotation(quatToCheck);

		ASSERT_TRUE(fabs(quatToCheck.getX() - quat1.getX()) < 1e-5);
		ASSERT_TRUE(fabs(quatToCheck.getY() - quat1.getY()) < 1e-5);
		ASSERT_TRUE(fabs(quatToCheck.getZ() - quat1.getZ()) < 1e-5);
		ASSERT_TRUE(fabs(quatToCheck.getW() - quat1.getW()) < 1e-5);
	}
}

}