#include <gtest/gtest.h>
#include <geometry_utilities/AxisAngle.hpp>
#include <eigen3/Eigen/Eigen>
#include <geometry_utilities/RigidBodyTransform.hpp>
#include "GeometryUtilitiesTestHelper.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_utilities/Quaternion.hpp"

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

TEST_F(RigidBodyTransformTest, testCreateTransformWithAxisAngleAndVector3d)
{
	for (int i = 0; i < nTests; i++)
	{
		Eigen::Vector3d vector = GeometryUtilitiesTestHelper::createRandomVector3d();
		AxisAngle a1 = GeometryUtilitiesTestHelper::createRandomAxisAngle();

		RigidBodyTransform transform(a1, vector);

		AxisAngle axisAngleCheck;
		Eigen::Vector3d vecCheck;

		transform.getRotation(axisAngleCheck);
		transform.get(vecCheck);

		EXPECT_TRUE(GeometryUtilitiesTestHelper::areAxisAngleEpsilonEqual(a1, axisAngleCheck, 1e-5));
		EXPECT_TRUE(GeometryUtilitiesTestHelper::areVector3dsEpsilonEqual(vector, vecCheck, 1e-5));
	}
}

TEST_F(RigidBodyTransformTest, testSetWithEuler)
{
	Eigen::Vector3d rpy;
	for (int i = 0; i < nTests; i++)
	{
		rpy(0) = 2 * M_PI * rand() / RAND_MAX - M_PI;
		rpy(1) = 2 * (M_PI / 2 - 0.01) * rand() / RAND_MAX - (M_PI / 2  - 0.01);
		rpy(2) = 2 * M_PI * rand() / RAND_MAX - M_PI;

		RigidBodyTransform transform;
		transform.setEuler(rpy);

		Eigen::Vector3d rpyCheck;
		transform.getEulerXYZ(rpyCheck);

		EXPECT_TRUE(fabs(rpy(0) - rpyCheck(0)) < 1e-5);
		EXPECT_TRUE(fabs(rpy(1) - rpyCheck(1)) < 1e-5);
		EXPECT_TRUE(fabs(rpy(2) - rpyCheck(2)) < 1e-5);
	}
}

TEST_F(RigidBodyTransformTest, testSetAsTranspose)
{
	Eigen::Matrix4d m1 = GeometryUtilitiesTestHelper::createRandomMatrix4d();

	RigidBodyTransform transform;
	transform.setAsTranspose(m1);

	m1.transpose();

	RigidBodyTransform transform2;
	transform2.setAsTranspose(m1);

	transform.get(m1);

	Eigen::Matrix4d m2;

	transform2.get(m2);

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			EXPECT_TRUE(fabs(m1(i, j) - m2(i, j)) < 1e-5);
		}
	}
}

}