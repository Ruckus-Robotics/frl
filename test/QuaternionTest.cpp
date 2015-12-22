#include "frl/geometry_utilities/Quaternion.hpp"
#include "frl/geometry_utilities/RigidBodyTransform.hpp"
#include "GeometryUtilitiesTestHelper.hpp"
#include <gtest/gtest.h>
#include <math.h>

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
	for (int i = 0; i < nTests; i++)
	{
		Eigen::Matrix3d rot1 = GeometryUtilitiesTestHelper::createRandomRotationMatrix();
		Eigen::Matrix4d trans1 = GeometryUtilitiesTestHelper::createRandomTransformationMatrix();

		Quaternion q1(rot1);
		Quaternion q2(trans1);

		Eigen::Vector4d v1;
		Eigen::Vector4d v2;

		q1.get(v1);
		q2.get(v2);

		EXPECT_TRUE(GeometryUtilitiesTestHelper::areVector4dsEpsilonEqual(v1, v2, 1e-4));
	}
}

TEST_F(QuaternionTest, testConjugate)
{
	for (int i = 0; i < nTests; i++)
	{
		Quaternion q1 = GeometryUtilitiesTestHelper::createRandomQuaternion();

		Quaternion q2(q1);

		q2.conjugate();

		EXPECT_TRUE(q1.getX() + q2.getX() < 1e-8);
		EXPECT_TRUE(q1.getY() + q2.getY() < 1e-8);
		EXPECT_TRUE(q1.getZ() + q2.getZ() < 1e-8);
		EXPECT_TRUE(q1.getW() - q2.getW() < 1e-8);
	}
}

TEST_F(QuaternionTest, testConjugate2)
{
	for (int i = 0; i < nTests; i++)
	{
		Quaternion q1 = GeometryUtilitiesTestHelper::createRandomQuaternion();

		Quaternion q2;


		q1.conjugate(q2);

		q2.conjugate();

		EXPECT_TRUE(q1.getX() + q2.getX() < 1e-8);
		EXPECT_TRUE(q1.getY() + q2.getY() < 1e-8);
		EXPECT_TRUE(q1.getZ() + q2.getZ() < 1e-8);
		EXPECT_TRUE(q1.getW() - q2.getW() < 1e-8);
	}
}

TEST_F(QuaternionTest, testMultiply1)
{
	for (int i = 0; i < nTests; i++)
	{
		Quaternion q1 = GeometryUtilitiesTestHelper::createRandomQuaternion();
		Quaternion q2 = GeometryUtilitiesTestHelper::createRandomQuaternion();

		Quaternion q3;

		double x = q1.getW() * q2.getX() + q1.getX() * q2.getW() + q1.getY() * q2.getZ() - q1.getZ() * q2.getY();
		double y = q1.getW() * q2.getY() - q1.getX() * q2.getZ() + q1.getY() * q2.getW() + q1.getZ() * q2.getX();
		double z = q1.getW() * q2.getZ() + q1.getX() * q2.getY() - q1.getY() * q2.getX() + q1.getZ() * q2.getW();
		double w = q1.getW() * q2.getW() - q1.getX() * q2.getX() - q1.getY() * q2.getY() - q1.getZ() * q2.getZ();

		q1.multiply(q2);

		q3.set(x, y, z, w);

		EXPECT_TRUE(GeometryUtilitiesTestHelper::areQuaternionsEpsilonEqual(q1, q3, 1e-8));
	}
}

TEST_F(QuaternionTest, testMultiply2)
{
	for (int i = 0; i < nTests; i++)
	{
		Quaternion q1 = GeometryUtilitiesTestHelper::createRandomQuaternion();
		Quaternion q2 = GeometryUtilitiesTestHelper::createRandomQuaternion();

		Quaternion q3;

		q3.multiply(q1, q2);

		double x = q1.getW() * q2.getX() + q1.getX() * q2.getW() + q1.getY() * q2.getZ() - q1.getZ() * q2.getY();
		double y = q1.getW() * q2.getY() - q1.getX() * q2.getZ() + q1.getY() * q2.getW() + q1.getZ() * q2.getX();
		double z = q1.getW() * q2.getZ() + q1.getX() * q2.getY() - q1.getY() * q2.getX() + q1.getZ() * q2.getW();
		double w = q1.getW() * q2.getW() - q1.getX() * q2.getX() - q1.getY() * q2.getY() - q1.getZ() * q2.getZ();

		Quaternion q4;
		q4.set(x, y, z, w);

		EXPECT_TRUE(GeometryUtilitiesTestHelper::areQuaternionsEpsilonEqual(q4, q3, 1e-8));
	}
}

TEST_F(QuaternionTest, testMultiply3)
{
	for (int i = 0; i < nTests; i++)
	{
		Quaternion q1 = GeometryUtilitiesTestHelper::createRandomQuaternion();
		Quaternion q2 = GeometryUtilitiesTestHelper::createRandomQuaternion();

		Quaternion q3;

		q3 = q1 * q2;
		q1.multiply(q2);

		EXPECT_TRUE(GeometryUtilitiesTestHelper::areQuaternionsEpsilonEqual(q1, q3, 1e-8));
	}
}

TEST_F(QuaternionTest, testMultiplyEquals)
{
	for (int i = 0; i < nTests; i++)
	{
		Quaternion q1 = GeometryUtilitiesTestHelper::createRandomQuaternion();
		Quaternion q2 = GeometryUtilitiesTestHelper::createRandomQuaternion();

		Quaternion q3;

		q3 = q1;

		q3 *= q2;
		q1.multiply(q2);

		EXPECT_TRUE(GeometryUtilitiesTestHelper::areQuaternionsEpsilonEqual(q1, q3, 1e-8));
	}
}

TEST_F(QuaternionTest, testInverse1)
{
	for (int i = 0; i < nTests; i++)
	{
		Quaternion q1 = GeometryUtilitiesTestHelper::createRandomQuaternion();
		Quaternion q2(q1);
		double q1Norm = sqrt(q1.getX() * q1.getX() + q1.getY() * q1.getY() + q1.getZ() * q1.getZ() + q1.getW() * q1.getW());

		q1.inverse();

		EXPECT_TRUE((q1.getX() + q2.getX() / q1Norm) - 1e-8);
		EXPECT_TRUE((q1.getY() + q2.getY() / q1Norm) - 1e-8);
		EXPECT_TRUE((q1.getZ() + q2.getZ() / q1Norm) - 1e-8);
		EXPECT_TRUE((q1.getW() - q2.getW() / q1Norm) - 1e-8);
	}
}

TEST_F(QuaternionTest, testInverse2)
{
	for (int i = 0; i < nTests; i++)
	{
		Quaternion q1 = GeometryUtilitiesTestHelper::createRandomQuaternion();
		Quaternion q2;
		Quaternion q3(q1);
		double q1Norm = sqrt(q1.getX() * q1.getX() + q1.getY() * q1.getY() + q1.getZ() * q1.getZ() + q1.getW() * q1.getW());

		q2.inverse(q1);

		EXPECT_TRUE((q2.getX() + q3.getX() / q1Norm) - 1e-8);
		EXPECT_TRUE((q2.getY() + q3.getY() / q1Norm) - 1e-8);
		EXPECT_TRUE((q2.getZ() + q3.getZ() / q1Norm) - 1e-8);
		EXPECT_TRUE((q2.getW() - q3.getW() / q1Norm) - 1e-8);
	}
}

TEST_F(QuaternionTest, testNormalize)
{
	for(int i = 0; i<nTests; i++)
	{
		double x = GeometryUtilitiesTestHelper::getRandomDouble();
		double y = GeometryUtilitiesTestHelper::getRandomDouble();
		double z = GeometryUtilitiesTestHelper::getRandomDouble();
		double w = GeometryUtilitiesTestHelper::getRandomDouble();

		Quaternion q1(x,y,z,w);

		q1.normalize();

		double norm = sqrt(pow(q1.getX(),2) + pow(q1.getY(),2) + pow(q1.getZ(),2) + pow(q1.getW(),2));

		EXPECT_TRUE((norm-1)<1e-8);
	}
}

TEST_F(QuaternionTest, testGetAndSetWithRotationMatrix)
{

	for(int i = 0; i<nTests; i++)
	{
		Quaternion q1 = GeometryUtilitiesTestHelper::createRandomQuaternion();
	
		RigidBodyTransform t1;
		t1.setRotation(q1);

		Eigen::Matrix3d m1;
		t1.getRotation(m1);

		Quaternion q2(m1);
	
		EXPECT_TRUE(GeometryUtilitiesTestHelper::areQuaternionsEpsilonEqual(q1,q2,1e-4));

		Eigen::Matrix3d m2 = q2.getAsMatrix3d();

		GeometryUtilitiesTestHelper::areMatrix3dEpsilonEqual(m1,m2,1e-4);
	}
}

}