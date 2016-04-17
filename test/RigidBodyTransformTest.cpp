#include <gtest/gtest.h>
#include <eigen3/Eigen/Eigen>
#include "frl/geometry_utilities/RigidBodyTransform.hpp"
#include "GeometryUtilitiesTestHelper.hpp"

namespace geometry_utilities
{

class RigidBodyTransformTest : public ::testing::Test
{
	protected:

		virtual void SetUp()
		{
			std::srand( time(NULL) );
		}
		virtual void TearDown()
		{

		}

		int nTests = 100;
};

TEST_F(RigidBodyTransformTest, testSetRotationAndZeroTranslationWithAxisAngle)
{
	Eigen::Vector3d vector;

	for (int i = 0; i < nTests; i++)
	{
		vector << 0, 0, 0;

		for (int i = 0; i < nTests; i++)
		{
			Eigen::AngleAxis<double> axisAngle = GeometryUtilitiesTestHelper::createRandomAxisAngle();

			RigidBodyTransform transform(axisAngle, vector);

			Eigen::AngleAxis<double> axisAngleToCheck;

			transform.getRotation(axisAngleToCheck);

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
		Eigen::AngleAxis<double> axisAngle = GeometryUtilitiesTestHelper::createRandomAxisAngle();

		RigidBodyTransform transform(axisAngle, vector);

		Eigen::AngleAxis<double> axisAngleToCheck;

		transform.getRotation(axisAngleToCheck);

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
		Eigen::Quaternion<double> quat1 = GeometryUtilitiesTestHelper::createRandomQuaternion();

		RigidBodyTransform transform;
		transform.setRotationAndZeroTranslation(quat1);

		Eigen::Quaternion<double> quatToCheck;
		transform.getRotation(quatToCheck);

		if (!GeometryUtilitiesTestHelper::areQuaternionsEpsilonEqual(quat1, quatToCheck, 1e-5))
		{
			ASSERT_TRUE(false);
		}
	}
}

TEST_F(RigidBodyTransformTest, testCreateTransformWithQuaternionAndVector3d)
{
	for (int i = 0; i < nTests; i++)
	{
		Eigen::Vector3d vector = GeometryUtilitiesTestHelper::createRandomVector3d();
		Eigen::Quaternion<double> quat1 = GeometryUtilitiesTestHelper::createRandomQuaternion();

		RigidBodyTransform transform(quat1, vector);

		Eigen::Quaternion<double> quatCheck;
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
		Eigen::AngleAxis<double> a1 = GeometryUtilitiesTestHelper::createRandomAxisAngle();

		RigidBodyTransform transform(a1, vector);

		Eigen::AngleAxis<double> axisAngleCheck;
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

TEST_F(RigidBodyTransformTest, testZeroTranslation)
{
	Eigen::Matrix4d m1;
	Eigen::Vector3d translation;

	for (int i = 0; i < nTests; i++)
	{
		m1 = GeometryUtilitiesTestHelper::createRandomMatrix4d();
		RigidBodyTransform transform(m1);

		transform.zeroTranslation();

		transform.getTranslation(translation);

		ASSERT_TRUE(translation(0) == 0.0);
		ASSERT_TRUE(translation(1) == 0.0);
		ASSERT_TRUE(translation(2) == 0.0);
	}
}

TEST_F(RigidBodyTransformTest, testSetRotationWithRotationMatrix)
{
	Eigen::Matrix3d rot1;
	Eigen::Matrix3d rot2;


	for (int i = 0; i < nTests; i++)
	{
		rot1 = GeometryUtilitiesTestHelper::createRandomRotationMatrix();

		RigidBodyTransform transform(rot1);

		transform.getRotation(rot2);

		bool tmp = GeometryUtilitiesTestHelper::areMatrix3dEpsilonEqual(rot1, rot2, 10);

		EXPECT_TRUE(tmp);
	}
}

TEST_F(RigidBodyTransformTest, testSetRotationWithQuaternion)
{
	Eigen::Quaternion<double> q1;
	Eigen::Quaternion<double> q2;

	for (int i = 0; i < nTests; i++)
	{
		q1 = GeometryUtilitiesTestHelper::createRandomQuaternion();

		RigidBodyTransform transform(q1);

		transform.getRotation(q2);

		EXPECT_TRUE(GeometryUtilitiesTestHelper::areQuaternionsEpsilonEqual(q1, q2, 1e-4));
	}
}

TEST_F(RigidBodyTransformTest, testSetRotationWithAxisAngle)
{
	Eigen::AngleAxis<double> a1;
	Eigen::AngleAxis<double> a2;

	for (int i = 0; i < nTests; i++)
	{
		a1 = GeometryUtilitiesTestHelper::createRandomAxisAngle();

		RigidBodyTransform transform(a1);

		transform.getRotation(a2);

		EXPECT_TRUE(GeometryUtilitiesTestHelper::areAxisAngleEpsilonEqual(a1, a2, 1e-4));
	}
}

TEST_F(RigidBodyTransformTest, testApplyTranslation)
{
	Eigen::Vector3d vec1;
	RigidBodyTransform transform;
	Eigen::Matrix3d mat1;

	for (int i = 0; i < nTests; i++)
	{
		transform = GeometryUtilitiesTestHelper::createRandomTransformationMatrix();
		vec1 = GeometryUtilitiesTestHelper::createRandomVector3d();
		transform.getRotation(mat1);

		Eigen::Vector3d trans;
		transform.getTranslation(trans);

		transform.applyTranslation(vec1);

		Eigen::Vector3d check = mat1 * vec1 + trans;

		Eigen::Vector3d newTrans;

		transform.getTranslation(newTrans);

		EXPECT_TRUE(GeometryUtilitiesTestHelper::areVector3dsEpsilonEqual(check, newTrans, 1e-5));;
	}
}

TEST_F(RigidBodyTransformTest, testApplyRotationX)
{
	Eigen::Matrix3d rotX;

	for (int i = 0; i < nTests; i++)
	{
		rotX(0, 0) = 1;
		rotX(0, 1) = 0;
		rotX(0, 2) = 0;
		rotX(1, 0) = 0;
		rotX(2, 0) = 0;

		double angle = (2 * M_PI * rand() / RAND_MAX - M_PI);
		rotX(1, 1) = cos(angle);
		rotX(2, 2) = cos(angle);
		rotX(1, 2) = -sin(angle);
		rotX(2, 1) = sin(angle);

		RigidBodyTransform transform;

		transform.applyRotationX(angle);

		Eigen::Matrix3d rot;

		transform.getRotation(rot);

		EXPECT_TRUE(GeometryUtilitiesTestHelper::areMatrix3dEpsilonEqual(rotX, rot, 1e-5));
	}

}

TEST_F(RigidBodyTransformTest, testApplyRotationY)
{
	Eigen::Matrix3d rotY;

	for (int i = 0; i < nTests; i++)
	{
		rotY(2, 1) = 0;
		rotY(0, 1) = 0;
		rotY(1, 0) = 0;
		rotY(1, 1) = 1;
		rotY(1, 2) = 0;

		double angle = (2 * M_PI * rand() / RAND_MAX - M_PI);
		rotY(0, 0) = cos(angle);
		rotY(2, 2) = cos(angle);
		rotY(2, 0) = -sin(angle);
		rotY(0, 2) = sin(angle);

		RigidBodyTransform transform;

		transform.applyRotationY(angle);

		Eigen::Matrix3d rot;

		transform.getRotation(rot);

		EXPECT_TRUE(GeometryUtilitiesTestHelper::areMatrix3dEpsilonEqual(rotY, rot, 1e-5));
	}

}

TEST_F(RigidBodyTransformTest, testApplyRotationZ)
{
	Eigen::Matrix3d rotZ;

	for (int i = 0; i < nTests; i++)
	{
		rotZ(0, 2) = 0;
		rotZ(1, 2) = 0;
		rotZ(2, 0) = 0;
		rotZ(2, 1) = 0;
		rotZ(2, 2) = 1;

		double angle = (2 * M_PI * rand() / RAND_MAX - M_PI);
		rotZ(0, 0) = cos(angle);
		rotZ(1, 1) = cos(angle);
		rotZ(0, 1) = -sin(angle);
		rotZ(1, 0) = sin(angle);

		RigidBodyTransform transform;

		transform.applyRotationZ(angle);

		Eigen::Matrix3d rot;

		transform.getRotation(rot);

		EXPECT_TRUE(GeometryUtilitiesTestHelper::areMatrix3dEpsilonEqual(rotZ, rot, 1e-5));
	}
}

TEST_F(RigidBodyTransformTest, testMultiply1)
{
	Eigen::Matrix4d mat1;
	Eigen::Matrix4d mat2;
	Eigen::Matrix4d mat3;

	RigidBodyTransform transform1;
	RigidBodyTransform transform2;
	RigidBodyTransform transform3;


	for (int i = 0; i < nTests; i++)
	{
		mat1 = GeometryUtilitiesTestHelper::createRandomTransformationMatrix();
		mat2 = GeometryUtilitiesTestHelper::createRandomTransformationMatrix();

		transform1.set(mat1);
		transform2.set(mat2);

		transform3.multiply(transform1, transform2);
		mat3 = mat1 * mat2;

		Eigen::Matrix4d mat4;

		transform3.get(mat4);

		EXPECT_TRUE(GeometryUtilitiesTestHelper::areMatrix4dEpsilonEqual(mat4, mat3, 1e-5));
	}
}

TEST_F(RigidBodyTransformTest, testMultiply2)
{
	Eigen::Matrix4d mat1;
	Eigen::Matrix4d mat2;
	Eigen::Matrix4d mat3;

	RigidBodyTransform transform1;
	RigidBodyTransform transform2;


	for (int i = 0; i < nTests; i++)
	{
		mat1 = GeometryUtilitiesTestHelper::createRandomTransformationMatrix();
		mat2 = GeometryUtilitiesTestHelper::createRandomTransformationMatrix();

		transform1.set(mat1);
		transform2.set(mat2);

		transform1.multiply(transform2);

		mat3 = mat1 * mat2;

		Eigen::Matrix4d mat4;

		transform1.get(mat4);

		EXPECT_TRUE(GeometryUtilitiesTestHelper::areMatrix4dEpsilonEqual(mat4, mat3, 1e-5));
	}
}

TEST_F(RigidBodyTransformTest, testMultiply3)
{
	Eigen::Matrix4d mat1;
	Eigen::Matrix4d mat2;
	Eigen::Matrix4d mat3;

	RigidBodyTransform transform1;
	RigidBodyTransform transform2;
	RigidBodyTransform transform3;


	for (int i = 0; i < nTests; i++)
	{
		mat1 = GeometryUtilitiesTestHelper::createRandomTransformationMatrix();
		mat2 = GeometryUtilitiesTestHelper::createRandomTransformationMatrix();

		transform1.set(mat1);
		transform2.set(mat2);

		transform3 = transform1 * transform2;

		mat3 = mat1 * mat2;

		Eigen::Matrix4d mat4;

		transform3.get(mat4);

		EXPECT_TRUE(GeometryUtilitiesTestHelper::areMatrix4dEpsilonEqual(mat4, mat3, 1e-5));
	}
}

TEST_F(RigidBodyTransformTest, testIsRotationMatrixEpsilonIdentity)
{
	RigidBodyTransform transform1;

	EXPECT_TRUE(transform1.isRotationMatrixEpsilonIdentity(1e-10));

	Eigen::Matrix3d matrix;

	matrix << 1.1, 0, 0, 0, 1.0, 0, 0, 0, 0.9;

	transform1.setRotation(matrix);

	EXPECT_FALSE(transform1.isRotationMatrixEpsilonIdentity(1e-5));
}

TEST_F(RigidBodyTransformTest, testOverloadTimesEquals)
{
	for (int i = 0; i < nTests; i++)
	{
		Eigen::Matrix4d m1 = GeometryUtilitiesTestHelper::createRandomTransformationMatrix();
		Eigen::Matrix4d m2 = GeometryUtilitiesTestHelper::createRandomTransformationMatrix();
		RigidBodyTransform t1(m1);
		RigidBodyTransform t2(m2);
		RigidBodyTransform t3;
		RigidBodyTransform t4 = t1;

		t3.multiply(t1, t2);

		t4 *= t2;

		t4.get(m1);
		t3.get(m2);

		EXPECT_TRUE(GeometryUtilitiesTestHelper::areMatrix4dEpsilonEqual(m1, m2, 1e-5));

	}
}

TEST_F(RigidBodyTransformTest, testInvert1)
{
	for (int i = 0; i < nTests; i++)
	{
		Eigen::Matrix4d m1 = GeometryUtilitiesTestHelper::createRandomTransformationMatrix();
		Eigen::Matrix4d m2 = m1;

		RigidBodyTransform t1(m1);
		RigidBodyTransform t2(m1);

		t1.invert();

		t1.get(m1);

		EXPECT_TRUE(GeometryUtilitiesTestHelper::areMatrix4dEpsilonEqual(m1, m2.inverse() , 1e-5));
	}
}

TEST_F(RigidBodyTransformTest, testInvert2)
{
	for (int i = 0; i < nTests; i++)
	{
		Eigen::Matrix4d m1 = GeometryUtilitiesTestHelper::createRandomTransformationMatrix();
		Eigen::Matrix4d m2;

		RigidBodyTransform t1(m1);
		RigidBodyTransform t2;

		t2.invert(t1);

		t1.invert();

		t1.get(m1);
		t2.get(m2);

		EXPECT_TRUE(GeometryUtilitiesTestHelper::areMatrix4dEpsilonEqual(m1, m2 , 1e-5));
	}
}

TEST_F(RigidBodyTransformTest, testGetTranslationDifference)
{
	for (int i = 0; i < nTests; i++)
	{
		Eigen::Matrix4d m1 = GeometryUtilitiesTestHelper::createRandomTransformationMatrix();
		Eigen::Matrix4d m2 = GeometryUtilitiesTestHelper::createRandomTransformationMatrix();
		Eigen::Vector3d v1;
		Eigen::Vector3d v2;

		RigidBodyTransform t1(m1);
		RigidBodyTransform t2(m2);

		t1.getTranslation(v1);
		t2.getTranslation(v2);

		EXPECT_TRUE(GeometryUtilitiesTestHelper::areVector3dsEpsilonEqual(RigidBodyTransform::getTranslationDifference(t1, t2), v2 - v1, 1e-5));
	}
}

TEST_F(RigidBodyTransformTest, testDeterminant)
{
	for (int i = 0; i < nTests; i++)
	{
		Eigen::Matrix4d m1 = GeometryUtilitiesTestHelper::createRandomTransformationMatrix();
		Eigen::Matrix4d m2;

		RigidBodyTransform t1(m1);

		EXPECT_TRUE(fabs(t1.determinant() - m1.determinant()) < 1e-5);
	}
}

TEST_F(RigidBodyTransformTest, testTransformVector3)
{
	Eigen::Vector3d v1(1,2,3);

	RigidBodyTransform transform;
	transform.setEuler(0.0,0.0,-M_PI/2);

	transform.transform(v1);
	EXPECT_TRUE(v1(0)==2);
	EXPECT_TRUE(v1(1)-(-1)<1e-15);
	EXPECT_TRUE(v1(2)==3);
}

TEST_F(RigidBodyTransformTest, testTransformPoints1)
{
	Point3d p1(3,2,4);
	Point3d p2;

	RigidBodyTransform t1;
	t1.setEuler(-M_PI/2,0,0);

	t1.transform(p1,p2);

	EXPECT_TRUE(p2.getX()-3<1e-15);
	EXPECT_TRUE(p2.getY()-4<1e-15);
	EXPECT_TRUE(p2.getZ()-(-2)<1e-15);
}

}
