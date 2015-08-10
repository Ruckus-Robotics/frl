
#include "gtest/gtest.h"
#include "RigidBodyTransform.hpp"

TEST(RigidBodyTransformTest, testEmptyConstructor)
{
	RigidBodyTransform rigidBodyTransform;
	EXPECT_EQ(rigidBodyTransform.mat00,1.0);
	EXPECT_EQ(rigidBodyTransform.mat01,0.0);
	EXPECT_EQ(rigidBodyTransform.mat02,0.0);
	EXPECT_EQ(rigidBodyTransform.mat03,0.0);
	EXPECT_EQ(rigidBodyTransform.mat10,0.0);
	EXPECT_EQ(rigidBodyTransform.mat11,1.0);
	EXPECT_EQ(rigidBodyTransform.mat12,0.0);
	EXPECT_EQ(rigidBodyTransform.mat13,0.0);
	EXPECT_EQ(rigidBodyTransform.mat20,0.0);
	EXPECT_EQ(rigidBodyTransform.mat21,0.0);
	EXPECT_EQ(rigidBodyTransform.mat22,1.0);
	EXPECT_EQ(rigidBodyTransform.mat23,0.0);
}