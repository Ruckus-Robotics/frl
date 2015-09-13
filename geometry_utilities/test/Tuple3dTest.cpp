#include "Tuple3d.hpp"
#include "GeometryUtilitiesTestHelper.hpp"
#include <gtest/gtest.h>

using namespace geometry_utilities;

TEST(Tuple3dTest, testAdd1)
{
	for (int i = 0; i < 1000; i++)
	{
		double array1[3] = {GeometryUtilitiesTestHelper::getRandomDouble(), GeometryUtilitiesTestHelper::getRandomDouble(), GeometryUtilitiesTestHelper::getRandomDouble()};
		double array2[3] = {GeometryUtilitiesTestHelper::getRandomDouble(), GeometryUtilitiesTestHelper::getRandomDouble(), GeometryUtilitiesTestHelper::getRandomDouble()};

		Tuple3d tuple1(array1);
		Tuple3d tuple2(array2);

		Tuple3d tuple3(array1[0] + array2[0], array1[1] + array2[1], array1[2] + array2[2]);

		tuple1.add(tuple2);

		EXPECT_TRUE(tuple1.epsilonEquals(tuple3, 1e-12));
	}
}

TEST(Tuple3dTest, testAdd2)
{
	Tuple3d tuple1;
	Tuple3d tuple2;

	for (int i = 0 ; i < 1000; i++)
	{
		tuple1.set(GeometryUtilitiesTestHelper::getRandomDouble(), GeometryUtilitiesTestHelper::getRandomDouble(), GeometryUtilitiesTestHelper::getRandomDouble());

		std::vector<double> vector(3);
		vector[0] = GeometryUtilitiesTestHelper::getRandomDouble();
		vector[1] = GeometryUtilitiesTestHelper::getRandomDouble();
		vector[2] = GeometryUtilitiesTestHelper::getRandomDouble();

		tuple2.set(vector);

		Tuple3d tuple3;
		tuple3.set(vector[0] + tuple1.getX(), vector[1] + tuple1.getY(), vector[2] + tuple1.getZ());

		tuple2.add(tuple1);

		EXPECT_TRUE(tuple2.epsilonEquals(tuple3, 1e-12));
	}
}

TEST(Tuple3dTest, testAdd3)
{
	for (int i = 0; i < 1000; i++)
	{
		Tuple3d tuple1 = GeometryUtilitiesTestHelper::getRandomTuple3d();
		Tuple3d tuple2 = GeometryUtilitiesTestHelper::getRandomTuple3d();
		Tuple3d tuple3;
		tuple3.add(tuple1, tuple2);

		tuple1.add(tuple2);

		EXPECT_TRUE(tuple3.epsilonEquals(tuple1, 1e-12));
	}
}

TEST(Tuple3dTest, testSubtract1)
{
	for (int i = 0; i < 1000; i++)
	{
		Tuple3d tuple1 = GeometryUtilitiesTestHelper::getRandomTuple3d();
		Tuple3d tuple2 = GeometryUtilitiesTestHelper::getRandomTuple3d();
		Tuple3d tuple3(tuple1.getX() - tuple2.getX(), tuple1.getY() - tuple2.getY(), tuple1.getZ() - tuple2.getZ());
		tuple1.subtract(tuple2);

		EXPECT_TRUE(tuple3.epsilonEquals(tuple1, 1e-12));
	}
}

TEST(Tuple3dTest, testSubtract2)
{
	for (int i = 0; i < 1000; i++)
	{
		Tuple3d tuple1 = GeometryUtilitiesTestHelper::getRandomTuple3d();
		Tuple3d tuple2 = GeometryUtilitiesTestHelper::getRandomTuple3d();
		Tuple3d tuple3(tuple1.getX() - tuple2.getX(), tuple1.getY() - tuple2.getY(), tuple1.getZ() - tuple2.getZ());
		tuple1.subtract(tuple2.getX(), tuple2.getY(), tuple2.getZ());

		EXPECT_TRUE(tuple3.epsilonEquals(tuple1, 1e-12));
	}
}

TEST(Tuple3dTest, testSubtract3)
{
	for (int i = 0; i < 1000; i++)
	{
		Tuple3d tuple1 = GeometryUtilitiesTestHelper::getRandomTuple3d();
		Tuple3d tuple2 = GeometryUtilitiesTestHelper::getRandomTuple3d();
		Tuple3d tuple3;
		tuple3.subtract(tuple1, tuple2);

		tuple1.subtract(tuple2);

		EXPECT_TRUE(tuple3.epsilonEquals(tuple1, 1e-12));
	}
}

TEST(Tuple3dTest, testNegate1)
{
	for (int i = 0; i < 1000; i++)
	{
		Tuple3d tuple1 = GeometryUtilitiesTestHelper::getRandomTuple3d();
		Tuple3d tuple2 = tuple1;
		tuple1.negate();

		EXPECT_TRUE(tuple1.getX() == -tuple2.getX());
		EXPECT_TRUE(tuple1.getY() == -tuple2.getY());
		EXPECT_TRUE(tuple1.getZ() == -tuple2.getZ());
	}
}

TEST(Tuple3dTest, testNegate2)
{
	for (int i = 0 ; i < 1000; i++)
	{
		Tuple3d tuple1 = GeometryUtilitiesTestHelper::getRandomTuple3d();
		Tuple3d tuple2;

		tuple2.negate(tuple1);

		EXPECT_TRUE(tuple1.getX() == -tuple2.getX());
		EXPECT_TRUE(tuple1.getY() == -tuple2.getY());
		EXPECT_TRUE(tuple1.getZ() == -tuple2.getZ());
	}
}