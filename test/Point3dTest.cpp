#include "geometry_utilities/Point3d.hpp"
#include "GeometryUtilitiesTestHelper.hpp"
#include <gtest/gtest.h>
#include <math.h>
#include <algorithm>

namespace geometry_utilities
{

class Point3dTest : public ::testing::Test
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

TEST_F(Point3dTest, testDistanceSquared)
{
	for (int i = 0; i < nTests; i++)
	{
		double x = GeometryUtilitiesTestHelper::getRandomDouble();
		double y = GeometryUtilitiesTestHelper::getRandomDouble();
		double z = GeometryUtilitiesTestHelper::getRandomDouble();

		Point3d point(x, y, z);

		double x2 = GeometryUtilitiesTestHelper::getRandomDouble();
		double y2 = GeometryUtilitiesTestHelper::getRandomDouble();
		double z2 = GeometryUtilitiesTestHelper::getRandomDouble();

		Point3d point2(x2, y2, z2);

		double distanceSquared = pow((x2 - x), 2) + pow((y2 - y), 2) + pow((z2 - z), 2);

		EXPECT_TRUE(distanceSquared - point.distanceSquared(point2) < 1e-8);
	}
}

TEST_F(Point3dTest, testDistance)
{
	for (int i = 0; i < nTests; i++)
	{
		double x = GeometryUtilitiesTestHelper::getRandomDouble();
		double y = GeometryUtilitiesTestHelper::getRandomDouble();
		double z = GeometryUtilitiesTestHelper::getRandomDouble();

		Point3d point(x, y, z);

		double x2 = GeometryUtilitiesTestHelper::getRandomDouble();
		double y2 = GeometryUtilitiesTestHelper::getRandomDouble();
		double z2 = GeometryUtilitiesTestHelper::getRandomDouble();

		Point3d point2(x2, y2, z2);

		double distance = sqrt(pow((x2 - x), 2) + pow((y2 - y), 2) + pow((z2 - z), 2));

		EXPECT_TRUE(distance - point.distance(point2) < 1e-8);
	}
}

TEST_F(Point3dTest, testDistanceL1)
{
	for (int i = 0; i < nTests; i++)
	{
		double x = GeometryUtilitiesTestHelper::getRandomDouble();
		double y = GeometryUtilitiesTestHelper::getRandomDouble();
		double z = GeometryUtilitiesTestHelper::getRandomDouble();

		Point3d point(x, y, z);

		double x2 = GeometryUtilitiesTestHelper::getRandomDouble();
		double y2 = GeometryUtilitiesTestHelper::getRandomDouble();
		double z2 = GeometryUtilitiesTestHelper::getRandomDouble();

		Point3d point2(x2, y2, z2);

		double distanceL1 = fabs(x2 - x) + fabs(y2 - y) + fabs(z2 - z);

		EXPECT_TRUE(distanceL1 - point.distanceL1(point2) < 1e-8);
	}
}

TEST_F(Point3dTest, testDistanceLInf)
{
	for (int i = 0; i < nTests; i++)
	{
		double x = GeometryUtilitiesTestHelper::getRandomDouble();
		double y = GeometryUtilitiesTestHelper::getRandomDouble();
		double z = GeometryUtilitiesTestHelper::getRandomDouble();

		Point3d point(x, y, z);

		double x2 = GeometryUtilitiesTestHelper::getRandomDouble();
		double y2 = GeometryUtilitiesTestHelper::getRandomDouble();
		double z2 = GeometryUtilitiesTestHelper::getRandomDouble();

		Point3d point2(x2, y2, z2);

		double distanceLInf = std::max(fabs(x2 - x), fabs(y2 - y));
		distanceLInf = std::max(distanceLInf, fabs(z2 - z));

		EXPECT_TRUE(distanceLInf - point.distanceLinf(point2) < 1e-8);
	}
}

}