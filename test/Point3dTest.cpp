#include "frl/geometry/Point3d.hpp"
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
            std::srand( time(NULL) );
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

    TEST_F(Point3dTest, testAdd1)
    {
        for (int i = 0; i < 1000; i++)
        {
            double array1[3] = {GeometryUtilitiesTestHelper::getRandomDouble(), GeometryUtilitiesTestHelper::getRandomDouble(), GeometryUtilitiesTestHelper::getRandomDouble()};
            double array2[3] = {GeometryUtilitiesTestHelper::getRandomDouble(), GeometryUtilitiesTestHelper::getRandomDouble(), GeometryUtilitiesTestHelper::getRandomDouble()};

            Point3d point1(array1);
            Point3d point2(array2);

            Point3d point3(array1[0] + array2[0], array1[1] + array2[1], array1[2] + array2[2]);

            point1 += point2;

            EXPECT_TRUE(point1.epsilonEquals(point3, 1e-12));
        }
    }

    TEST_F(Point3dTest, testAdd2)
    {
        Point3d point1;
        Point3d point2;

        for (int i = 0; i < 1000; i++)
        {
            point1.set(GeometryUtilitiesTestHelper::getRandomDouble(), GeometryUtilitiesTestHelper::getRandomDouble(), GeometryUtilitiesTestHelper::getRandomDouble());

            std::vector<double> vector(3);
            vector[0] = GeometryUtilitiesTestHelper::getRandomDouble();
            vector[1] = GeometryUtilitiesTestHelper::getRandomDouble();
            vector[2] = GeometryUtilitiesTestHelper::getRandomDouble();

            point2.set(vector);

            Point3d point3;
            point3.set(vector[0] + point1.getX(), vector[1] + point1.getY(), vector[2] + point1.getZ());

            point2 = point2 + point1;

            EXPECT_TRUE(point2.epsilonEquals(point3, 1e-12));
        }
    }

    TEST_F(Point3dTest, testAdd3)
    {
        for (int i = 0; i < 1000; i++)
        {
            Point3d point1 = GeometryUtilitiesTestHelper::getRandomPoint3d();
            Point3d point2 = GeometryUtilitiesTestHelper::getRandomPoint3d();
            Point3d point3(point1.getX() + point2.getX(), point1.getY() + point2.getY(), point1.getZ() + point2.getZ());
            point1.add(point2.getX(), point2.getY(), point2.getZ());

            EXPECT_TRUE(point3.epsilonEquals(point1, 1e-12));
        }
    }

    TEST_F(Point3dTest, testSubtract1)
    {
        for (int i = 0; i < 1000; i++)
        {
            Point3d point1 = GeometryUtilitiesTestHelper::getRandomPoint3d();
            Point3d point2 = GeometryUtilitiesTestHelper::getRandomPoint3d();
            Point3d point3(point1.getX() - point2.getX(), point1.getY() - point2.getY(), point1.getZ() - point2.getZ());
            point1 -= point2;

            EXPECT_TRUE(point3.epsilonEquals(point1, 1e-12));
        }
    }

    TEST_F(Point3dTest, testSubtract2)
    {
        for (int i = 0; i < 1000; i++)
        {
            Point3d point1 = GeometryUtilitiesTestHelper::getRandomPoint3d();
            Point3d point2 = GeometryUtilitiesTestHelper::getRandomPoint3d();
            Point3d point3(point1.getX() - point2.getX(), point1.getY() - point2.getY(), point1.getZ() - point2.getZ());
            point1.subtract(point2.getX(), point2.getY(), point2.getZ());

            EXPECT_TRUE(point3.epsilonEquals(point1, 1e-12));
        }
    }

    TEST_F(Point3dTest, testNegate1)
    {
        for (int i = 0; i < 1000; i++)
        {
            Point3d point1 = GeometryUtilitiesTestHelper::getRandomPoint3d();
            Point3d point2 = point1;
            point1.negate();

            EXPECT_TRUE(point1.getX() == -point2.getX());
            EXPECT_TRUE(point1.getY() == -point2.getY());
            EXPECT_TRUE(point1.getZ() == -point2.getZ());
        }
    }

    TEST_F(Point3dTest, testScale1)
    {
        for (int i = 0; i < 1000; i++)
        {
            Point3d point1 = GeometryUtilitiesTestHelper::getRandomPoint3d();
            Point3d point2 = point1;
            double scale = rand() % 100 - 50;

            point1.scale(scale);

            EXPECT_TRUE(point1.getX() == point2.getX() * scale);
            EXPECT_TRUE(point1.getY() == point2.getY() * scale);
            EXPECT_TRUE(point1.getZ() == point2.getZ() * scale);
        }
    }

    TEST_F(Point3dTest, testScaleAdd1)
    {
        for (int i = 0; i < 1000; i++)
        {
            Point3d point1 = GeometryUtilitiesTestHelper::getRandomPoint3d();
            Point3d point2 = GeometryUtilitiesTestHelper::getRandomPoint3d();
            Point3d point3 = point1;
            double scale = rand() % 100 - 50;

            point1.scaleAdd(scale, point2);

            point3.scale(scale);
            point3 += point2;

            EXPECT_TRUE(point1.equals(point3));
        }
    }

    TEST_F(Point3dTest, testAbsoluteValue1)
    {
        for (int i = 0; i < 1000; i++)
        {
            Point3d point1 = GeometryUtilitiesTestHelper::getRandomPoint3d();
            Point3d point2 = point1;
            point1.absoluteValue();

            EXPECT_TRUE(point1.getX() == fabs(point2.getX()));
            EXPECT_TRUE(point1.getY() == fabs(point2.getY()));
            EXPECT_TRUE(point1.getZ() == fabs(point2.getZ()));
        }
    }

    TEST_F(Point3dTest, testClampMinMax1)
    {
        for (int i = 0; i < 1000; i++)
        {
            Point3d point1(100, 200, 300);
            Point3d point2 = point1;
            point2.clampMinMax(-100, -50);

            EXPECT_TRUE(point2.getX() == -50);
            EXPECT_TRUE(point2.getY() == -50);
            EXPECT_TRUE(point2.getZ() == -50);
        }
    }

    TEST_F(Point3dTest, testClampMinMax2)
    {
        for (int i = 0; i < 1000; i++)
        {
            Point3d point1(100, 200, 300);
            Point3d point2 = point1;
            point2.clampMinMax(-100, 200);

            EXPECT_TRUE(point2.getX() == 100);
            EXPECT_TRUE(point2.getY() == 200);
            EXPECT_TRUE(point2.getZ() == 200);
        }
    }

    TEST_F(Point3dTest, testClampMinMax3)
    {
        for (int i = 0; i < 1000; i++)
        {
            Point3d point1(100, 200, 300);
            Point3d point2 = point1;
            point2.clampMinMax(201, 220);

            EXPECT_TRUE(point2.getX() == 201);
            EXPECT_TRUE(point2.getY() == 201);
            EXPECT_TRUE(point2.getZ() == 220);
        }
    }

    TEST_F(Point3dTest, testClamMin)
    {
        for (int i = 0; i < 1000; i++)
        {
            Point3d point1(100, 200, 300);
            Point3d point2 = point1;
            point2.clampMin(201);

            EXPECT_TRUE(point2.getX() == 201);
            EXPECT_TRUE(point2.getY() == 201);
            EXPECT_TRUE(point2.getZ() == 300);
        }
    }

    TEST_F(Point3dTest, testClamMax)
    {
        for (int i = 0; i < 1000; i++)
        {
            Point3d point1(100, 200, 300);
            Point3d point2 = point1;
            point2.clampMax(201);

            EXPECT_TRUE(point2.getX() == 100);
            EXPECT_TRUE(point2.getY() == 200);
            EXPECT_TRUE(point2.getZ() == 201);
        }
    }

}