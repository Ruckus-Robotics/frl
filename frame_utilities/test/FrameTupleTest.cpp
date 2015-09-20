#include "FrameTuple.hpp"
#include "ReferenceFrameTestHelper.hpp"
#include "FrameUtilitiesTestHelper.hpp"
#include <gtest/gtest.h>

using namespace frame_utilities;

class FrameTupleTest : public ::testing::Test
{
	protected:

		virtual void SetUp()
		{
			allFrames.push_back(root.get());
			allFrames.push_back(frame1.get());
			allFrames.push_back(frame2.get());
			allFrames.push_back(frame3.get());
		}
		virtual void TearDown()
		{
			allFrames.clear();
		}

		std::unique_ptr<ReferenceFrame> root = ReferenceFrame::createARootFrame("root");

		std::shared_ptr<RandomUnchangingFrame> frame1 = RandomUnchangingFrame::create("frame1", root.get());
		std::shared_ptr<RandomUnchangingFrame> frame2 = RandomUnchangingFrame::create("frame2", frame1.get());
		std::shared_ptr<RandomUnchangingFrame> frame3 = RandomUnchangingFrame::create("frame3", frame2.get());

		std::vector<ReferenceFrame*> allFrames;

		int nTests = 1000;
};

typedef FrameTupleTest FrameTupleTest_DeathTest;

TEST_F(FrameTupleTest_DeathTest, testFrameMismatch1)
{
	for (int i = 0; i < 1000; i++)
	{
		double array1[3] = {FrameUtilitiesTestHelper::getRandomDouble(), FrameUtilitiesTestHelper::getRandomDouble(), FrameUtilitiesTestHelper::getRandomDouble()};
		double array2[3] = {FrameUtilitiesTestHelper::getRandomDouble(), FrameUtilitiesTestHelper::getRandomDouble(), FrameUtilitiesTestHelper::getRandomDouble()};

		FrameTuple tuple1("tuple1", frame1.get(), array1);
		FrameTuple tuple2("tuple2", frame2.get(), array2);

		// tuple1.add(tuple2);
		try
		{
			//This line should throw an exception.
			tuple1.add(tuple2);
			ASSERT_TRUE(false);
		}
		catch ( ... )
		{
			ASSERT_TRUE(true);
		}
	}
}

TEST_F(FrameTupleTest, testGetName)
{
	FrameTuple tuple1;
	ASSERT_TRUE(tuple1.getName() == "");

	FrameTuple tuple2("Yippy", nullptr);
	ASSERT_TRUE(tuple2.getName() == "Yippy");
}

TEST_F(FrameTupleTest, testAdd1)
{

	for (int i = 0; i < 1000; i++)
	{
		double array1[3] = {FrameUtilitiesTestHelper::getRandomDouble(), FrameUtilitiesTestHelper::getRandomDouble(), FrameUtilitiesTestHelper::getRandomDouble()};
		double array2[3] = {FrameUtilitiesTestHelper::getRandomDouble(), FrameUtilitiesTestHelper::getRandomDouble(), FrameUtilitiesTestHelper::getRandomDouble()};

		FrameTuple tuple1("tuple1", frame1.get(), array1);
		FrameTuple tuple2("tuple2", frame1.get(), array2);
		FrameTuple tuple3("tuple3", frame1.get(), array1[0] + array2[0], array1[1] + array2[1], array1[2] + array2[2]);

		tuple1.add(tuple2);

		EXPECT_TRUE(tuple1.epsilonEquals(tuple3, 1e-12));
	}
}

TEST_F(FrameTupleTest, testAdd2)
{
	FrameTuple tuple1("tuple1", frame2.get());
	FrameTuple tuple2("tuple2", frame2.get());

	for (int i = 0 ; i < 1000; i++)
	{
		tuple1.set(FrameUtilitiesTestHelper::getRandomDouble(), FrameUtilitiesTestHelper::getRandomDouble(), FrameUtilitiesTestHelper::getRandomDouble());

		std::vector<double> vector(3);
		vector[0] = FrameUtilitiesTestHelper::getRandomDouble();
		vector[1] = FrameUtilitiesTestHelper::getRandomDouble();
		vector[2] = FrameUtilitiesTestHelper::getRandomDouble();

		tuple2.set(vector);

		FrameTuple tuple3("tuple3", frame2.get());
		tuple3.set(vector[0] + tuple1.getX(), vector[1] + tuple1.getY(), vector[2] + tuple1.getZ());

		tuple2.add(tuple1);

		EXPECT_TRUE(tuple2.epsilonEquals(tuple3, 1e-12));
	}
}

TEST_F(FrameTupleTest, testAdd3)
{
	for (int i = 0; i < 1000; i++)
	{
		FrameTuple tuple1 = FrameUtilitiesTestHelper::getRandomFrameTuple("tuple1", frame3.get());
		FrameTuple tuple2 = FrameUtilitiesTestHelper::getRandomFrameTuple("tuple2", frame3.get());
		FrameTuple tuple3;
		tuple3.add(tuple1, tuple2);

		tuple1.add(tuple2);

		EXPECT_TRUE(tuple3.epsilonEquals(tuple1, 1e-12));
	}
}

TEST_F(FrameTupleTest, testSubtract1)
{
	for (int i = 0; i < 1000; i++)
	{
		FrameTuple tuple1 = FrameUtilitiesTestHelper::getRandomFrameTuple("tuple1", frame1.get());
		FrameTuple tuple2 = FrameUtilitiesTestHelper::getRandomFrameTuple("tuple2", frame1.get());
		FrameTuple tuple3("tuple3", frame1.get(), tuple1.getX() - tuple2.getX(), tuple1.getY() - tuple2.getY(), tuple1.getZ() - tuple2.getZ());
		tuple1.subtract(tuple2);

		EXPECT_TRUE(tuple3.epsilonEquals(tuple1, 1e-12));
	}
}

TEST_F(FrameTupleTest, testSubtract2)
{
	for (int i = 0; i < 1000; i++)
	{
		FrameTuple tuple1 = FrameUtilitiesTestHelper::getRandomFrameTuple("tuple1", frame2.get());
		FrameTuple tuple2 = FrameUtilitiesTestHelper::getRandomFrameTuple("tuple2", frame2.get());
		FrameTuple tuple3;
		tuple3.subtract(tuple1, tuple2);

		tuple1.subtract(tuple2);

		EXPECT_TRUE(tuple3.epsilonEquals(tuple1, 1e-12));
	}
}

TEST_F(FrameTupleTest, testNegate1)
{
	for (int i = 0; i < 1000; i++)
	{
		FrameTuple tuple1 = FrameUtilitiesTestHelper::getRandomFrameTuple("tuple1", frame1.get());
		FrameTuple tuple2 = tuple1;
		tuple1.negate();

		EXPECT_TRUE(tuple1.getX() == -tuple2.getX());
		EXPECT_TRUE(tuple1.getY() == -tuple2.getY());
		EXPECT_TRUE(tuple1.getZ() == -tuple2.getZ());
	}
}

TEST_F(FrameTupleTest, testNegate2)
{
	for (int i = 0 ; i < 1000; i++)
	{
		FrameTuple tuple1 = FrameUtilitiesTestHelper::getRandomFrameTuple("tuple1", root.get());
		FrameTuple tuple2;

		tuple2.negate(tuple1);

		EXPECT_TRUE(tuple1.getX() == -tuple2.getX());
		EXPECT_TRUE(tuple1.getY() == -tuple2.getY());
		EXPECT_TRUE(tuple1.getZ() == -tuple2.getZ());
	}
}

TEST_F(FrameTupleTest, testScale1)
{
	for (int i = 0; i < 1000; i++)
	{
		FrameTuple tuple1 = FrameUtilitiesTestHelper::getRandomFrameTuple("tuple1", frame1.get());
		FrameTuple tuple2 = tuple1;
		double scale = rand() % 100 - 50;

		tuple1.scale(scale);

		EXPECT_TRUE(tuple1.getX() == tuple2.getX()*scale);
		EXPECT_TRUE(tuple1.getY() == tuple2.getY()*scale);
		EXPECT_TRUE(tuple1.getZ() == tuple2.getZ()*scale);
	}
}

TEST_F(FrameTupleTest, testScale2)
{
	for (int i = 0; i < 1000; i++)
	{
		FrameTuple tuple1 = FrameUtilitiesTestHelper::getRandomFrameTuple("tuple1", frame2.get());
		double scale = rand() % 100 - 50;
		FrameTuple tuple2;

		tuple2.scale(scale, tuple1);

		EXPECT_TRUE(tuple2.getX() == tuple1.getX()*scale);
		EXPECT_TRUE(tuple2.getY() == tuple1.getY()*scale);
		EXPECT_TRUE(tuple2.getZ() == tuple1.getZ()*scale);
	}
}

TEST_F(FrameTupleTest, testScaleAdd1)
{
	for (int i = 0; i < 1000; i++)
	{
		FrameTuple tuple1 = FrameUtilitiesTestHelper::getRandomFrameTuple("tuple1", root.get());
		FrameTuple tuple2 = FrameUtilitiesTestHelper::getRandomFrameTuple("tuple2", root.get());
		FrameTuple tuple3 = tuple1;
		double scale = rand() % 100 - 50;

		tuple1.scaleAdd(scale, tuple2);

		tuple3.scale(scale);
		tuple3.add(tuple2);

		EXPECT_TRUE(tuple1.equals(tuple3));
	}
}

TEST_F(FrameTupleTest, testScaleAdd2)
{
	for (int i = 0; i < 1000; i++)
	{
		FrameTuple tuple1 = FrameUtilitiesTestHelper::getRandomFrameTuple("tuple1", frame3.get());
		FrameTuple tuple2 = FrameUtilitiesTestHelper::getRandomFrameTuple("tuple2", frame3.get());
		double scale = rand() % 100 - 50;

		FrameTuple tuple3;
		tuple3.scaleAdd(scale, tuple1, tuple2);

		tuple1.scale(scale);
		tuple1.add(tuple2);

		EXPECT_TRUE(tuple1.equals(tuple3));
	}
}

TEST_F(FrameTupleTest, testAbsoluteValue1)
{
	for (int i = 0; i < 1000; i++)
	{
		FrameTuple tuple1 = FrameUtilitiesTestHelper::getRandomFrameTuple("tuple1", frame2.get());
		FrameTuple tuple2 = tuple1;
		tuple1.absoluteValue();

		EXPECT_TRUE(tuple1.getX() == fabs(tuple2.getX()));
		EXPECT_TRUE(tuple1.getY() == fabs(tuple2.getY()));
		EXPECT_TRUE(tuple1.getZ() == fabs(tuple2.getZ()));
	}
}

TEST_F(FrameTupleTest, testAbsoluteValue2)
{
	for (int i = 0; i < 1000; i++)
	{
		FrameTuple tuple1 = FrameUtilitiesTestHelper::getRandomFrameTuple("tuple2", frame1.get());
		FrameTuple tuple2;
		tuple2.absoluteValue(tuple1);

		EXPECT_TRUE(tuple2.getX() == fabs(tuple1.getX()));
		EXPECT_TRUE(tuple2.getY() == fabs(tuple1.getY()));
		EXPECT_TRUE(tuple2.getZ() == fabs(tuple1.getZ()));
	}
}

TEST_F(FrameTupleTest, testClampMin1)
{
	for (int i = 0; i < 1000; i++)
	{
		FrameTuple tuple1("tuple1", root.get(), 100, -200, -300);
		FrameTuple tuple2 = tuple1;
		tuple2.clampMin(-100);

		EXPECT_TRUE(tuple2.getX() == 100);
		EXPECT_TRUE(tuple2.getY() == -100);
		EXPECT_TRUE(tuple2.getZ() == -100);
	}
}

TEST_F(FrameTupleTest, testClampMax1)
{
	for (int i = 0; i < 1000; i++)
	{
		FrameTuple tuple1("tuple1", root.get(), 100, -200, -300);
		FrameTuple tuple2 = tuple1;
		tuple2.clampMax(0.0);

		EXPECT_TRUE(tuple2.getX() == 0.0);
		EXPECT_TRUE(tuple2.getY() == -200);
		EXPECT_TRUE(tuple2.getZ() == -300);
	}
}

TEST_F(FrameTupleTest, testClampMinMax1)
{
	for (int i = 0; i < 1000; i++)
	{
		FrameTuple tuple1("tuple1", root.get(), 100, 200, 300);
		FrameTuple tuple2 = tuple1;
		tuple2.clampMinMax(-100, -50);

		EXPECT_TRUE(tuple2.getX() == -50);
		EXPECT_TRUE(tuple2.getY() == -50);
		EXPECT_TRUE(tuple2.getZ() == -50);
	}
}

TEST_F(FrameTupleTest, testClampMinMax2)
{
	for (int i = 0; i < 1000; i++)
	{
		FrameTuple tuple1("tuple1", root.get(), 100, 200, 300);
		FrameTuple tuple2 = tuple1;
		tuple2.clampMinMax(-100, 200);

		EXPECT_TRUE(tuple2.getX() == 100);
		EXPECT_TRUE(tuple2.getY() == 200);
		EXPECT_TRUE(tuple2.getZ() == 200);
	}
}

TEST_F(FrameTupleTest, testClampMinMax3)
{
	for (int i = 0; i < 1; i++)
	{
		FrameTuple tuple1("tuple1", frame3.get(), 100, 200, 300);
		FrameTuple tuple2 = tuple1;
		tuple2.clampMinMax(201, 220);

		EXPECT_TRUE(tuple2.getX() == 201);
		EXPECT_TRUE(tuple2.getY() == 201);
		EXPECT_TRUE(tuple2.getZ() == 220);
	}
}