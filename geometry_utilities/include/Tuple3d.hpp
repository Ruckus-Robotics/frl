#ifndef TUPLE_3D_HPP
#define TUPLE_3D_HPP

#include <vector>
#include <stdexcept>
#include <iostream>

/** This class and its implementation are an adaptation
**  of the ReferenceFrame.java by Jerry Pratt and the IHMC robotics group.
**  All credit goes to them.
**/

namespace geometry_utilities
{
class Tuple3d
{
	public:
		Tuple3d();
		Tuple3d(const double &x, const double &y, const double &z);
		Tuple3d(const Tuple3d &tupleToCopy);
		Tuple3d(double array[3]);
		Tuple3d(std::vector<double> vector);

		void set(const double &x, const double &y, const double &z);
		void set(double array[3]);
		void set(std::vector<double> vector);

		void add(const Tuple3d &tuple);
		void add(const double &x, const double &y, const double &z);
		void add(const Tuple3d &tuple1, const Tuple3d &tuple2);

		void subtract(const Tuple3d &tuple);
		void subtract(const double &x, const double &y, const double &z);
		void subtract(const Tuple3d &tuple1, const Tuple3d &tuple2);

		void negate();
		void negate(const Tuple3d &tuple);

		void scale(const double &value);
		void scale(const double &value, const Tuple3d &tuple);

		void scaleAdd(const double &value, const Tuple3d &tuple);
		void scaleAdd(const double &value, const Tuple3d &tuple1, const Tuple3d &tuple2);

		bool equals(const Tuple3d &tuple);
		bool epsilonEquals(const Tuple3d &tuple, const double &epsilon);

		void clampMin(const double &min);
		void clampMax(const double &max);
		void clampMinMax(const double &min, const double &max);

		void absoluteValue(const Tuple3d &tuple);
		void absoluteValue();

		inline double getX()
		{
			return this->x;
		};
		inline double getY()
		{
			return this->y;
		};
		inline double getZ()
		{
			return this->z;
		};
		inline void setX(double x)
		{
			this->x = x;
		}
		inline void setY(double y)
		{
			this->y = y;
		}
		inline void setZ(double z)
		{
			this->z = z;
		}

		friend std::ostream &operator<<( std::ostream &os, const Tuple3d &tuple )
		{
			os << "x: " << tuple.x << '\n' << "y: " << tuple.y << '\n' << "z: " << tuple.z << "\n";
			return os;
		}

		double x, y, z;

	private:

};
}

#endif