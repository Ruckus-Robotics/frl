#ifndef TUPLE_3D_HPP
#define TUPLE_3D_HPP

#include <vector>
#include <stdexcept>

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

		double x, y, z;

	private:

};
}

#endif