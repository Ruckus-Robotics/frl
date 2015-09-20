#ifndef FRAME_TUPLE_HPP
#define FRAME_TUPLE_HPP

#include "ReferenceFrame.hpp"
#include "ReferenceFrameHolder.hpp"
#include <memory>

/** This class and its implementation are an adaptation
**  of the ReferenceFrame.java by Jerry Pratt and the IHMC robotics group.
**  All credit goes to them.
**/

namespace frame_utilities
{

class FrameTuple : public ReferenceFrameHolder
{
	public:
		FrameTuple();
		FrameTuple(const std::string &name, ReferenceFrame* referenceFrame);
		FrameTuple(const std::string &name, ReferenceFrame* referenceFrame, const double &x, const double &y, const double &z);
		FrameTuple(const FrameTuple &frameTuple);
		FrameTuple(const std::string &name, ReferenceFrame* referenceFrame, double array[3]);
		FrameTuple(const std::string &name, ReferenceFrame* referenceFrame, const std::vector<double> &vector);

		ReferenceFrame* getReferenceFrame()
		{
			return this->referenceFrame.get();
		}

		void set(const double &x, const double &y, const double &z);
		void set(double array[3]);
		void set(std::vector<double> vector);

		void setToZero();

		void setIncludingFrame(const FrameTuple &frameTuple);
		void setIncludingFrame(ReferenceFrame* referenceFrame, const double &x, const double &y, const double &z);

		void add(const FrameTuple &frameTuple);
		void add(const FrameTuple &frameTuple1, const FrameTuple &frameTuple2);

		// void subtract(const Tuple3d &tuple);
		// void subtract(const double &x, const double &y, const double &z);
		// void subtract(const Tuple3d &tuple1, const Tuple3d &tuple2);

		// void negate();
		// void negate(const Tuple3d &tuple);

		// void scale(const double &value);
		// void scale(const double &value, const Tuple3d &tuple);

		// void scaleAdd(const double &value, const Tuple3d &tuple);
		// void scaleAdd(const double &value, const Tuple3d &tuple1, const Tuple3d &tuple2);

		// bool equals(const Tuple3d &tuple);
		// bool epsilonEquals(const Tuple3d &tuple, const double &epsilon);

		// void clampMin(const double &min);
		// void clampMax(const double &max);
		// void clampMinMax(const double &min, const double &max);

		// void absoluteValue(const Tuple3d &tuple);
		// void absoluteValue();

		inline std::string getName()
		{
			return this->name;
		}

	protected:
		std::unique_ptr<ReferenceFrame> referenceFrame;
		double x, y, z;
		std::string name;

	private:
};

}

#endif