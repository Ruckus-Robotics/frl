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

		ReferenceFrame* getReferenceFrame() const
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

		void subtract(const FrameTuple &tuple);
		void subtract(const FrameTuple &tuple1, const FrameTuple &tuple2);

		void negate();
		void negate(const FrameTuple &frameTuple);

		void scale(const double &value);
		void scale(const double &value, const FrameTuple &frameTuple);
		void scaleXYZ(const double &scaleX, const double &scaleY, const double &scaleZ);

		// void scaleAdd(const double &value, const FrameTuple &tuple);
		// void scaleAdd(const double &value, const FrameTuple &tuple1, const FrameTuple &tuple2);

		// bool equals(const FrameTuple &tuple);
		// bool epsilonEquals(const FrameTuple &tuple, const double &epsilon);

		// void clampMin(const double &min);
		// void clampMax(const double &max);
		// void clampMinMax(const double &min, const double &max);

		// void absoluteValue(const FrameTuple &tuple);
		// void absoluteValue();

		inline std::string getName() const
		{
			return this->name;
		}

		double x, y, z;

	protected:
		std::unique_ptr<ReferenceFrame> referenceFrame;
		std::string name;

	private:
};

}

#endif