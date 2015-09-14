#ifndef FRAME_TUPLE_HPP
#define FRAME_TUPLE_HPP

#include "ReferenceFrame.hpp"
#include "ReferenceFrameHolder.hpp"
#include "Tuple3d.hpp"

namespace frame_utilities
{

class FrameTuple : public ReferenceFrameHolder
{
	public:
		FrameTuple();
		FrameTuple(const std::string &name, ReferenceFrame* referenceFrame, const geometry_utilities::Tuple3d &tuple);

		ReferenceFrame* getReferenceFrame()
		{
			return this->referenceFrame;
		}

		void set(geometry_utilities::Tuple3d tuple);
		void set(double x, double y, double z);

		void setIncludingFrame(FrameTuple frameTuple);
		void setIncludingFrame(ReferenceFrame* referenceFrame, geometry_utilities::Tuple3d tuple);

		inline std::string getName()
		{
			return this->name;
		}

	protected:
		ReferenceFrame* referenceFrame;
		geometry_utilities::Tuple3d tuple;
		std::string name;

	private:
};

}

#endif