#ifndef FRAME_TUPLE_HPP
#define FRAME_TUPLE_HPP

#include "ReferenceFrame.hpp"
#include "ReferenceFrameHolder.hpp"
#include "Tuple3d.hpp"

namespace frame_utilities
{

class FrameTuple3d : public ReferenceFrameHolder
{
	public:
		ReferenceFrame* getReferenceFrame();
	protected:
		ReferenceFrame* referenceFrame;
		Tuple3d tuple;

	private:
};

}

#endif