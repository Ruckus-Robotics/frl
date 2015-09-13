#ifndef REFERENCE_FRAME_HOLDER_HPP
#define REFERENCE_FRAME_HOLDER_HPP

namespace frame_utilities
{

class ReferenceFrameHolder
{
	public:
		virtual ReferenceFrame* getReferenceFrame() = 0;
		virtual bool checkReferenceFramesMatch() = 0;
};

}

#endif