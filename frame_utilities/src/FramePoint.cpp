#include "FramePoint.hpp"

namespace frame_utilities
{

FramePoint::FramePoint() : FrameTuple()
{

}

FramePoint::FramePoint(const std::string &name, ReferenceFrame* referenceFrame, const double &x, const double &y, const double &z) : FrameTuple(name, referenceFrame, x, y, z)
{

}

FramePoint::FramePoint(const std::string &name, ReferenceFrame* referenceFrame, double array[3]) : FrameTuple(name, referenceFrame, array)
{

}

FramePoint::FramePoint(const std::string &name, ReferenceFrame* referenceFrame, std::vector<double> vector) : FrameTuple(name, referenceFrame, vector)
{

}

FramePoint::FramePoint(const FramePoint &framePoint) : FrameTuple(framePoint)
{

}

FramePoint::FramePoint(const std::string &name, ReferenceFrame* referenceFrame) : FrameTuple(name, referenceFrame)
{

}

}