#ifndef FRAME_POINT_HPP
#define FRAME_POINT_HPP

#include "ReferenceFrame.hpp"
#include "ReferenceFrameHolder.hpp"
#include "geometry_utilities/Point3d.hpp"

namespace frame_utilities
{

class FramePoint : public ReferenceFrameHolder
{
	public:
		FramePoint(const std::string &name);
		FramePoint(const std::string &name, ReferenceFrame* referenceFrame, const double &x, const double &y, const double &z);
		FramePoint(const std::string &name, ReferenceFrame* referenceFrame, double array[3]);
		FramePoint(const std::string &name, ReferenceFrame* referenceFrame, std::vector<double> vector);
		FramePoint(const FramePoint &framePoint);
		FramePoint(const std::string &name, ReferenceFrame* referenceFrame);

		double distance(const FramePoint &framePoint);
		double distanceSquared(const FramePoint &framePoint);

		void changeFrame(ReferenceFrame* desiredFrame);

		ReferenceFrame* referenceFrame;
		geometry_utilities::Point3d point;
		std::string name;
};

}

#endif