#include <geometry_msgs/Point.h>
// #include "ReferenceFrame.hpp"


class FramePoint
{
	public:
		FramePoint();
		FramePoint(const FramePoint& framePointToCopy);

	private:
		geometry_msgs::Point point3d;
		const ReferenceFrame* referenceFrame;
};