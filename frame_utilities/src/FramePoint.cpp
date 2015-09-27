#include "FramePoint.hpp"
#include "RigidBodyTransform.hpp"

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

double FramePoint::distance(const FramePoint &point)
{
	checkReferenceFramesMatch(point.getReferenceFrame());

	double dx = this->x - point.x;
	double dy = this->y - point.y;
	double dz = this->z - point.z;

	return sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
}

double FramePoint::distanceSquared(const FramePoint &point)
{
	checkReferenceFramesMatch(point.getReferenceFrame());

	double dx = this->x - point.x;
	double dy = this->y - point.y;
	double dz = this->z - point.z;

	return pow(dx, 2) + pow(dy, 2) + pow(dz, 2);
}

void FramePoint::changeFrame(ReferenceFrame* desiredFrame)
{
	if (desiredFrame != this->referenceFrame)
	{
		this->referenceFrame->verifyFramesHaveSameRoot(desiredFrame);

		geometry_utilities::RigidBodyTransform thisFramesTransformToRoot, desirdeFramesTransformToRoot;

		// if ((thisFramesTransformToRoot = this->referenceFrame->getTransformToRoot()) != nullptr)
		// {

		// }
	}





	// if (desiredFrame != referenceFrame)
	// {
	// 	referenceFrame.verifySameRoots(desiredFrame);
	// 	RigidBodyTransform referenceTf, desiredTf;

	// 	if ((referenceTf = referenceFrame.getTransformToRoot()) != null)
	// 	{
	// 		referenceTf.transform(tuple);
	// 	}

	// 	if ((desiredTf = desiredFrame.getInverseTransformToRoot()) != null)
	// 	{
	// 		desiredTf.transform(tuple);
	// 	}

	// 	referenceFrame = desiredFrame;
	// }
}

}