#include "ReferenceFrame.hpp"
#include <random>
#include "tf/LinearMath/Quaternion.h"

std::unique_ptr<ReferenceFrame> ReferenceFrame::worldFrame = ReferenceFrame::createAWorldFrame("World");

std::unique_ptr<ReferenceFrame> ReferenceFrame::createAWorldFrame(const std::string &frameName)
{
	std::unique_ptr<ReferenceFrame> worldFrame( new ReferenceFrame(frameName, true, false) );
	return worldFrame;
}

std::unique_ptr<ReferenceFrame> ReferenceFrame::createARootFrame(const std::string &frameName)
{
	std::unique_ptr<ReferenceFrame> rootFrame( new ReferenceFrame(frameName, false, false) );
	return rootFrame;
}

const ReferenceFrame* ReferenceFrame::getWorldFrame()
{
	return worldFrame.get();
}

ReferenceFrame::~ReferenceFrame()
{

}

std::vector<ReferenceFrame> ReferenceFrame::constructVectorOfFramesStartingWithRootEndingWithThis()
{

}

ReferenceFrame::ReferenceFrame(const std::string &frameName, bool isWorldFrame, bool isBodyCenteredFrame)
{
	this->frameName = frameName;
	this->isWorldFrame = isWorldFrame;
	this->isBodyCenteredFrame = isBodyCenteredFrame;
	this->parentFrame = NULL;

	tf::Quaternion quaternion(0.0, 0.0, 0.0, 1.0);
	tf::Vector3 translation(0.0, 0.0, 0.0);
	this->transformToParent = tf::Transform(quaternion, translation);

}


ReferenceFrame::ReferenceFrame(const std::string &frameName, ReferenceFrame* const parentFrame, const tf::Transform &transformToParent, bool isWorldFrame, bool isBodyCenteredFrame)
{
	this->frameName = frameName;
	this->parentFrame = parentFrame;
	this->transformToParent = transformToParent;
	this->isWorldFrame = isWorldFrame;
	this->isBodyCenteredFrame = isBodyCenteredFrame;
}

ReferenceFrame::ReferenceFrame(const std::string &frameName, ReferenceFrame* const parentFrame, bool isWorldFrame, bool isBodyCenteredFrame)
{
	tf::Quaternion quaternion(0.0, 0.0, 0.0, 1.0);
	tf::Vector3 translation(0.0, 0.0, 0.0);
	tf::Transform transformToParent(quaternion, translation);

	this->transformToParent = transformToParent;

	this->frameName = frameName;
	this->parentFrame = parentFrame;
	this->isWorldFrame = isWorldFrame;
	this->isBodyCenteredFrame = isBodyCenteredFrame;
}

ReferenceFrame ReferenceFrame::createFrameWithUnchangingTransformToParent(const std::string &frameName, ReferenceFrame* const parentFrame, const tf::Transform &transformToParent,
        bool isBodyCenteredFrame, bool isWorldFrame)
{
	//Need to check here if the quaternion in tf is valid. Its possible tf makes sure it is, not sure.
	ReferenceFrame frame(frameName, parentFrame, transformToParent, isWorldFrame, isBodyCenteredFrame);

	return frame;
}
