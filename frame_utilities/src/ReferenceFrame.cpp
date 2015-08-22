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

ReferenceFrame::ReferenceFrame(const std::string &frameName, bool isWorldFrame, bool isBodyCenteredFrame)
{
	this->frameName = frameName;
	this->isWorldFrame = isWorldFrame;
	this->isBodyCenteredFrame = isBodyCenteredFrame;
	this->parentFrame = NULL;

	this->transformToParent = geometry_msgs::Transform();
	this->transformToParent.rotation.x = 0.0;
	this->transformToParent.rotation.y = 0.0;
	this->transformToParent.rotation.z = 0.0;
	this->transformToParent.rotation.w = 1.0;

	this->transformToParent.translation.x = 0.0;
	this->transformToParent.translation.x = 0.0;
	this->transformToParent.translation.x = 0.0;

}


ReferenceFrame::ReferenceFrame(const std::string &frameName, ReferenceFrame *parentFrame, const geometry_msgs::Transform &transformToParent, bool isBodyCenteredFrame)
{
	this->frameName = frameName;
	this->parentFrame = parentFrame;
	this->transformToParent = transformToParent;
	this->isWorldFrame = false;
	this->isBodyCenteredFrame = isBodyCenteredFrame;
}
