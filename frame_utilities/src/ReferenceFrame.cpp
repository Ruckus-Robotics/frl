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

ReferenceFrame* ReferenceFrame::getWorldFrame()
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

geometry_msgs::Transform ReferenceFrame::createRandomTransformationMatrix()
{
	geometry_msgs::Transform transform;
	tf::Quaternion quaternion = generateRandomQuaternion();
}

tf::Quaternion ReferenceFrame::generateRandomQuaternion()
{
	tf::Quaternion quaternion;
	quaternion.setRPY(generateRandomAngle(),generateRandomAngle(),generateRandomAngle());

	return quaternion;
}

double ReferenceFrame::generateRandomAngle()
{
	std::random_device randomDevice;
    std::mt19937 mt(randomDevice());
    std::uniform_real_distribution<double> dist(0, 1);

    return (dist(mt)*6.28 - 3.14);
}
