#include "ReferenceFrame.hpp"
#include <random>
#include "tf/LinearMath/Quaternion.h"
#include <iostream>

/** This class and its implementation are an adaptation
**  of the ReferenceFrame.java by Jerry Pratt and the IHMC robotics group.
**  All credit goes to them.
**/

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

const ReferenceFrame* const ReferenceFrame::getWorldFrame()
{
	return worldFrame.get();
}

ReferenceFrame::~ReferenceFrame()
{

}

// copy constructor
ReferenceFrame::ReferenceFrame(const ReferenceFrame &referenceFrameToCopy)
{
	parentFrame = referenceFrameToCopy.parentFrame;
	frameName = referenceFrameToCopy.frameName;
	framesStartingWithRootEndingWithThis = referenceFrameToCopy.framesStartingWithRootEndingWithThis;
	transformToParent = referenceFrameToCopy.transformToParent;
	transformToRoot = referenceFrameToCopy.transformToRoot;
	isWorldFrame = referenceFrameToCopy.isWorldFrame;
	isBodyCenteredFrame = referenceFrameToCopy.isBodyCenteredFrame;
}

ReferenceFrame::ReferenceFrame(const std::string &frameName, bool isWorldFrame, bool isBodyCenteredFrame)
{
	this->frameName = frameName;
	this->isWorldFrame = isWorldFrame;
	this->isBodyCenteredFrame = isBodyCenteredFrame;
	this->parentFrame = nullptr;

	tf::Quaternion quaternion(0.0, 0.0, 0.0, 1.0);
	tf::Vector3 translation(0.0, 0.0, 0.0);
	this->transformToParent = tf::Transform(quaternion, translation);
	this->framesStartingWithRootEndingWithThis = constructVectorOfFramesStartingWithRootEndingWithThis(this);
}

ReferenceFrame::ReferenceFrame(const std::string &frameName, ReferenceFrame* parentFrame, const tf::Transform transfomToParent, bool isBodyCenteredFrame)
{
	this->frameName = frameName;
	this->parentFrame = parentFrame;
	this->transformToParent = transformToParent;
	this->isBodyCenteredFrame = isBodyCenteredFrame;
	this->isWorldFrame = false;
	this->framesStartingWithRootEndingWithThis = constructVectorOfFramesStartingWithRootEndingWithThis(this);
}


ReferenceFrame::ReferenceFrame(const std::string &frameName, ReferenceFrame* parentFrame, const tf::Transform &transformToParent, bool isWorldFrame, bool isBodyCenteredFrame)
{
	this->frameName = frameName;
	this->parentFrame = parentFrame;
	this->transformToParent = transformToParent;
	this->isWorldFrame = isWorldFrame;
	this->isBodyCenteredFrame = isBodyCenteredFrame;
	this->framesStartingWithRootEndingWithThis = constructVectorOfFramesStartingWithRootEndingWithThis(this);
}

ReferenceFrame::ReferenceFrame(const std::string &frameName, std::unique_ptr<ReferenceFrame> parentFrame, const tf::Transform &transformToParent, bool isWorldFrame, bool isBodyCenteredFrame)
{
	this->frameName = frameName;
	this->parentFrame = parentFrame.get();
	this->transformToParent = transformToParent;
	this->isWorldFrame = isWorldFrame;
	this->isBodyCenteredFrame = isBodyCenteredFrame;
	this->framesStartingWithRootEndingWithThis = constructVectorOfFramesStartingWithRootEndingWithThis(this);
}

ReferenceFrame::ReferenceFrame(const std::string &frameName, ReferenceFrame* parentFrame, bool isWorldFrame, bool isBodyCenteredFrame)
{
	tf::Quaternion quaternion(0.0, 0.0, 0.0, 1.0);
	tf::Vector3 translation(0.0, 0.0, 0.0);
	tf::Transform transformToParent(quaternion, translation);

	this->transformToParent = transformToParent;

	this->frameName = frameName;
	this->parentFrame = parentFrame;
	this->isWorldFrame = isWorldFrame;
	this->isBodyCenteredFrame = isBodyCenteredFrame;
	this->framesStartingWithRootEndingWithThis = constructVectorOfFramesStartingWithRootEndingWithThis(this);
}

std::vector<ReferenceFrame*> ReferenceFrame::constructVectorOfFramesStartingWithRootEndingWithThis(ReferenceFrame* thisFrame)
{
	ReferenceFrame* parentFrame = thisFrame->getParentFrame();
	if (parentFrame == nullptr)
	{
		// referenceFrame is the root frame.
		std::vector<ReferenceFrame*> vector(1);
		vector[0] = thisFrame;

		return vector;
	}

	// Need to add refereceFrame to the chain.
	int nElements = parentFrame->framesStartingWithRootEndingWithThis.size() + 1;
	std::vector<ReferenceFrame*> vector(nElements);
	// std::vector<ReferenceFrame*> vector;

	for (int i = 0; i < (nElements - 1); i++)
	{
		vector[i] = parentFrame->framesStartingWithRootEndingWithThis[i];
		// vector.push_back(parentFrame->framesStartingWithRootEndingWithThis[i]);
	}

	vector[nElements - 1] = thisFrame;
	// vector.push_back(thisFrame);

	// std::cout << "FrameName:" << thisFrame->getName() << std::endl;
	// std::cout << "Size Of Vector:" << vector.size() << std::endl;

	return vector;
}

void ReferenceFrame::getTransformToDesiredFrame(tf::Transform &transformToPack, const ReferenceFrame desiredFrame)
{

}

void ReferenceFrame::verifyFramesHaveSameRoot(const ReferenceFrame &frame)
{
	// if (!(frame.getRootFrame() == this->getRootFrame()))
	// {
	// 	throw std::runtime_error("Frames do not have the same root!");
	// }
}

void ReferenceFrame::setTransformToParent(const tf::Transform &transformToParent)
{
	this->transformToParent = transformToParent;
}