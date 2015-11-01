#include "frame_utilities/ReferenceFrame.hpp"
#include "frame_utilities/ReferenceFrameHolder.hpp"
#include <random>
#include <iostream>
#include <math.h>
#include <stdexcept>

/** This class and its implementation are an adaptation
**  of the ReferenceFrame.java by Jerry Pratt and the IHMC robotics group.
**  All credit goes to them.
**/

namespace frame_utilities
{

std::unique_ptr<ReferenceFrame> ReferenceFrame::worldFrame = ReferenceFrame::createAWorldFrame("World");
long ReferenceFrame::nextTransformToRootID = 1;

std::unique_ptr<ReferenceFrame> ReferenceFrame::createAWorldFrame(const std::string& frameName)
{
	std::unique_ptr<ReferenceFrame> worldFrame( new ReferenceFrame(frameName, true, false) );
	return worldFrame;
}

std::unique_ptr<ReferenceFrame> ReferenceFrame::createARootFrame(const std::string& frameName)
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

// copy constructor
ReferenceFrame::ReferenceFrame(const ReferenceFrame& referenceFrameToCopy)
{
	parentFrame = referenceFrameToCopy.parentFrame;
	frameName = referenceFrameToCopy.frameName;
	framesStartingWithRootEndingWithThis = referenceFrameToCopy.framesStartingWithRootEndingWithThis;
	transformToParent = referenceFrameToCopy.transformToParent;
	transformToRoot = referenceFrameToCopy.transformToRoot;
	isWorldFrame = referenceFrameToCopy.isWorldFrame;
	isBodyCenteredFrame = referenceFrameToCopy.isBodyCenteredFrame;
}

/** Create a top level ReferenceFrame with parentFrame = null **/
ReferenceFrame::ReferenceFrame(const std::string& frameName, bool isWorldFrame, bool isBodyCenteredFrame)
{
	this->frameName = frameName;
	this->isWorldFrame = isWorldFrame;
	this->isBodyCenteredFrame = isBodyCenteredFrame;
	this->parentFrame = nullptr;
	this->transformToRoot.setIdentity();
	this->inverseTransformToRoot.setIdentity();
	this->transformToRootID = 0;

	this->transformToParent.setIdentity();
	std::vector<ReferenceFrame*> vector;
	vector.push_back(this);
	this->framesStartingWithRootEndingWithThis = vector;
}

ReferenceFrame::ReferenceFrame(const std::string& frameName, ReferenceFrame* parentFrame, const geometry_utilities::RigidBodyTransform& transfomToParent, bool isBodyCenteredFrame)
{
	this->frameName = frameName;
	this->parentFrame = parentFrame;
	this->transformToParent = transformToParent;
	this->isBodyCenteredFrame = isBodyCenteredFrame;
	this->transformToRoot.setIdentity();
	this->inverseTransformToRoot.setIdentity();
	this->isWorldFrame = false;
	this->framesStartingWithRootEndingWithThis = constructVectorOfFramesStartingWithRootEndingWithThis(this);
}


ReferenceFrame::ReferenceFrame(const std::string& frameName, ReferenceFrame* parentFrame, const geometry_utilities::RigidBodyTransform& transformToParent, bool isWorldFrame, bool isBodyCenteredFrame)
{
	this->frameName = frameName;
	this->parentFrame = parentFrame;
	this->transformToParent = transformToParent;
	this->transformToRoot.setIdentity();
	this->inverseTransformToRoot.setIdentity();
	this->isWorldFrame = isWorldFrame;
	this->isBodyCenteredFrame = isBodyCenteredFrame;
	this->framesStartingWithRootEndingWithThis = constructVectorOfFramesStartingWithRootEndingWithThis(this);
}

ReferenceFrame::ReferenceFrame(const std::string& frameName, std::unique_ptr<ReferenceFrame> parentFrame, const geometry_utilities::RigidBodyTransform& transformToParent, bool isWorldFrame, bool isBodyCenteredFrame)
{
	this->frameName = frameName;
	this->parentFrame = parentFrame.get();
	this->transformToParent = transformToParent;
	this->transformToRoot.setIdentity();
	this->inverseTransformToRoot.setIdentity();
	this->isWorldFrame = isWorldFrame;
	this->isBodyCenteredFrame = isBodyCenteredFrame;
	this->framesStartingWithRootEndingWithThis = constructVectorOfFramesStartingWithRootEndingWithThis(this);
}

ReferenceFrame::ReferenceFrame(const std::string& frameName, ReferenceFrame* parentFrame, bool isWorldFrame, bool isBodyCenteredFrame)
{
	this->transformToParent.setIdentity();
	this->transformToRoot.setIdentity();
	this->inverseTransformToRoot.setIdentity();

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

	for (int i = 0; i < (nElements - 1); i++)
	{
		vector[i] = parentFrame->framesStartingWithRootEndingWithThis[i];
	}

	vector[nElements - 1] = thisFrame;

	return vector;
}

geometry_utilities::RigidBodyTransform ReferenceFrame::getTransformToDesiredFrame(ReferenceFrame* desiredFrame)
{
	geometry_utilities::RigidBodyTransform transform;
	getTransformToDesiredFrame(transform, desiredFrame);
	return transform;
}

void ReferenceFrame::getTransformToDesiredFrame(geometry_utilities::RigidBodyTransform& transformToPack, ReferenceFrame* desiredFrame)
{
	verifyFramesHaveSameRoot(desiredFrame);

	this->computeTransform();
	desiredFrame->computeTransform();

	geometry_utilities::RigidBodyTransform tmpTransform = desiredFrame->inverseTransformToRoot;
	geometry_utilities::RigidBodyTransform tmpTransform2 = this->transformToRoot;

	tmpTransform *= tmpTransform2;

	transformToPack = tmpTransform;

}

void ReferenceFrame::verifyFramesHaveSameRoot( ReferenceFrame* frame)
{
	if (!(frame->getRootFrame() == this->getRootFrame()))
	{
		throw std::runtime_error("Frames do not have the same root!");
	}
}

void ReferenceFrame::setTransformToParent(const geometry_utilities::RigidBodyTransform& transformToParent)
{
	this->transformToParent = transformToParent;
}

void ReferenceFrame::update()
{
	// NOTE: IF YOU OVERRIDE UPDATE, YOU MUST RESET this->transformToRootID TO LLONG_MIN IN THE UPDATE METHOD!!

	updateTransformToParent(this->transformToParent);

	this->transformToRootID = LLONG_MIN;

}

void ReferenceFrame::computeTransform()
{
	int chainLength = this->framesStartingWithRootEndingWithThis.size();

	bool updateFromHereOnOut = false;
	long previousUpdateID = 0;

	for (int i = 0; i < chainLength; i++)
	{
		ReferenceFrame* frame = this->framesStartingWithRootEndingWithThis[i];

		if (!updateFromHereOnOut)
		{
			if (frame->transformToRootID < previousUpdateID)
			{
				updateFromHereOnOut = true;
				nextTransformToRootID++;
			}
		}

		if (updateFromHereOnOut)
		{
			if (frame->getParentFrame() != nullptr)
			{
				geometry_utilities::RigidBodyTransform parentsTransformToRoot = frame->getParentFrame()->transformToRoot;

				frame->transformToRoot = parentsTransformToRoot;

				frame->transformToRoot *= frame->transformToParent;

				geometry_utilities::RigidBodyTransform transformToRoot = frame->transformToRoot;
				frame->inverseTransformToRoot = transformToRoot;
				frame->inverseTransformToRoot.invert();

				frame->transformToRootID = nextTransformToRootID;
			}
		}

		previousUpdateID = frame->transformToRootID;
	}
}

void ReferenceFrame::checkReferenceFramesMatch(ReferenceFrame* referenceFrame)
{
	if (referenceFrame == nullptr)
	{
		throw std::runtime_error("referenceFrame is NULL!");
	}

	if (referenceFrame != this)
	{
		throw std::runtime_error("Frame mismatch!");
	}
}

}
