#include "ReferenceFrame.hpp"
#include "ReferenceFrameHolder.hpp"
#include <random>
#include "tf/LinearMath/Quaternion.h"
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
	this->transformToRoot = createIdentityTransform();
	this->inverseTransformToRoot = createIdentityTransform();
	this->transformToRootID = 0;

	this->transformToParent = createIdentityTransform();
	std::vector<ReferenceFrame*> vector;
	vector.push_back(this);
	this->framesStartingWithRootEndingWithThis = vector;
}

ReferenceFrame::ReferenceFrame(const std::string& frameName, ReferenceFrame* parentFrame, const tf::Transform& transfomToParent, bool isBodyCenteredFrame)
{
	this->frameName = frameName;
	this->parentFrame = parentFrame;
	this->transformToParent = transformToParent;
	this->isBodyCenteredFrame = isBodyCenteredFrame;
	this->transformToRoot = createIdentityTransform();
	this->inverseTransformToRoot = createIdentityTransform();
	this->isWorldFrame = false;
	this->framesStartingWithRootEndingWithThis = constructVectorOfFramesStartingWithRootEndingWithThis(this);
}


ReferenceFrame::ReferenceFrame(const std::string& frameName, ReferenceFrame* parentFrame, const tf::Transform& transformToParent, bool isWorldFrame, bool isBodyCenteredFrame)
{
	this->frameName = frameName;
	this->parentFrame = parentFrame;
	this->transformToParent = transformToParent;
	this->transformToRoot = createIdentityTransform();
	this->inverseTransformToRoot = createIdentityTransform();
	this->isWorldFrame = isWorldFrame;
	this->isBodyCenteredFrame = isBodyCenteredFrame;
	this->framesStartingWithRootEndingWithThis = constructVectorOfFramesStartingWithRootEndingWithThis(this);
}

ReferenceFrame::ReferenceFrame(const std::string& frameName, std::unique_ptr<ReferenceFrame> parentFrame, const tf::Transform& transformToParent, bool isWorldFrame, bool isBodyCenteredFrame)
{
	this->frameName = frameName;
	this->parentFrame = parentFrame.get();
	this->transformToParent = transformToParent;
	this->transformToRoot = createIdentityTransform();
	this->inverseTransformToRoot = createIdentityTransform();
	this->isWorldFrame = isWorldFrame;
	this->isBodyCenteredFrame = isBodyCenteredFrame;
	this->framesStartingWithRootEndingWithThis = constructVectorOfFramesStartingWithRootEndingWithThis(this);
}

ReferenceFrame::ReferenceFrame(const std::string& frameName, ReferenceFrame* parentFrame, bool isWorldFrame, bool isBodyCenteredFrame)
{
	this->transformToParent = createIdentityTransform();
	this->transformToRoot = createIdentityTransform();
	this->inverseTransformToRoot = createIdentityTransform();

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

tf::Transform ReferenceFrame::getTransformToDesiredFrame(ReferenceFrame* desiredFrame)
{
	tf::Transform transform;
	getTransformToDesiredFrame(transform, desiredFrame);
	return transform;
}

void ReferenceFrame::getTransformToDesiredFrame(tf::Transform& transformToPack, ReferenceFrame* desiredFrame)
{
	verifyFramesHaveSameRoot(desiredFrame);

	this->computeTransform();
	desiredFrame->computeTransform();

	tf::Transform tmpTransform = desiredFrame->inverseTransformToRoot;
	tf::Transform tmpTransform2 = this->transformToRoot;

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

void ReferenceFrame::setTransformToParent(const tf::Transform& transformToParent)
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
				tf::Transform parentsTransformToRoot = frame->getParentFrame()->transformToRoot;

				frame->transformToRoot = parentsTransformToRoot;

				frame->transformToRoot *= frame->transformToParent;

				frame->transformToRoot.getRotation().normalize();

				tf::Transform transformToRoot = frame->transformToRoot;
				frame->inverseTransformToRoot = transformToRoot.inverse();

				frame->transformToRootID = nextTransformToRootID;
			}
		}

		previousUpdateID = frame->transformToRootID;
	}
}

tf::Transform ReferenceFrame::createIdentityTransform()
{
	tf::Quaternion quaternion;
	quaternion = tf::Quaternion::getIdentity();
	tf::Vector3 translation(0.0, 0.0, 0.0);
	tf::Transform transform(quaternion, translation);

	return transform;
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
