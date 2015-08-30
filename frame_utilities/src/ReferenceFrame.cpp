#include "ReferenceFrame.hpp"
#include <random>
#include "tf/LinearMath/Quaternion.h"
#include <iostream>

/** This class and its implementation are an adaptation
**  of the ReferenceFrame.java by Jerry Pratt and the IHMC robotics group.
**  All credit goes to them.
**/

std::unique_ptr<ReferenceFrame> ReferenceFrame::worldFrame = ReferenceFrame::createAWorldFrame("World");
long ReferenceFrame::nextTransformToRootID = 1;

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

/** Create a top level ReferenceFrame with parentFrame = null **/
ReferenceFrame::ReferenceFrame(const std::string &frameName, bool isWorldFrame, bool isBodyCenteredFrame)
{
	this->frameName = frameName;
	this->isWorldFrame = isWorldFrame;
	this->isBodyCenteredFrame = isBodyCenteredFrame;
	this->parentFrame = nullptr;
	this->transformToRoot = createIdentityTransform();

	this->transformToParent = createIdentityTransform();
	this->framesStartingWithRootEndingWithThis = constructVectorOfFramesStartingWithRootEndingWithThis(this);
}

ReferenceFrame::ReferenceFrame(const std::string &frameName, ReferenceFrame* parentFrame, const tf::Transform &transfomToParent, bool isBodyCenteredFrame)
{
	this->frameName = frameName;
	this->parentFrame = parentFrame;
	this->transformToParent = transformToParent;
	this->isBodyCenteredFrame = isBodyCenteredFrame;
	this->transformToRoot = createIdentityTransform();
	this->isWorldFrame = false;
	this->framesStartingWithRootEndingWithThis = constructVectorOfFramesStartingWithRootEndingWithThis(this);
}


ReferenceFrame::ReferenceFrame(const std::string &frameName, ReferenceFrame* parentFrame, const tf::Transform &transformToParent, bool isWorldFrame, bool isBodyCenteredFrame)
{
	this->frameName = frameName;
	this->parentFrame = parentFrame;
	this->transformToParent = transformToParent;
	this->transformToRoot = createIdentityTransform();
	this->isWorldFrame = isWorldFrame;
	this->isBodyCenteredFrame = isBodyCenteredFrame;
	this->framesStartingWithRootEndingWithThis = constructVectorOfFramesStartingWithRootEndingWithThis(this);
}

ReferenceFrame::ReferenceFrame(const std::string &frameName, std::unique_ptr<ReferenceFrame> parentFrame, const tf::Transform &transformToParent, bool isWorldFrame, bool isBodyCenteredFrame)
{
	this->frameName = frameName;
	this->parentFrame = parentFrame.get();
	this->transformToParent = transformToParent;
	this->transformToRoot = createIdentityTransform();
	this->isWorldFrame = isWorldFrame;
	this->isBodyCenteredFrame = isBodyCenteredFrame;
	this->framesStartingWithRootEndingWithThis = constructVectorOfFramesStartingWithRootEndingWithThis(this);
}

ReferenceFrame::ReferenceFrame(const std::string &frameName, ReferenceFrame* parentFrame, bool isWorldFrame, bool isBodyCenteredFrame)
{
	this->transformToParent = createIdentityTransform();
	this->transformToRoot = createIdentityTransform();

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

// void ReferenceFrame::getTransformToDesiredFrame(ReferenceFrame* desiredFrame)
// {
// 	verifyFramesHaveSameRoot(desiredFrame);
// }

void ReferenceFrame::getTransformToDesiredFrame(tf::Transform &transformToPack, ReferenceFrame* desiredFrame)
{
	verifyFramesHaveSameRoot(desiredFrame);
}

void ReferenceFrame::verifyFramesHaveSameRoot( ReferenceFrame* frame)
{
	if (!(frame->getRootFrame() == this->getRootFrame()))
	{
		throw std::runtime_error("Frames do not have the same root!");
	}
}

void ReferenceFrame::setTransformToParent(const tf::Transform &transformToParent)
{
	this->transformToParent = transformToParent;
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

				// if (referenceFrame.preCorruptionTransform != null)
				// {
				// 	referenceFrame.transformToRoot.multiply(referenceFrame.preCorruptionTransform);
				// }

				frame->transformToRoot *= frame->transformToParent;

				// if (referenceFrame.postCorruptionTransform != null)
				// {
				// 	referenceFrame.transformToRoot.multiply(referenceFrame.postCorruptionTransform);
				// }

				// referenceFrame.inverseTransformToRoot.invert(referenceFrame.transformToRoot);

				// referenceFrame.transformToRootID = nextTransformToRootID;
			}
		}

		previousUpdateID = frame->transformToRootID;
	}
}

tf::Transform ReferenceFrame::createIdentityTransform()
{
	tf::Quaternion quaternion(0.0, 0.0, 0.0, 1.0);
	tf::Vector3 translation(0.0, 0.0, 0.0);
	tf::Transform transform(quaternion, translation);

	return transform;
}