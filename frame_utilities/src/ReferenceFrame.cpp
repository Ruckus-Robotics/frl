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

std::vector<ReferenceFrame*> ReferenceFrame::constructVectorOfFramesStartingWithRootEndingWithThis(ReferenceFrame* thisFrame)
{
	if (thisFrame->parentFrame == nullptr)
	{
		// referenceFrame is the root frame.
		std::vector<ReferenceFrame*> vector;
		vector.push_back(thisFrame);

		return vector;
	}

	// Need to add refereceFrame to the chain.
	int nElements = thisFrame->framesStartingWithRootEndingWithThis.size() + 1;
	std::vector<ReferenceFrame*> vector(nElements);

	for (int i = 0; i < nElements - 1; i++)
	{
		vector[i] = thisFrame->parentFrame->framesStartingWithRootEndingWithThis[i];
	}

	vector[nElements] = thisFrame;

	return vector;
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
