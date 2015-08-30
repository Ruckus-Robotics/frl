#ifndef REFERENCE_FRAME_HPP
#define REFERENCE_FRAME_HPP

/** This class and its implementation are an adaptation
**  of the ReferenceFrame.java by Jerry Pratt and the IHMC robotics group.
**  All credit goes to them.
**/

#include <memory>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <string>
#include <vector>

class ReferenceFrame
{
	public:
		ReferenceFrame(const std::string &frameName, ReferenceFrame* parentFrame, const tf::Transform &transformToParent, bool isWorldFrame, bool isBodyCenteredFrame);
		ReferenceFrame(const ReferenceFrame &referenceFrameToCopy);
		ReferenceFrame(const std::string &frameName, ReferenceFrame* parentFrame, bool isWorldFrame, bool isBodyCenteredFrame);
		ReferenceFrame(const std::string &frameName, std::unique_ptr<ReferenceFrame> parentframe, const tf::Transform &transformToParent, bool isWorldFrame, bool isBodyCenteredFrame);
		ReferenceFrame(const std::string &frameName, bool isWorldFrame, bool isBodyCenteredFrame);
		ReferenceFrame(const std::string &frameName, ReferenceFrame* parentFrame, const tf::Transform transfomToParent, bool isBodyCenteredFrame);
		ReferenceFrame() {};
		~ReferenceFrame();

		void getTransformToDesiredFrame(tf::Transform &transformToPack, const ReferenceFrame desiredFrame);
		tf::Transform getTransformToDesiredFrame(ReferenceFrame* desiredFrame);
		void verifyFramesHaveSameRoot(const ReferenceFrame &frame);
		void setTransformToParent(const tf::Transform &transformToParent);

		ReferenceFrame* getRootFrame()
		{
			return this->framesStartingWithRootEndingWithThis[0];
		}

		ReferenceFrame* getParentFrame()
		{
			return this->parentFrame;
		}

		std::string getName()
		{
			return this->frameName;
		}

		static std::unique_ptr<ReferenceFrame> createAWorldFrame(const std::string &frameName);
		static std::unique_ptr<ReferenceFrame> createARootFrame(const std::string &frameName);
		static const ReferenceFrame* const getWorldFrame();

		//Super classes are expected to override this method.
		virtual void updateTransformToParent(tf::Transform &transformToParent) {};

		inline tf::Transform getTransformToParent()
		{
			return this->transformToParent;
		}

	protected:

	private:
		static std::vector<ReferenceFrame*> constructVectorOfFramesStartingWithRootEndingWithThis(ReferenceFrame* thisFrame);

		static std::unique_ptr<ReferenceFrame> worldFrame;
		std::vector<ReferenceFrame*> framesStartingWithRootEndingWithThis;
		std::string frameName;
		ReferenceFrame *parentFrame;
		tf::Transform transformToParent;
		tf::Transform transformToRoot;
		bool isWorldFrame;
		bool isBodyCenteredFrame;
};

#endif