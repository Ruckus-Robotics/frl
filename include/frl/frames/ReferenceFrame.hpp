#ifndef REFERENCE_FRAME_HPP
#define REFERENCE_FRAME_HPP

/** This class and its implementation are an adaptation
**  of the ReferenceFrame.java by Jerry Pratt and the IHMC robotics group.
**  All credit goes to them.
**/

#include <memory>
#include "frl/geometry/RigidBodyTransform.hpp"
#include <string>
#include <vector>
#include <climits>

namespace frames
{

class ReferenceFrame
{
	public:
		ReferenceFrame(const std::string &frameName, ReferenceFrame* parentFrame, const geometry::RigidBodyTransform &transformToParent, bool isWorldFrame, bool isBodyCenteredFrame);
		ReferenceFrame(const ReferenceFrame &referenceFrameToCopy);
		ReferenceFrame(const std::string &frameName, ReferenceFrame* parentFrame, bool isWorldFrame, bool isBodyCenteredFrame);
		ReferenceFrame(const std::string &frameName, std::unique_ptr<ReferenceFrame> parentframe, const geometry::RigidBodyTransform &transformToParent, bool isWorldFrame, bool isBodyCenteredFrame);
		ReferenceFrame(const std::string &frameName, bool isWorldFrame, bool isBodyCenteredFrame);
		ReferenceFrame(const std::string &frameName, ReferenceFrame* parentFrame, const geometry::RigidBodyTransform &transfomToParent, bool isBodyCenteredFrame);
		ReferenceFrame() {}
		~ReferenceFrame();

		void update();
		void getTransformToDesiredFrame(geometry::RigidBodyTransform &transformToPack, ReferenceFrame* desiredFrame);
		geometry::RigidBodyTransform getTransformToDesiredFrame(ReferenceFrame* desiredFrame);
		void verifyFramesHaveSameRoot(ReferenceFrame* desiredFrame);
		void setTransformToParent(const geometry::RigidBodyTransform &transformToParent);
		void checkReferenceFramesMatch(ReferenceFrame* referenceFrame) const;
		void checkReferenceFramesMatch(const ReferenceFrame* referenceFrame) const;

		geometry::RigidBodyTransform getTransformToRoot()
		{
			computeTransform();
			return this->transformToRoot;
		}

		geometry::RigidBodyTransform getInverseTransformToRoot()
		{
			return this->inverseTransformToRoot;
		}

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

		const std::vector<ReferenceFrame*> getFramesStartingWithRootEndingWithThis()
		{
			return this->framesStartingWithRootEndingWithThis;
		}

		static std::unique_ptr<ReferenceFrame> createAWorldFrame(const std::string &frameName);
		static std::unique_ptr<ReferenceFrame> createARootFrame(const std::string &frameName);
		static ReferenceFrame* getWorldFrame();

		virtual void updateTransformToParent(geometry::RigidBodyTransform &transformToParent) {};

		inline geometry::RigidBodyTransform getTransformToParent()
		{
			return this->transformToParent;
		}

	protected:

	private:
		static std::vector<ReferenceFrame*> constructVectorOfFramesStartingWithRootEndingWithThis(ReferenceFrame* thisFrame);

		void computeTransform();
		geometry::RigidBodyTransform createIdentityTransform();

		static long nextTransformToRootID;
		long transformToRootID = LLONG_MIN;
		static std::unique_ptr<ReferenceFrame> worldFrame;
		std::vector<ReferenceFrame*> framesStartingWithRootEndingWithThis;
		std::string frameName;
		ReferenceFrame *parentFrame;
		geometry::RigidBodyTransform transformToParent;
		geometry::RigidBodyTransform transformToRoot;
		geometry::RigidBodyTransform inverseTransformToRoot;
		bool isWorldFrame;
		bool isBodyCenteredFrame;
};

}

#endif
