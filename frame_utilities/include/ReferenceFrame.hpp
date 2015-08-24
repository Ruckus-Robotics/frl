#ifndef REFERENCE_FRAME_HPP
#define REFERENCE_FRAME_HPP

#include <memory>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <string>

class ReferenceFrame
{
	public:
		ReferenceFrame(const std::string &frameName, ReferenceFrame* const parentFrame, const tf::Transform &transformToParent, bool isWorldFrame, bool isBodyCenteredFrame);
		ReferenceFrame(const std::string &frameName, ReferenceFrame* const parentFrame, bool isWorldFrame, bool isBodyCenteredFrame);
		ReferenceFrame(const std::string &frameName, bool isWorldFrame, bool isBodyCenteredFrame);
		~ReferenceFrame();
		static std::unique_ptr<ReferenceFrame> createAWorldFrame(const std::string &frameName);
		static std::unique_ptr<ReferenceFrame> createARootFrame(const std::string &frameName);
		static const ReferenceFrame* getWorldFrame();

		static ReferenceFrame createFrameWithUnchangingTransformToParent(const std::string &name, ReferenceFrame* const parentFrame, const tf::Transform &transformToParent,
		        bool isBodyCenteredFrame, bool isWorldFrame);

		//Super classes are expected to override this method.
		virtual void updateTransformToParent(tf::Transform &transformToParent) {};

		inline tf::Transform getTransformToParent()
		{
			return this->transformToParent;
		}

	private:

		static std::unique_ptr<ReferenceFrame> worldFrame;
		std::string frameName;
		ReferenceFrame *parentFrame;
		tf::Transform transformToParent;
		tf::Transform transformToRoot;
		bool isWorldFrame;
		bool isBodyCenteredFrame;
};

#endif