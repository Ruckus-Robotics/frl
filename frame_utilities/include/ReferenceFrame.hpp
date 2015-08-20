#ifndef REFERENCE_FRAME_HPP
#define REFERENCE_FRAME_HPP

#include <geometry_msgs/Transform.h>
#include <tf/LinearMath/Quaternion.h>
#include <string>

class ReferenceFrame
{
public:
	ReferenceFrame(const std::string &frameName, ReferenceFrame *parentFrame, const geometry_msgs::Transform &transformToParent, bool isBodyCenteredFrame);
	ReferenceFrame(const std::string &frameName, bool isWorldFrame, bool isBodyCenteredFrame);
	~ReferenceFrame();
	static std::unique_ptr<ReferenceFrame> createAWorldFrame(const std::string &frameName);
	static std::unique_ptr<ReferenceFrame> createARootFrame(const std::string &frameName);
	static ReferenceFrame* getWorldFrame();

	virtual void updateTransformToParent(const geometry_msgs::Transform &transformToParent);
	inline geometry_msgs::Transform getTransformToParent(){return this->transformToParent;}
	
private:
	geometry_msgs::Transform createRandomTransformationMatrix();
	tf::Quaternion generateRandomQuaternion();
	double generateRandomAngle();

	static std::unique_ptr<ReferenceFrame> worldFrame;
	std::string frameName;
	ReferenceFrame *parentFrame;
	geometry_msgs::Transform transformToParent;
	geometry_msgs::Transform transformToRoot;
	bool isWorldFrame;
	bool isBodyCenteredFrame;
};

#endif