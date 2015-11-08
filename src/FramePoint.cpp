#include "frame_utilities/FramePoint.hpp"
#include "geometry_utilities/RigidBodyTransform.hpp"

namespace frame_utilities
{

    FramePoint::FramePoint(const std::string &name, ReferenceFrame *referenceFrame, const double &x, const double &y, const double &z) : point(x, y, z)
    {
        this->name = name;
        this->referenceFrame = referenceFrame;
    }

    FramePoint::FramePoint(const std::string &name, ReferenceFrame *referenceFrame, double array[3]) : point(array)
    {
        this->name = name;
        this->referenceFrame = referenceFrame;
    }

    FramePoint::FramePoint(const std::string &name, ReferenceFrame *referenceFrame, std::vector<double> vector) : point(vector)
    {
        this->name = name;
        this->referenceFrame = referenceFrame;
    }

    FramePoint::FramePoint(const FramePoint &framePoint)
    {
        point.set(framePoint.point.x, framePoint.point.y, framePoint.point.z);
        this->referenceFrame = framePoint.referenceFrame;
    }

    FramePoint::FramePoint(const std::string &name, ReferenceFrame *referenceFrame) : point(0.0, 0.0, 0.0)
    {
        this->name = name;
        this->referenceFrame = referenceFrame;
    }

    double FramePoint::distance(const FramePoint &framePoint)
    {
        checkReferenceFramesMatch(framePoint.getReferenceFrame());

        double dx = this->point.x - framePoint.point.x;
        double dy = this->point.y - framePoint.point.y;
        double dz = this->point.z - framePoint.point.z;

        return sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
    }

    double FramePoint::distanceSquared(const FramePoint &framePoint)
    {
        checkReferenceFramesMatch(framePoint.getReferenceFrame());

        double dx = this->point.x - framePoint.point.x;
        double dy = this->point.y - framePoint.point.y;
        double dz = this->point.z - framePoint.point.z;

        return pow(dx, 2) + pow(dy, 2) + pow(dz, 2);
    }

    void FramePoint::changeFrame(ReferenceFrame *desiredFrame)
    {
        if (desiredFrame != this->referenceFrame)
        {
            this->referenceFrame->verifyFramesHaveSameRoot(desiredFrame);

            geometry_utilities::RigidBodyTransform thisFramesTransformToRoot, desiredFramesTransformToRoot;
            thisFramesTransformToRoot = this->referenceFrame->getTransformToRoot();
            desiredFramesTransformToRoot = desiredFrame->getTransformToRoot();

//		thisFramesTransformToRoot.transform()
        }





        // if (desiredFrame != referenceFrame)
        // {
        // 	referenceFrame.verifySameRoots(desiredFrame);
        // 	RigidBodyTransform referenceTf, desiredTf;

        // 	if ((referenceTf = referenceFrame.getTransformToRoot()) != null)
        // 	{
        // 		referenceTf.transform(tuple);
        // 	}

        // 	if ((desiredTf = desiredFrame.getInverseTransformToRoot()) != null)
        // 	{
        // 		desiredTf.transform(tuple);
        // 	}

        // 	referenceFrame = desiredFrame;
        // }
    }

}