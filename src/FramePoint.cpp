#include "frl/frames/FramePoint.hpp"
#include "frl/geometry/RigidBodyTransform.hpp"

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

    FramePoint::FramePoint(const std::string &name, ReferenceFrame *referenceFrame, const geometry_utilities::Point3d &point) : point(point)
    {
        this->name = name;
        this->referenceFrame = referenceFrame;
    }

    void FramePoint::setIncludingFrame(const double &x, const double &y, const double &z, ReferenceFrame *referenceFrame)
    {
        if(!referenceFrame)
        {
            throw std::runtime_error("Reference frame cannot be nullptr!");
        }
        this->point.setX(x);
        this->point.setY(y);
        this->point.setZ(z);
        this->referenceFrame = referenceFrame;
    }

    void FramePoint::setIncludingFrame(const geometry_utilities::Point3d &point, ReferenceFrame *referenceFrame)
    {
        setIncludingFrame(point.getX(),point.getY(),point.getZ(),referenceFrame);
    }

    void FramePoint::setAndKeepFrame(const double &x, const double &y, const double &z)
    {
        this->point.setX(x);
        this->point.setY(y);
        this->point.setZ(z);
    }

    double FramePoint::distance(const FramePoint &framePoint) const
    {
        checkReferenceFramesMatch(framePoint.getReferenceFrame());

        double distance = this->point.distance(framePoint.point);

        return distance;
    }

    double FramePoint::distanceSquared(const FramePoint &framePoint) const
    {
        checkReferenceFramesMatch(framePoint.getReferenceFrame());

        double distanceSquared = this->point.distanceSquared(framePoint.point);

        return distanceSquared;
    }

    void FramePoint::changeFrame(ReferenceFrame *desiredFrame)
    {
        if (desiredFrame != this->referenceFrame)
        {
            this->referenceFrame->verifyFramesHaveSameRoot(desiredFrame);

            geometry_utilities::RigidBodyTransform thisFramesTransformToRoot, desiredFramesInverseTransformToRoot;
            thisFramesTransformToRoot = this->referenceFrame->getTransformToRoot();
            desiredFramesInverseTransformToRoot = desiredFrame->getInverseTransformToRoot();

            if(this->referenceFrame && desiredFrame)
            {
                thisFramesTransformToRoot.transform(point);
                desiredFramesInverseTransformToRoot.transform(point);
                this->referenceFrame = desiredFrame;
            }
            else
            {
                throw std::runtime_error("Cannot change the frame of a FramePoint if either this's reference frame or the desired frame is nullptr!");
            }
        }
    }
}