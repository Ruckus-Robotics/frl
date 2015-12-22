#include "frl/frame_utilities/FrameVector.hpp"

namespace frame_utilities
{

    FrameVector::FrameVector(const std::string &name, ReferenceFrame *referenceFrame, const double &x, const double &y, const double &z) : vector(x,y,z)
    {
        if(!referenceFrame)
        {
            throw std::runtime_error("Reference frame cannot be nullptr!");
        }

        this->name = name;
        this->referenceFrame = referenceFrame;
    }

    FrameVector::FrameVector(const std::string &name, ReferenceFrame *referenceFrame, const Eigen::Vector3d &vector) : vector(vector)
    {
        if(!referenceFrame)
        {
            throw std::runtime_error("Reference frame cannot be nullptr!");
        }

        this->name = name;
        this->referenceFrame = referenceFrame;
    }

    void FrameVector::setIncludingFrame(const double &x, const double &y, const double &z, ReferenceFrame *referenceFrame)
    {
        if(!referenceFrame)
        {
            throw std::runtime_error("Reference frame cannot be nullptr!");
        }

        this->vector(0) = x;
        this->vector(1) = y;
        this->vector(2) = z;

        this->referenceFrame = referenceFrame;
    }

    void FrameVector::setIncludingFrame(const Eigen::Vector3d &vector, ReferenceFrame *referenceFrame)
    {
        if(!referenceFrame)
        {
            throw std::runtime_error("Reference frame cannot be nullptr!");
        }

        setIncludingFrame(vector(0),vector(1),vector(2),referenceFrame);
    }

    void FrameVector::setAndKeepFrame(const double &x, const double &y, const double &z)
    {
        vector(0) = x;
        vector(1) = y;
        vector(2) = z;
    }

    void FrameVector::setAndKeepFrame(const Eigen::Vector3d vector)
    {
        setAndKeepFrame(vector(0),vector(1),vector(2));
    }

    double FrameVector::dot(const FrameVector &frameVector) const
    {
        checkReferenceFramesMatch(&frameVector);
        return this->vector.dot(frameVector.getVector());
    }

    void FrameVector::cross(const FrameVector &frameVector,FrameVector &frameVectorToPack) const
    {
        checkReferenceFramesMatch(&frameVector);
        checkReferenceFramesMatch(&frameVectorToPack);
        Eigen::Vector3d tmpVector3d = this->vector.cross(frameVector.getVector());
        frameVectorToPack.setAndKeepFrame(tmpVector3d(0),tmpVector3d(1),tmpVector3d(2));
    }

    Eigen::Vector3d FrameVector::cross(const FrameVector &frameVector) const
    {
        checkReferenceFramesMatch(&frameVector);
        return this->vector.cross(frameVector.getVector());
    }

}