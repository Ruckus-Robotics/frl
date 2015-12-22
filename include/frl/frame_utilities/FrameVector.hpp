#ifndef __FRAME_VECTOR_HPP__
#define __FRAME_VECTOR_HPP__

/**
 * This class and its implementation is an adaptation of FrameVector.java by Jerry Pratt and the IHMC Robotics Group.
 * All credit goes to them.
 */

#include <eigen3/Eigen/Eigen>
#include "frl/frame_utilities/ReferenceFrame.hpp"
#include "frl/frame_utilities/ReferenceFrameHolder.hpp"

namespace frame_utilities
{
    class FrameVector : public ReferenceFrameHolder
    {
    public:
        FrameVector(const std::string &name, ReferenceFrame *referenceFrame, const double &x, const double &y, const double &z);
        FrameVector(const std::string &name, ReferenceFrame *referenceFrame, const Eigen::Vector3d &vector);

        ~FrameVector(){};

        void setIncludingFrame(const double &x, const double &y, const double &z, ReferenceFrame *referenceFrame);
        void setIncludingFrame(const Eigen::Vector3d &vector, ReferenceFrame *referenceFrame);

        void setAndKeepFrame(const double &x, const double &y, const double &z);
        void setAndKeepFrame(const Eigen::Vector3d vector);

        double dot(const FrameVector &frameVector) const;
        // NEED TO FINISH FRAMEVECTOR!!
        void cross(const FrameVector &frameVector,FrameVector &frameVectorToPack) const;
        double length() const;
        double getAngleBetweenVectors(const FrameVector &frameVector) const;

        ReferenceFrame* getReferenceFrame() const
        {
            return this->referenceFrame;
        }

        Eigen::Vector3d getVector() const
        {
            return this->vector;
        }

    private:
        Eigen::Vector3d vector;
        ReferenceFrame *referenceFrame;
        std::string name;
    };
}

#endif