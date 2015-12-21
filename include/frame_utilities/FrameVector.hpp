#ifndef __FRAME_VECTOR_HPP__
#define __FRAME_VECTOR_HPP__

/**
 * This class and its implementation is an adaptation of FrameVector.java by Jerry Pratt and the IHMC Robotics Group.
 * All credit goes to them.
 */

#include <eigen3/Eigen/Eigen>
#include "ReferenceFrame.hpp"
#include "ReferenceFrameHolder.hpp"

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

    private:
        Eigen::Vector3d vector;
        ReferenceFrame *referenceFrame;
        std::string name;
    };
}

#endif