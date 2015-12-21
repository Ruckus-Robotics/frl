#ifndef FRAME_POINT_HPP
#define FRAME_POINT_HPP

/**
 * This class and its implementation is an adaptation of FramePoint.java by Jerry Pratt and the IHMC Robotics Group.
 * All credit goes to them.
 */

#include "ReferenceFrame.hpp"
#include "ReferenceFrameHolder.hpp"
#include "geometry_utilities/Point3d.hpp"

namespace frame_utilities
{

    class FramePoint : public ReferenceFrameHolder
    {
    public:
        FramePoint(const std::string &name);

        FramePoint(const std::string &name, ReferenceFrame *referenceFrame, const double &x, const double &y, const double &z);

        FramePoint(const std::string &name, ReferenceFrame *referenceFrame, double array[3]);

        FramePoint(const std::string &name, ReferenceFrame *referenceFrame, std::vector<double> vector);

        FramePoint(const FramePoint &framePoint);

        FramePoint(const std::string &name, ReferenceFrame *referenceFrame);

        void setIncludingFrame(const double &x, const double &y, const double &z, ReferenceFrame *referenceFrame);

        void setAndKeepFrame(const double &x, const double &y, const double &z);

        double distance(const FramePoint &framePoint) const;

        double distanceSquared(const FramePoint &framePoint) const;

        void changeFrame(ReferenceFrame *desiredFrame);

        ReferenceFrame* getReferenceFrame() const
        {
            return this->referenceFrame;
        }

        inline double getX() const
        {
            return point.getX();
        }

        inline double getY() const
        {
            return point.getY();
        }

        inline double getZ() const
        {
            return point.getZ();
        }

        ReferenceFrame *referenceFrame;
        geometry_utilities::Point3d point;
        std::string name;
    };

}

#endif