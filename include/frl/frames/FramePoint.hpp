#ifndef FRAME_POINT_HPP
#define FRAME_POINT_HPP

/**
 * This class and its implementation is an adaptation of FramePoint.java by Jerry Pratt and the IHMC Robotics Group.
 * All credit goes to them.
 */

#include "frl/frames/ReferenceFrame.hpp"
#include "frl/frames/ReferenceFrameHolder.hpp"
#include "frl/geometry/Point3d.hpp"

namespace frame_utilities
{
    class FramePoint : public ReferenceFrameHolder
    {
    public:
        FramePoint(const std::string &name);

        FramePoint(const std::string &name, ReferenceFrame *referenceFrame, const double &x, const double &y, const double &z);

        FramePoint(const std::string &name, ReferenceFrame *referenceFrame, double array[3]);

        FramePoint(const std::string &name, ReferenceFrame *referenceFrame, std::vector<double> vector);

        FramePoint(const std::string &name, ReferenceFrame *referenceFrame, const geometry_utilities::Point3d &point);

        FramePoint(const FramePoint &framePoint);

        FramePoint(const std::string &name, ReferenceFrame *referenceFrame);

        ~FramePoint(){};

        void setIncludingFrame(const double &x, const double &y, const double &z, ReferenceFrame *referenceFrame);
        void setIncludingFrame(const geometry_utilities::Point3d &point, ReferenceFrame *referenceFrame);

        void setAndKeepFrame(const double &x, const double &y, const double &z);

        double distance(const FramePoint &framePoint) const;

        double distanceSquared(const FramePoint &framePoint) const;

        void changeFrame(ReferenceFrame *desiredFrame);

        ReferenceFrame* getReferenceFrame() const
        {
            return this->referenceFrame;
        }

        geometry_utilities::Point3d getPoint() const
        {
            return point;
        }

        double getX() const
        {
            return point.getX();
        }

        double getY() const
        {
            return point.getY();
        }

        double getZ() const
        {
            return point.getZ();
        }

        ReferenceFrame *referenceFrame;
        geometry_utilities::Point3d point;
        std::string name;
    };

}

#endif