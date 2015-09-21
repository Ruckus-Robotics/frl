#ifndef AXIS_ANGLE_HPP
#define AXIS_ANGLE_HPP


#include <eigen3/Eigen/Eigen>
#include <tf2/LinearMath/Quaternion.h>

namespace geometry_utilities
{

    class AxisAngle
    {
    public:
        AxisAngle();
        AxisAngle(const double& x, const double& y, const double& z, const double& angle);
        AxisAngle(const Eigen::Vector3d& axis, const double& angle);
        AxisAngle(const AxisAngle& axisAngle);

        void set(const double& x, const double& y, const double& z, const double& angle);
        void set(const Eigen::Vector4d& axisAngle);
        void set(const std::vector<double>& axisAngle);
        void set(const Eigen::Vector3d& axis, const double& angle);
        void set(const Eigen::Matrix4d& transform);
        void set(const Eigen::Matrix3d& rotationMatrix);
        void set(const tf2::Quaternion& q1);

        void setAngle(const double& angle);
        void setX(const double& x);
        void setY(const double& y);
        void setZ(const double& z);

        bool equals(const AxisAngle& a1);
        bool epsilonEquals(const AxisAngle& a1, const double& epsilon);

        double getAngle() const;
        double getX() const;
        double getY() const;
        double getZ() const;

        double x, y, z, angle;
    };

}

#endif
