#include "AxisAngle.hpp"
#include <stdexcept>

namespace geometry_utilities
{

    const double EPS = 1.0e-12;

    AxisAngle::AxisAngle(const double& x, const double& y, const double& z, const double& angle)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->angle = angle;
    }

    AxisAngle::AxisAngle(const AxisAngle& axisAngle)
    {
        this->x = axisAngle.x;
        this->y = axisAngle.y;
        this->z = axisAngle.z;
        this->angle = axisAngle.angle;
    }

    AxisAngle::AxisAngle(const Eigen::Vector3d& axis, const double& angle)
    {
        this->x = axis(0);
        this->y = axis(1);
        this->z = axis(2);
        this->angle = angle;
    }

    AxisAngle::AxisAngle()
    {
        this->x = 0.0;
        this->y = 0.0;
        this->z = 1.0;
        this->angle = 0.0;
    }

    void AxisAngle::set(const double& x, const double& y, const double& z, const double& angle)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->angle = angle;
    }


    void  AxisAngle::set(const std::vector<double>& axisAngle)
    {
        if (axisAngle.size() != 4)
        {
            throw std::runtime_error("Axis angle can ONLY have 4 elements!");
        }

        this->x = axisAngle[0];
        this->y = axisAngle[1];
        this->z = axisAngle[2];
        this->angle = axisAngle[3];
    }

    void  AxisAngle::set(const Eigen::Vector4d& axisAngle)
    {
        this->x = axisAngle(0);
        this->y = axisAngle(1);
        this->z = axisAngle(2);
        this->angle = axisAngle(3);
    }

    void AxisAngle::set(const Eigen::Vector3d& axis, const double& angle)
    {
        this->x = axis(0);
        this->y = axis(1);
        this->z = axis(2);
        this->angle = angle;
    }

    void AxisAngle::set(const RigidBodyTransform& transform)
    {
        Eigen::Matrix3d rotationMatrix;

        transform.get(rotationMatrix);

        x = (rotationMatrix(2, 1) - rotationMatrix(1, 2));
        y = (rotationMatrix(0, 2) - rotationMatrix(2, 0));
        z = (rotationMatrix(1, 0) - rotationMatrix(0, 1));

        double mag = x * x + y * y + z * z;

        if (mag > EPS )
        {
            mag = sqrt(mag);

            double sine = 0.5 * mag;
            double cosine = 0.5 * (rotationMatrix(0, 0) + rotationMatrix(1, 1) + rotationMatrix(2, 2) - 1.0);
            this->angle = atan2(sine, cosine);

            double invMag = 1.0 / mag;
            this->x = x * invMag;
            this->y = y * invMag;
            this->z = z * invMag;
        }
        else
        {
            this->x = 0.0;
            this->y = 1.0;
            this->z = 0.0;
            this->angle = 0.0;
        }
    }

    void AxisAngle::set(const Eigen::Matrix3d& rotationMatrix)
    {
        x = rotationMatrix(2, 1) - rotationMatrix(1, 2);
        y = rotationMatrix(0, 2) - rotationMatrix(2, 0);
        z = rotationMatrix(1, 0) - rotationMatrix(0, 1);

        double mag = x * x + y * y + z * z;

        if (mag > EPS )
        {
            mag = sqrt(mag);

            double sine = 0.5 * mag;
            double cosine = 0.5 * (rotationMatrix(0, 0) + rotationMatrix(1, 1) + rotationMatrix(2, 2) - 1.0);

            this->angle = atan2(sine, cosine);

            double invMag = 1.0 / mag;
            this->x = x * invMag;
            this->y = y * invMag;
            this->z = z * invMag;
        }
        else
        {
            this->x = 0.0;
            this->y = 1.0;
            this->z = 0.0;
            this->angle = 0.0;
        }

    }

    void AxisAngle::set(const tf2::Quaternion& q1)
    {
        double q1x = q1.getAxis().getX();
        double q1y = q1.getAxis().getY();
        double q1z = q1.getAxis().getZ();
        double q1w = q1.getW();

        double mag = q1x * q1x + q1y * q1y + q1z * q1z;

        if ( mag > EPS )
        {
            mag = sqrt(mag);
            double invMag = 1.0 / mag;

            this->x = q1x * invMag;
            this->y = q1y * invMag;
            this->z = q1z * invMag;
            this->angle = 2.0 * atan2(mag, q1w);
        }
        else
        {
            this->x = 0.0;
            this->y = 1.0;
            this->z = 0.0;
            this->angle = 0.0;
        }
    }

    bool AxisAngle::equals(const AxisAngle& a1)
    {
        return (this->x == a1.x && this->y == a1.y && this->z == a1.z
                && this->angle == a1.angle);
    }

    bool AxisAngle::epsilonEquals(const AxisAngle& a1, const double& epsilon)
    {
        double diff;

        diff = this->x - a1.x;

        if ((diff < 0 ? -diff : diff) > epsilon) { return false; }

        diff = this->y - a1.y;

        if ((diff < 0 ? -diff : diff) > epsilon) { return false; }

        diff = this->z - a1.z;

        if ((diff < 0 ? -diff : diff) > epsilon) { return false; }

        diff = this->angle - a1.angle;

        if ((diff < 0 ? -diff : diff) > epsilon) { return false; }

        return true;
    }

    double AxisAngle::getAngle() const
    {
        return this->angle;
    }

    void AxisAngle::setAngle(const double& angle)
    {
        this->angle = angle;
    }

    double AxisAngle::getX() const
    {
        return this->x;
    }

    void AxisAngle::setX(const double& x)
    {
        this->x = x;
    }

    double AxisAngle::getY() const
    {
        return this->y;
    }

    void AxisAngle::setY(const double& y)
    {
        this->y = y;
    }

    double AxisAngle::getZ() const
    {
        return this->z;
    }

    void AxisAngle::setZ(const double& z)
    {
        this->z = z;
    }

}