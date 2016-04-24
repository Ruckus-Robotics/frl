#include "frl/geometry/Point3d.hpp"

namespace geometry
{

    Point3d::Point3d(const double &x, const double &y, const double &z)
    {
        set(x, y, z);
    }

    Point3d::Point3d(const std::vector<double> &vector)
    {
        set(vector[0], vector[1], vector[2]);
    }

    Point3d::Point3d(const Point3d &point)
    {
        set(point.x, point.y, point.z);
    }

    Point3d::Point3d(const Eigen::Vector3d &vector)
    {
        set(vector(0), vector(1), vector(2));
    }

    Point3d::Point3d()
    {
        set(0.0, 0.0, 0.0);
    }

    Point3d::Point3d(const double array[3])
    {
        set(array[0],array[1],array[2]);
    }

    void Point3d:: set(const std::vector<double> &vector)
    {
        set(vector[0],vector[1],vector[2]);
    }

    void Point3d::set(const double &x, const double &y, const double &z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    void Point3d::add(const double &x, const double &y, const double &z)
    {
        this->x += x;
        this->y += y;
        this->z += z;
    }

    void Point3d::subtract(const double &x, const double &y, const double &z)
    {
        this->x -= x;
        this->y -= y;
        this->z -= z;
    }

    void Point3d::negate()
    {
        this->x *= -1;
        this->y *= -1;
        this->z *= -1;
    }

    void Point3d::scale(const double &scale)
    {
        this->x *= scale;
        this->y *= scale;
        this->z *= scale;
    }

    void Point3d::scaleAdd(const double &scale, const Point3d &point)
    {
        this->x *= scale;
        this->y *= scale;
        this->z *= scale;

        this->x += point.x;
        this->y += point.y;
        this->z += point.z;
    }

    bool Point3d::epsilonEquals(const Point3d &tuple, const double &epsilon)
    {
        if (std::isnan(this->x) || std::isnan(this->y) || std::isnan(this->z) || std::isnan(tuple.x) || std::isnan(tuple.y) || std::isnan(tuple.z))
        {
            return false;
        }

        return (fabs(this->x - tuple.x) < epsilon && fabs(this->y - tuple.y) < epsilon && fabs(this->z - tuple.z) < epsilon);
    }

    bool Point3d::equals(const Point3d &tuple)
    {
        if (std::isnan(this->x) || std::isnan(this->y) || std::isnan(this->z) || std::isnan(tuple.x) || std::isnan(tuple.y) || std::isnan(tuple.z))
        {
            return false;
        }

        return (this->x == tuple.x && this->y == tuple.y && this->z == tuple.z);
    }

    void Point3d::clampMin(const double &min)
    {
        if (this->x < min)
        {
            this->x = min;
        }

        if (this->y < min)
        {
            this->y = min;
        }

        if (this->z < min)
        {
            this->z = min;
        }
    }

    void Point3d::clampMax(const double &max)
    {
        if (this->x > max)
        {
            this->x = max;
        }

        if (this->y > max)
        {
            this->y = max;
        }

        if (this->z > max)
        {
            this->z = max;
        }
    }

    void Point3d::clampMinMax(const double &min, const double &max)
    {
        if (min > max)
        {
            throw std::runtime_error("Invalid bounds!");
        }

        if (this->x < min)
        {
            this->x = min;
        }
        else if (this->x > max)
        {
            this->x = max;
        }

        if (this->y < min)
        {
            this->y = min;
        }
        else if (this->y > max)
        {
            this->y = max;
        }

        if (this->z < min)
        {
            this->z = min;
        }
        else if (this->z > max)
        {
            this->z = max;
        }
    }

    void Point3d::absoluteValue()
    {
        this->x = fabs(this->x);
        this->y = fabs(this->y);
        this->z = fabs(this->z);
    }

    double Point3d::distanceSquared(const Point3d &point) const
    {
        double dx, dy, dz;

        dx = this->x - point.x;
        dy = this->y - point.y;
        dz = this->z - point.z;
        return (dx * dx + dy * dy + dz * dz);
    }

    double Point3d::distance(const Point3d point) const
    {
        double dx, dy, dz;

        dx = this->x - point.x;
        dy = this->y - point.y;
        dz = this->z - point.z;
        return sqrt(dx * dx + dy * dy + dz * dz);
    }

    double Point3d::distanceL1(const Point3d &point) const
    {
        return (fabs(this->x - point.x) + fabs(this->y - point.y) +
                fabs(this->z - point.z));
    }

    double Point3d::distanceLinf(const Point3d &point) const
    {
        double tmp;
        tmp = std::max(fabs(this->x - point.x), fabs(this->y - point.y));

        return std::max(tmp, fabs(this->z - point.z));
    }

}