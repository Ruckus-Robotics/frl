#ifndef POINT_3D_HPP
#define POINT_3D_HPP

#include <eigen3/Eigen/Eigen>
#include <vector>

namespace geometry_utilities
{
    class Point3d
    {
    public:
        Point3d(const double &x, const double &y, const double &z);

        Point3d(const std::vector<double> &vector);

        Point3d(const Point3d &point);

        Point3d(const Eigen::Vector3d &vector);

        Point3d(const double array[3]);

        Point3d();

        void set(const double &x, const double &y, const double &z);

        void set(const std::vector<double> &vector);

        void add(const double &x, const double &y, const double &z);

        void subtract(const double &x, const double &y, const double &z);

        void negate();

        void scale(const double &scale);

        void scaleAdd(const double &scale, const Point3d &point);

        bool equals(const Point3d &point);

        bool epsilonEquals(const Point3d &point, const double &epsilon);

        void clampMin(const double &min);

        void clampMax(const double &max);

        void clampMinMax(const double &min, const double &max);

        void absoluteValue();

        double distanceSquared(const Point3d &point) const;

        double distance(const Point3d point) const;

        double distanceL1(const Point3d &point) const;

        double distanceLinf(const Point3d &point) const;

        inline double getX() const
        {
            return this->x;
        };
        inline double getY() const
        {
            return this->y;
        };
        inline double getZ() const
        {
            return this->z;
        };
        inline void setX(double x)
        {
            this->x = x;
        }
        inline void setY(double y)
        {
            this->y = y;
        }
        inline void setZ(double z)
        {
            this->z = z;
        }

        friend std::ostream &operator<<( std::ostream &os, const Point3d &point )
        {
            os << "x: " << point.x << '\n' << "y: " << point.y << '\n' << "z: " << point.z << "\n";
            return os;
        }

        Point3d& operator+=(const Point3d &point)
        {
            this->x += point.x;
            this->y += point.y;
            this->z += point.z;

            return *this;
        }

        Point3d& operator-=(const Point3d &point)
        {
            this->x -= point.x;
            this->y -= point.y;
            this->z -= point.z;

            return *this;
        }

        friend Point3d operator+(Point3d leftHandSide, const Point3d &point)
        {
            leftHandSide.x += point.x;
            leftHandSide.y += point.y;
            leftHandSide.z += point.z;

            return leftHandSide;
        }

        friend Point3d operator-(Point3d leftHandSide, const Point3d &point)
        {
            leftHandSide.x -= point.x;
            leftHandSide.y -= point.y;
            leftHandSide.z -= point.z;

            return leftHandSide;
        }

        double x, y, z;
    };
}

#endif