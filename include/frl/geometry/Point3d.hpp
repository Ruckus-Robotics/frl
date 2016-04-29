#ifndef POINT_3D_HPP
#define POINT_3D_HPP

#include <eigen3/Eigen/Eigen>
#include <vector>
#include <math.h>
#include "frl/utils/Utilities.hpp"
#include <stdexcept>

namespace frl
{
    namespace geometry
    {
        template<class T>
        class Point3d
        {
        public:
            Point3d(const T x, const T y, const T z)
            {
                set(x, y, z);
            }

            Point3d(const std::vector<T> &vector)
            {
                set(vector[0], vector[1], vector[2]);
            }

            Point3d(const Point3d &point)
            {
                set(point.x, point.y, point.z);
            }

            Point3d(const T array[3])
            {
                set(array[0], array[1], array[2]);
            }

            Point3d()
            {
                set(0.0, 0.0, 0.0);
            }

            ~Point3d()
            { };

            void set(const std::vector<T> &vector)
            {
                if(vector.size() != 3)
                {
                    throw std::runtime_error("Vector must be size 3!");
                }
                set(vector[0], vector[1], vector[2]);
            }

            void set(const T x, const T y, const T z)
            {
                this->x = x;
                this->y = y;
                this->z = z;
            }

            void add(const T &x, const T &y, const T &z)
            {
                this->x += x;
                this->y += y;
                this->z += z;
            }

            void subtract(const T &x, const T &y, const T &z)
            {
                this->x -= x;
                this->y -= y;
                this->z -= z;
            }

            void negate()
            {
                this->x *= -1;
                this->y *= -1;
                this->z *= -1;
            }

            void scale(const T &scale)
            {
                this->x *= scale;
                this->y *= scale;
                this->z *= scale;
            }

            void scaleAdd(const T &scale, const Point3d &point)
            {
                this->x *= scale;
                this->y *= scale;
                this->z *= scale;

                this->x += point.x;
                this->y += point.y;
                this->z += point.z;
            }

            bool equals(const Point3d &point)
            {
                return (this->x == point.x && this->y == point.y && this->z == point.z);
            }

            bool epsilonEquals(const Point3d &point, const T epsilon)
            {
                return (fabs(this->x - point.x) < epsilon && fabs(this->y - point.y) < epsilon && fabs(this->z - point.z) < epsilon);
            }

            void clampMin(const T min)
            {
                frl::utils::clampMin(x,min);
                frl::utils::clampMin(y,min);
                frl::utils::clampMin(z,min);
            }

            void clampMax(const T max)
            {
                frl::utils::clampMax(x,max);
                frl::utils::clampMax(y,max);
                frl::utils::clampMax(z,max);
            }

            void clampMinMax(const T min, const T max)
            {
                clampMin(min);
                clampMax(max);
            }

            void absoluteValue()
            {
                this->x = fabs(this->x);
                this->y = fabs(this->y);
                this->z = fabs(this->z);
            }

            T distanceSquared(const Point3d &point) const
            {
                return frl::utils::computeDistanceBetweenPointsSquared(x,y,z,point.x,point.y,point.z);
            }

            T distance(const Point3d point) const
            {
                return frl::utils::computeDistanceBetweenPoints(x,y,z,point.x,point.y,point.z);
            }

            T distanceL1(const Point3d &point) const
            {
                return frl::utils::distanceL1(x,y,z,point.x,point.y,point.z);
            }

            T distanceLinf(const Point3d &point) const
            {
                return frl::utils::distanceLinf(x,y,z,point.x,point.y,point.z);
            }

            inline T getX() const
            {
                return this->x;
            };

            inline T getY() const
            {
                return this->y;
            };

            inline T getZ() const
            {
                return this->z;
            };

            inline void setX(T x)
            {
                this->x = x;
            }

            inline void setY(T y)
            {
                this->y = y;
            }

            inline void setZ(T z)
            {
                this->z = z;
            }

            friend std::ostream &operator<<(std::ostream &os, const Point3d &point)
            {
                os << "x: " << point.x << '\n' << "y: " << point.y << '\n' << "z: " << point.z << "\n";
                return os;
            }

            Point3d &operator+=(const Point3d &point)
            {
                this->x += point.x;
                this->y += point.y;
                this->z += point.z;

                return *this;
            }

            Point3d &operator-=(const Point3d &point)
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

            T x, y, z;
        };
    }

}
#endif