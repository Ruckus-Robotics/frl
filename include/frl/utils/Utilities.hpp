//
// Created by jordan on 4/28/16.
//

#ifndef FRL_UTILITIES_HPP
#define FRL_UTILITIES_HPP

#include <math.h>
#include <algorithm>

namespace frl
{
    namespace utils
    {
        template<typename T>
        static T computeDistanceBetweenPointsSquared(const T p1X,const T p1Y, const T p1Z,const T p2X,const T p2Y, const T p2Z)
        {
            T dx = p1X - p2X;
            T dy = p1Y - p2Y;
            T dz = p1Z - p2Z;
            return (dx * dx + dy * dy + dz * dz);
        }

        template<typename T>
        static T computeDistanceBetweenPoints(const T p1X,const T p1Y, const T p1Z,const T p2X,const T p2Y, const T p2Z)
        {
            return sqrt(computeDistanceBetweenPointsSquared(p1X,p1Y,p1Z,p2X,p2Y,p2Z));
        }

        template<typename T>
        static void clampMin(T &x,const T min)
        {
            if (x < min)
            {
                x = min;
            }
        }

        template<typename T>
        static void clampMax(T &x,const T max)
        {
            if (x > max)
            {
                x = max;
            }
        }

        template<typename T>
        static T distanceL1(const T p1X,const T p1Y, const T p1Z,const T p2X,const T p2Y, const T p2Z)
        {
            return (fabs(p1X - p2X) + fabs(p1Y - p2Y) +
                    fabs(p1Z - p2Z));
        }

        template<typename T>
        T distanceLinf(const T p1X,const T p1Y, const T p1Z,const T p2X,const T p2Y, const T p2Z)
        {
            T tmp;
            tmp = std::max(fabs(p1X - p2X), fabs(p1Y - p2Y));

            return std::max(tmp, fabs(p1Z - p2Z));
        }

        template<typename T>
        static bool almostZero(const T a, const double epsilon=1e-5)
        {
            return ((a < epsilon) && (a > -epsilon));
        }
    }
}

#endif