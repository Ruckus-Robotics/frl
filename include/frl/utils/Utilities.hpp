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

        template<typename T1, typename T2>
        static void clampMin(T1 &x,const T2 min)
        {
            if (x < min)
            {
                x = min;
            }
        }

        template<typename T1, typename T2>
        static void clampMax(T1 &x,const T2 max)
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
        static T distanceLinf(const T p1X,const T p1Y, const T p1Z,const T p2X,const T p2Y, const T p2Z)
        {
            T tmp,dx,dy,dz;

            dx=p1X-p2X;
            dy=p1Y-p2Y;
            dz=p1Z-p2Z;

            tmp = fabs(dx)>fabs(dy) ? fabs(dx) : fabs(dy);

            return tmp>fabs(dz) ? tmp : fabs(dz);
        }

        template<typename T>
        static bool almostZero(const T a, const double epsilon=1e-5)
        {
            return ((a < epsilon) && (a > -epsilon));
        }
    }
}

#endif