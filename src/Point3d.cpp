//#include "frl/geometry/Point3d.hpp"
//
//namespace frl
//{
//
//    namespace geometry
//    {
//
//        Point3d::Point3d(const double &x, const double &y, const double &z)
//        {
//            set(x, y, z);
//        }

//        bool Point3d::epsilonEquals(const Point3d &tuple, const double &epsilon)
//        {
//            if (std::isnan(this->x) || std::isnan(this->y) || std::isnan(this->z) || std::isnan(tuple.x) || std::isnan(tuple.y) || std::isnan(tuple.z))
//            {
//                return false;
//            }
//
//            return (fabs(this->x - tuple.x) < epsilon && fabs(this->y - tuple.y) < epsilon && fabs(this->z - tuple.z) < epsilon);
//        }
//
//        bool Point3d::equals(const Point3d &tuple)
//        {
//            if (std::isnan(this->x) || std::isnan(this->y) || std::isnan(this->z) || std::isnan(tuple.x) || std::isnan(tuple.y) || std::isnan(tuple.z))
//            {
//                return false;
//            }
//
//
//        }
//

//
//        void Point3d::clampMax(const double &max)
//        {
//            if (this->x > max)
//            {
//                this->x = max;
//            }
//
//            if (this->y > max)
//            {
//                this->y = max;
//            }
//
//            if (this->z > max)
//            {
//                this->z = max;
//            }
//        }
//
//        void Point3d::clampMinMax(const double &min, const double &max)
//        {
//            if (min > max)
//            {
//                throw std::runtime_error("Invalid bounds!");
//            }
//
//            if (this->x < min)
//            {
//                this->x = min;
//            }
//            else if (this->x > max)
//            {
//                this->x = max;
//            }
//
//            if (this->y < min)
//            {
//                this->y = min;
//            }
//            else if (this->y > max)
//            {
//                this->y = max;
//            }
//
//            if (this->z < min)
//            {
//                this->z = min;
//            }
//            else if (this->z > max)
//            {
//                this->z = max;
//            }
//        }
//
//        void Point3d::absoluteValue()
//        {

//        }
//
//        double Point3d::distanceSquared(const Point3d &point) const
//        {
//            double dx, dy, dz;
//
//            dx = this->x - point.x;
//            dy = this->y - point.y;
//            dz = this->z - point.z;
//            return (dx * dx + dy * dy + dz * dz);
//        }
//

//    }
//
//}