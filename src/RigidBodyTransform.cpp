#include <frl/geometry/RigidBodyTransform.hpp>
#include "frl/utils/Utilities.hpp"
#include <iostream>

/**
 *
 * This class creates a 4x4 rigid body transformation matrix. The top
 * left 3x3 is an orthogonal rotation matrix, while the top right 3x1 is a vector
 * describing a translation.
 *
 * T = | xx yx zx px |
 *     | xy yy zy py |
 *     | xz yz zz pz |
 *     | 0 0 0 1 |
 */

/**
 * Set to identity
 */

namespace frl
{
    namespace geometry
    {
        RigidBodyTransform::RigidBodyTransform()
        {
            setIdentity();
        }

        /**
         * Set this transform equal to the RigidBodyTransformed sent as an argument.
         */
        RigidBodyTransform::RigidBodyTransform(const RigidBodyTransform &transform)
        {
            set(transform);
        }

        /**
        * Create transformation matrix from Eigen::Matrix4d
        *
        * @param mat4d
        */
        RigidBodyTransform::RigidBodyTransform(const Eigen::Matrix4d &matrix)
        {
            set(matrix);
        }

        RigidBodyTransform::RigidBodyTransform(const Eigen::Matrix4f &matrix)
        {
            set(matrix);
        }

        /**
         * Create transformation matrix from rotation matrix and vector translation
         *
         * @param matrix
         * @param vector
         */
        RigidBodyTransform::RigidBodyTransform(const Eigen::Matrix3d &matrix, const Eigen::Vector3d &vector)
        {
            set(matrix, vector);
        }

        /**
         * Create transformation matrix from rotation matrix and vector translation
         *
         * @param matrix
         * @param vector
         */
        RigidBodyTransform::RigidBodyTransform(const Eigen::Matrix3f &matrix, const Eigen::Vector3f &vector)
        {
            set(matrix, vector);
        }

/**
 * Create RigidBodyTransform from quaternion describing a rotation and vector
 * describing a translation.
 *
 * @param Eigen::Quaternion<double>
 * @param Eigen::Vector3d
 */
        RigidBodyTransform::RigidBodyTransform(const Eigen::Quaternion<double> &quaternion, const Eigen::Vector3d &vector)
        {
            set(quaternion, vector);
        }

/**
* Construct a RigidBodyTransform from a rotation matrix, setting the translational vector to zero.
*
* @param Eigen::Matrix3d
*/

        RigidBodyTransform::RigidBodyTransform(const Eigen::Matrix3d &matrix)
        {
            setRotation(matrix);
            setTranslation(0.0, 0.0, 0.0);
        }

/**
* Constructs a RigidBodyTransfor with rotation element described by quaternion and
* zero translational component.
*
* @param Quaternion
*/

        RigidBodyTransform::RigidBodyTransform(const Eigen::Quaternion<double> &quat)
        {
            setRotation(quat);
            setTranslation(0.0, 0.0, 0.0);
        }

/**
* Construct a RigidBodyTransform with rotation element described by axisAngle and
* zero translational component.
*
*/
        RigidBodyTransform::RigidBodyTransform(const Eigen::AngleAxis<double> &axisAngle)
        {
            setRotation(axisAngle);
            setTranslation(0.0, 0.0, 0.0);
        }

        void RigidBodyTransform::setRotation(const Eigen::AngleAxis<double> &axisAngle)
        {
            setRotationWithAxisAngle(axisAngle.axis()[0], axisAngle.axis()[1], axisAngle.axis()[2], axisAngle.angle());
        }

        void RigidBodyTransform::setRotationWithAxisAngle(const double &axisAngleX, const double &axisAngleY, const double &axisAngleZ, const double &axisAngleTheta)
        {

        }

        void RigidBodyTransform::setRotation(const Eigen::Quaternion<double> &quat)
        {
            setRotationWithQuaternion(quat.x(), quat.y(), quat.z(), quat.w());
        }

        void RigidBodyTransform::setRotationWithQuaternion(const double &qx, const double &qy, const double &qz, const double &qw)
        {
            double yy2 = 2.0 * qy * qy;
            double zz2 = 2.0 * qz * qz;
            double xx2 = 2.0 * qx * qx;
            double xy2 = 2.0 * qx * qy;
            double wz2 = 2.0 * qw * qz;
            double xz2 = 2.0 * qx * qz;
            double wy2 = 2.0 * qw * qy;
            double yz2 = 2.0 * qy * qz;
            double wx2 = 2.0 * qw * qx;

            this->mat00 = (1.0 - yy2 - zz2);
            this->mat01 = (xy2 - wz2);
            this->mat02 = (xz2 + wy2);
            this->mat10 = (xy2 + wz2);
            this->mat11 = (1.0 - xx2 - zz2);
            this->mat12 = (yz2 - wx2);
            this->mat20 = (xz2 - wy2);
            this->mat21 = (yz2 + wx2);
            this->mat22 = (1.0 - xx2 - yy2);
        }

/**
 * Set the 3x3 rotation matrix equal to mat3d.
 *
 * @param matrix
 */
        void RigidBodyTransform::setRotation(const Eigen::Matrix3d &matrix)
        {

        }

/**
*  Add a translation to the current transform. It is equivalent to:
*
*      transform.setTranslationAndIdentityRotation(translation);
*      this = this*transform
*/
        void RigidBodyTransform::applyTranslation(const Eigen::Vector3d &translation)
        {
            Point3d<double> temp(translation(0),translation(1),translation(2));
            transform(temp);
            mat03 = temp.x;
            mat13 = temp.y;
            mat23 = temp.z;
        }

        /**
         * Set this transform to have translation described in vector
         * and a rotation equal to the Eigen::Matrix3d matrix.
         *
         * @param Eigen::Matrix3f matrix
         * @param Eigen::Vector3f vector
         */
        void RigidBodyTransform::set(const Eigen::Matrix3f &matrix, const Eigen::Vector3f &vector)
        {
            setRotation(matrix);
            setTranslation(vector);
        }

/**
 * Set this transform to have zero translation and a rotation equal to the
 * Eigen::Quaternion<double> quat.
 *
 * @param Eigen::Quaternion<double> quat
 */
        void RigidBodyTransform::setRotationAndZeroTranslation(const Eigen::Quaternion<double> &quat)
        {
            setRotation(quat);
            setTranslation(0, 0, 0);
        }

// /**
//  * Set this transform to have translation described in vector and a rotation
//  * equal to the Eigen::Quaternion<double> quat.
//  *
//  * @param Eigen::Quaternion<double> quat
//  */
        void RigidBodyTransform::set(const Eigen::Quaternion<double> &quat, const Eigen::Vector3d &vector)
        {
            setRotation(quat);
            setTranslation(vector);
        }

/**
 * Sets this transform to have rotation described by axisAngle and zero
 * translation.
 *
 * @param axisAngle
 */
        void RigidBodyTransform::setRotationAndZeroTranslation(const Eigen::AngleAxis<double> &axisAngle)
        {
            setRotation(axisAngle);
            setTranslation(0, 0, 0);
        }

        void RigidBodyTransform::set(const Eigen::AngleAxis<double> &axisAngle, const Eigen::Vector3d &vector)
        {
            setRotation(axisAngle);
            setTranslation(vector(0), vector(1), vector(2));
        }

/**
 * Set this transform to have zero translation
 */
        void RigidBodyTransform::zeroTranslation()
        {
            setTranslation(0, 0, 0);
        }

/**
 * Set this transform to have zero translation and a rotation equal to the
 * Matrix3f matrix.
 *
 * @param matrix
 */
        void RigidBodyTransform::setRotationAndZeroTranslation(const Eigen::Matrix3d &matrix)
        {
            setRotation(matrix);
            setTranslation(0, 0, 0);
        }


        bool RigidBodyTransform::isRotationMatrixEpsilonIdentity(const double &epsilon) const
        {

            return fabs(mat01) < epsilon && fabs(mat02) < epsilon && fabs(mat10) < epsilon && fabs(mat12) < epsilon && fabs(mat20) < epsilon &&
                   fabs(mat21) < epsilon && fabs(mat00 - 1.0) < epsilon && fabs(mat11 - 1.0) < epsilon && fabs(mat22 - 1.0) - epsilon;
        }


        void RigidBodyTransform::getTranslation(Eigen::Vector3d &vector) const
        {

        }


/**
 * Pack rotation part into Eigen::Matrix3d and translation part into Eigen::Vector3d
 *
 * @param matrix
 * @param vector
 */
        void RigidBodyTransform::get(Eigen::Matrix3d &matrix, Eigen::Vector3d &vector) const
        {
            getRotation(matrix);
            getTranslation(vector);
        }

/**
 * Return rotation portion of this transform.
 *
 * @param matrix
 */
        void RigidBodyTransform::get(Eigen::Matrix3d &matrix) const
        {
            getRotation(matrix);
        }

/**
 * Return translational portion of this transform.
 *
 * @param vector
 */
        void RigidBodyTransform::get(Eigen::Vector3d &vector) const
        {
            getTranslation(vector);
        }

/**
 * Convert and pack rotation part of transform into Eigen::Quaternion<double> and pack
 * translation into Eigen::Vector3d.
 *
 * @param quat
 * @param vector
 */
        void RigidBodyTransform::get(Eigen::Quaternion<double> &quat, Eigen::Vector3d &vector) const
        {
            getRotation(quat);
            getTranslation(vector);
        }

/**
 * Convert and pack rotation part of transform into Eigen::Quaternion<double> and pack
 * translation into Point3d.
 *
 * @param quat
 * @param point
 */
        void RigidBodyTransform::get(Eigen::Quaternion<double> &quat, Point3d &point) const
        {
            getRotation(quat);
            getTranslation(point);
        }

/**
 * Convert and pack rotation part of transform into Eigen::Quaternion<double>.
 *
 * @param quat
 * @param vector
 */
        void RigidBodyTransform::get(Eigen::Quaternion<double> &quat) const
        {
            getRotation(quat);
        }

    }
}