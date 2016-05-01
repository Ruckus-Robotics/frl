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

/**
 * Multiplies this RigidBodyTransform by transform and stores the result in this,
 * i.e. this = this*transform
 *
 * @param transform
 */
        void RigidBodyTransform::multiply(const RigidBodyTransform &transform)
        {
            double tmp00 = mat00 * transform.mat00 + mat01 * transform.mat10 + mat02 * transform.mat20;
            double tmp01 = mat00 * transform.mat01 + mat01 * transform.mat11 + mat02 * transform.mat21;
            double tmp02 = mat00 * transform.mat02 + mat01 * transform.mat12 + mat02 * transform.mat22;
            double tmp03 = mat00 * transform.mat03 + mat01 * transform.mat13 + mat02 * transform.mat23 + mat03;

            double tmp10 = mat10 * transform.mat00 + mat11 * transform.mat10 + mat12 * transform.mat20;
            double tmp11 = mat10 * transform.mat01 + mat11 * transform.mat11 + mat12 * transform.mat21;
            double tmp12 = mat10 * transform.mat02 + mat11 * transform.mat12 + mat12 * transform.mat22;
            double tmp13 = mat10 * transform.mat03 + mat11 * transform.mat13 + mat12 * transform.mat23 + mat13;

            double tmp20 = mat20 * transform.mat00 + mat21 * transform.mat10 + mat22 * transform.mat20;
            double tmp21 = mat20 * transform.mat01 + mat21 * transform.mat11 + mat22 * transform.mat21;
            double tmp22 = mat20 * transform.mat02 + mat21 * transform.mat12 + mat22 * transform.mat22;
            double tmp23 = mat20 * transform.mat03 + mat21 * transform.mat13 + mat22 * transform.mat23 + mat23;

            mat00 = tmp00;
            mat01 = tmp01;
            mat02 = tmp02;
            mat03 = tmp03;
            mat10 = tmp10;
            mat11 = tmp11;
            mat12 = tmp12;
            mat13 = tmp13;
            mat20 = tmp20;
            mat21 = tmp21;
            mat22 = tmp22;
            mat23 = tmp23;
        }

/**
 * Multiplies transform1 and transform and puts result into this. this =
 * transform1*transform
 *
 * @param transform1
 * @param transform
 */
        void RigidBodyTransform::multiply(const RigidBodyTransform &transform1, const RigidBodyTransform &transform)
        {
            double tmp00 = transform1.mat00 * transform.mat00 + transform1.mat01 * transform.mat10 + transform1.mat02 * transform.mat20;
            double tmp01 = transform1.mat00 * transform.mat01 + transform1.mat01 * transform.mat11 + transform1.mat02 * transform.mat21;
            double tmp02 = transform1.mat00 * transform.mat02 + transform1.mat01 * transform.mat12 + transform1.mat02 * transform.mat22;
            double tmp03 = transform1.mat00 * transform.mat03 + transform1.mat01 * transform.mat13 + transform1.mat02 * transform.mat23 + transform1.mat03;

            double tmp10 = transform1.mat10 * transform.mat00 + transform1.mat11 * transform.mat10 + transform1.mat12 * transform.mat20;
            double tmp11 = transform1.mat10 * transform.mat01 + transform1.mat11 * transform.mat11 + transform1.mat12 * transform.mat21;
            double tmp12 = transform1.mat10 * transform.mat02 + transform1.mat11 * transform.mat12 + transform1.mat12 * transform.mat22;
            double tmp13 = transform1.mat10 * transform.mat03 + transform1.mat11 * transform.mat13 + transform1.mat12 * transform.mat23 + transform1.mat13;

            double tmp20 = transform1.mat20 * transform.mat00 + transform1.mat21 * transform.mat10 + transform1.mat22 * transform.mat20;
            double tmp21 = transform1.mat20 * transform.mat01 + transform1.mat21 * transform.mat11 + transform1.mat22 * transform.mat21;
            double tmp22 = transform1.mat20 * transform.mat02 + transform1.mat21 * transform.mat12 + transform1.mat22 * transform.mat22;
            double tmp23 = transform1.mat20 * transform.mat03 + transform1.mat21 * transform.mat13 + transform1.mat22 * transform.mat23 + transform1.mat23;

            mat00 = tmp00;
            mat01 = tmp01;
            mat02 = tmp02;
            mat03 = tmp03;
            mat10 = tmp10;
            mat11 = tmp11;
            mat12 = tmp12;
            mat13 = tmp13;
            mat20 = tmp20;
            mat21 = tmp21;
            mat22 = tmp22;
            mat23 = tmp23;
        }

/**
 * Compute the inverse of the RigidBodyTransform passed in as an
 * argument exploiting the orthogonality of the rotation matrix
 * and store the result in this.
 * @param transform
 */
        void RigidBodyTransform::invert(const RigidBodyTransform &transform)
        {
            set(transform);
            invert();
        }

/**
* Inverte this transform.
*/
        void RigidBodyTransform::invert()
        {
            invertOrthogonal();
        }

        void RigidBodyTransform::invertRotationButKeepTranslation()
        {
            double tmp01 = mat01;
            double tmp02 = mat02;
            double tmp12 = mat12;

            // For orthogonal matrix, R^{-1} = R^{T}
            mat01 = mat10;
            mat02 = mat20;
            mat12 = mat21;
            mat10 = tmp01;
            mat20 = tmp02;
            mat21 = tmp12;
        }

/**
 * Check if the elements of this are within epsilon of the elements of
 * transform.
 *
 * @param transform
 * @param epsilon
 * @return
 */
        bool RigidBodyTransform::epsilonEquals(const RigidBodyTransform &transform, const double &epsilon) const
        {
            if (!fabs(mat00 - transform.mat00) < epsilon)
            {
                return false;
            }

            if (!fabs(mat01 - transform.mat01) < epsilon)
            {
                return false;
            }

            if (!fabs(mat02 - transform.mat02) < epsilon)
            {
                return false;
            }

            if (!fabs(mat03 - transform.mat03) < epsilon)
            {
                return false;
            }

            if (!fabs(mat10 - transform.mat10) < epsilon)
            {
                return false;
            }

            if (!fabs(mat11 - transform.mat11) < epsilon)
            {
                return false;
            }

            if (!fabs(mat12 - transform.mat12) < epsilon)
            {
                return false;
            }

            if (!fabs(mat13 - transform.mat13) < epsilon)
            {
                return false;
            }

            if (!fabs(mat20 - transform.mat20) < epsilon)
            {
                return false;
            }

            if (!fabs(mat21 - transform.mat21) < epsilon)
            {
                return false;
            }

            if (!fabs(mat22 - transform.mat22) < epsilon)
            {
                return false;
            }

            if (!fabs(mat23 - transform.mat23) < epsilon)
            {
                return false;
            }

            return true;
        }

/**
 * Returns true if each element of this is equal to each element of
 * transform within a default tolerance of 1e-10.
 *
 * @param transform
 * @return
 */
        bool RigidBodyTransform::equals(const RigidBodyTransform &transform) const
        {
            return epsilonEquals(transform, 1e-10);
        }


/**
 * Transform vector by multiplying it by this transform and put result back
 * into vector.
 *
 * @param vector
 */
        void RigidBodyTransform::transform(Eigen::Vector3d &vector)
        {
            double x = mat00 * vector(0) + mat01 * vector(1) + mat02 * vector(2);
            double y = mat10 * vector(0) + mat11 * vector(1) + mat12 * vector(2);
            vector(2) = mat20 * vector(0) + mat21 * vector(1) + mat22 * vector(2);

            vector(0) = x;
            vector(1) = y;
        }

/**
 * Transform vector by multiplying it by this transform and put result back
 * into vector.
 *
 * @param vector
 */
        void RigidBodyTransform::transform(const Eigen::Vector3d &vectorIn, Eigen::Vector3d &vectorOut)
        {
            vectorOut(0) = mat00 * vectorIn(0) + mat01 * vectorIn(1) + mat02 * vectorIn(2);
            vectorOut(1) = mat10 * vectorIn(0) + mat11 * vectorIn(1) + mat12 * vectorIn(2);
            vectorOut(2) = mat20 * vectorIn(0) + mat21 * vectorIn(1) + mat22 * vectorIn(2);
        }

/**
 * Transform vectorIn using this transform and store result in vectorOut.
 *
 * @param vectorIn
 * @param vectorOut
 */
        void RigidBodyTransform::transform(const Eigen::Vector4d &vectorIn, Eigen::Vector4d &vectorOut)
        {
            vectorOut(0) = mat00 * vectorIn(0) + mat01 * vectorIn(1) + mat02 * vectorIn(2) + mat03;
            vectorOut(1) = mat10 * vectorIn(0) + mat11 * vectorIn(1) + mat12 * vectorIn(2) + mat13;
            vectorOut(2) = mat20 * vectorIn(0) + mat21 * vectorIn(1) + mat22 * vectorIn(2) + mat23;
            vectorOut(3) = 1.0;
        }
/**
 * Transform the Point3d pointIn by this transform and place result in
 * pointOut.
 *
 * @param point
 */
        void RigidBodyTransform::transform(const Point3d &pointIn, Point3d &pointOut)
        {
            pointOut.x = mat00 * pointIn.x + mat01 * pointIn.y + mat02 * pointIn.z + mat03;
            pointOut.y = mat10 * pointIn.x + mat11 * pointIn.y + mat12 * pointIn.z + mat13;
            pointOut.z = mat20 * pointIn.x + mat21 * pointIn.y + mat22 * pointIn.z + mat23;
        }


        Eigen::Vector3d RigidBodyTransform::getTranslationDifference(const RigidBodyTransform &transform1, const RigidBodyTransform &transform2)
        {
            Eigen::Vector3d pos1;
            Eigen::Vector3d pos2;
            transform1.getTranslation(pos1);
            transform2.getTranslation(pos2);

            return (pos2 - pos1);
        }
    }
}