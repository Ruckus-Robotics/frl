#ifndef RIGID_BODY_TRANSFORM_HPP
#define RIGID_BODY_TRANSFORM_HPP

#include <eigen3/Eigen/Eigen>
#include "frl/geometry/Point3.hpp"
#include "../../../../../../../../../usr/include/eigen3/Eigen/src/Core/Matrix.h"

namespace frl
{
	namespace geometry
	{
		template<class T>
		class RigidBodyTransform
        {
        public:
            RigidBodyTransform();

            template<class TYPE>
            RigidBodyTransform(const RigidBodyTransform<TYPE> &transform)
            {
                set(transform);
            }

            template<class TYPE>
            RigidBodyTransform(const Eigen::Matrix<TYPE, 4, 4> &matrix)
            {
                set(matrix);
            }

            template<class T1, class T2>
            RigidBodyTransform(const Eigen::Matrix<T1, 3, 3> &matrix, const Eigen::Matrix<T2, 3, 1> &vector)
            {
                set(matrix, vector);
            };

            template<class T1, class T2>
            RigidBodyTransform(const Eigen::AngleAxis<T1> &axisAngle, const Eigen::Matrix<T2, 3, 1> &vector)
            {
                set(axisAngle, vector);
            }

            template<class T1, class T2>
            RigidBodyTransform(const Eigen::Quaternion<T1> &quaternion, const Eigen::Matrix<T2, 3, 1> &vector)
            {
                set(quaternion, vector);
            };

            template<class TYPE>
            RigidBodyTransform(const Eigen::Matrix<TYPE, 3, 3> &matrix)
            {
                setRotation(matrix);
                setTranslation(0.0, 0.0, 0.0);
            }

            template<class TYPE>
            RigidBodyTransform(const Eigen::Quaternion<TYPE> &quat)
            {
                setRotation(quat);
                setTranslation(0.0, 0.0, 0.0);
            }

            template<class TYPE>
            RigidBodyTransform(const Eigen::AngleAxis<TYPE> &axisAngle)
            {
                setRotation(axisAngle);
                setTranslation(0.0, 0.0, 0.0);
            }

            ~RigidBodyTransform()
            { };

            /**
            * Set transformation matrix to Identity, meaning no rotation or
            * translation.
            */
            void setIdentity()
            {
                this->mat00 = 1.0;
                this->mat01 = 0.0;
                this->mat02 = 0.0;
                this->mat03 = 0.0;
                this->mat10 = 0.0;
                this->mat11 = 1.0;
                this->mat12 = 0.0;
                this->mat13 = 0.0;
                this->mat20 = 0.0;
                this->mat21 = 0.0;
                this->mat22 = 1.0;
                this->mat23 = 0.0;
            }

            template<class TYPE>
            void set(const RigidBodyTransform<TYPE> &transform)
            {
                this->mat00 = transform.mat00;
                this->mat01 = transform.mat01;
                this->mat02 = transform.mat02;
                this->mat03 = transform.mat03;
                this->mat10 = transform.mat10;
                this->mat11 = transform.mat11;
                this->mat12 = transform.mat12;
                this->mat13 = transform.mat13;
                this->mat20 = transform.mat20;
                this->mat21 = transform.mat21;
                this->mat22 = transform.mat22;
                this->mat23 = transform.mat23;
            }

            /**
             * Set elements of transform equal to elements of the Eigen::Matrix4d.
             *
             * @param matrix
             */
            template<class TYPE>
            void set(const Eigen::Matrix<TYPE, 4, 4> &matrix)
            {
                this->mat00 = matrix(0, 0);
                this->mat01 = matrix(0, 1);
                this->mat02 = matrix(0, 2);
                this->mat03 = matrix(0, 3);
                this->mat10 = matrix(1, 0);
                this->mat11 = matrix(1, 1);
                this->mat12 = matrix(1, 2);
                this->mat13 = matrix(1, 3);
                this->mat20 = matrix(2, 0);
                this->mat21 = matrix(2, 1);
                this->mat22 = matrix(2, 2);
                this->mat23 = matrix(2, 3);
            }

            /**
             * Set elements of the transform
             *
             * @param matrix
             */
            template<typename TYPE>
            void set(const Eigen::Matrix<TYPE, 4, 4> &matrix)
            {
                this->mat00 = matrix(0, 0);
                this->mat01 = matrix(0, 1);
                this->mat02 = matrix(0, 2);
                this->mat03 = matrix(0, 3);
                this->mat10 = matrix(1, 0);
                this->mat11 = matrix(1, 1);
                this->mat12 = matrix(1, 2);
                this->mat13 = matrix(1, 3);
                this->mat20 = matrix(2, 0);
                this->mat21 = matrix(2, 1);
                this->mat22 = matrix(2, 2);
                this->mat23 = matrix(2, 3);
            }

            /**
            * Set this transform to have translation described in vector
            * and a rotation equal to matrix.
            *
            * @param Eigen::Matrix matrix
            * @param Eigen::Matrix vector
            */
            template<class T1, class T2>
            void set(const Eigen::Matrix<T1, 3, 3> &matrix, const Eigen::Matrix<T2, 3, 1> &vector)
            {
                setRotation(matrix);
                setTranslation(vector);
            }

            /**
            * Set this transform to have translation described in vector
            * and a rotation equal to axisAngles.
            *
            * @param Eigen::AxisAngle axisAngle
            * @param Eigen::Matrix vector
            */
            template<class T1, class T2>
            void set(const Eigen::AngleAxis<T1> &axisAngle, const Eigen::Matrix<T2, 3, 1> &vector)
            {
                setRotation(axisAngle);
                setTranslation(vector);
            }

            /**
            * Set this transform to have translation described in vector
            * and a rotation equal to quat.
            *
            * @param Eigen::Quaternion quat
            * @param Eigen::Matrix vector
            */
            template<class T1, class T2>
            void set(const Eigen::Quaternion<T1> &quat, const Eigen::Matrix<T2, 3, 1> &vector)
            {
                setRotation(quat);
                setTranslation(vector);
            }


            /**
             * Set translational portion of the transformation matrix
             *
             * @param x The x-component of the translation
             * @param y The y-component of the translation
             * @param z The z-component of the translation
             */
            template<typename TYPE>
            void setTranslation(const TYPE x, const TYPE y, const TYPE z)
            {
                this->mat03 = x;
                this->mat13 = y;
                this->mat23 = z;
            }

            /**
             * Set translational portion of the transformation matrix
             *
             * @param vector
             */
            template<class TYPE>
            void setTranslation(const Eigen::Matrix<TYPE, 3, 1> &vector)
            {
                setTranslation(vector(0), vector(1), vector(2));
            }

            /**
            * This method is for when the matrix is column major and needs to
            * be transposed.
            *
            * @param matrix
            */
            template<class TYPE>
            void setAsTranspose(const Eigen::Matrix<TYPE, 4, 4> &matrix)
            {
                double tmp10 = matrix(1, 0);
                double tmp20 = matrix(2, 0);
                double tmp21 = matrix(2, 1);
                double tmp30 = matrix(3, 0);
                double tmp31 = matrix(3, 1);
                double tmp32 = matrix(3, 2);

                mat00 = matrix(0, 0);
                mat11 = matrix(1, 1);
                mat22 = matrix(2, 2);
                mat10 = matrix(0, 1);
                mat20 = matrix(0, 2);
                mat21 = matrix(1, 2);
                mat01 = tmp10;
                mat03 = tmp30;
                mat13 = tmp31;
                mat23 = tmp32;
                mat02 = tmp20;
                mat12 = tmp21;
            }

            void zeroTranslation()
            {
                mat03 = 0.0;
                mat13 = 0.0;
                mat23 = 0.0;
            }

            template<class TYPE>
            void setRotation(const Eigen::Matrix<TYPE, 3, 3> &matrix)
            {
                this->mat00 = matrix(0, 0);
                this->mat01 = matrix(0, 1);
                this->mat02 = matrix(0, 2);
                this->mat10 = matrix(1, 0);
                this->mat11 = matrix(1, 1);
                this->mat12 = matrix(1, 2);
                this->mat20 = matrix(2, 0);
                this->mat21 = matrix(2, 1);
                this->mat22 = matrix(2, 2);
            }

            template<class TYPE>
            void setRotation(const Eigen::Quaternion<TYPE> &quat)
            {
                setRotationWithQuaternion(quat.x(),quat.y(),quat.z(),quat.w());
            }

            template<class TYPE>
			void setRotation(const Eigen::AngleAxis<TYPE> &axisAngle)
            {
                setRotationWithAxisAngle(axisAngle.axis()[0], axisAngle.axis()[1], axisAngle.axis()[2], axisAngle.angle());
            }

			template<typename TYPE>
			void setRotationWithQuaternion(const TYPE qx, const TYPE qy, const TYPE qz, const TYPE qw)
            {
                TYPE yy2 = 2.0 * qy * qy;
                TYPE zz2 = 2.0 * qz * qz;
                TYPE xx2 = 2.0 * qx * qx;
                TYPE xy2 = 2.0 * qx * qy;
                TYPE wz2 = 2.0 * qw * qz;
                TYPE xz2 = 2.0 * qx * qz;
                TYPE wy2 = 2.0 * qw * qy;
                TYPE yz2 = 2.0 * qy * qz;
                TYPE wx2 = 2.0 * qw * qx;

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

			template<typename TYPE>
			void setRotationWithAxisAngle(const TYPE axisAngleX, const TYPE axisAngleY, const TYPE &axisAngleZ, const TYPE axisAngleTheta)
            {
                TYPE mag = sqrt(axisAngleX * axisAngleX + axisAngleY * axisAngleY + axisAngleZ * axisAngleZ);

                if (frl::utils::almostZero(mag))
                {
                    setIdentity();
                }
                else
                {
                    mag = 1.0 / mag;
                    TYPE ax = axisAngleX * mag;
                    TYPE ay = axisAngleY * mag;
                    TYPE az = axisAngleZ * mag;

                    TYPE sinTheta = sin(axisAngleTheta);
                    TYPE cosTheta = cos(axisAngleTheta);
                    TYPE t = 1.0 - cosTheta;

                    TYPE xz = ax * az;
                    TYPE xy = ax * ay;
                    TYPE yz = ay * az;

                    mat00 = (t * ax * ax + cosTheta);
                    mat01 = (t * xy - sinTheta * az);
                    mat02 = (t * xz + sinTheta * ay);

                    mat10 = (t * xy + sinTheta * az);
                    mat11 = (t * ay * ay + cosTheta);
                    mat12 = (t * yz - sinTheta * ax);

                    mat20 = (t * xz - sinTheta * ay);
                    mat21 = (t * yz + sinTheta * ax);
                    mat22 = (t * az * az + cosTheta);
                }
            }

            template<class TYPE>
			void setRotationAndZeroTranslation(const Eigen::Matrix<TYPE,3,3> &matrix)
            {
                setRotation(matrix);
                setTranslation(0.0,0.0,0.0);
            }

            template<class TYPE>
			void setRotationAndZeroTranslation(const Eigen::Quaternion<TYPE> &quat)
            {
                setRotation(quat);
                setTranslation(0.0,0.0,0.0);
            }

            template<class TYPE>
			void setRotationAndZeroTranslation(const Eigen::AngleAxis<TYPE> &axisAngle)
            {
                setRotation(axisAngle);
                setTranslation(0.0,0.0,0.0);
            }

            /**
            * Set this transform to have an identity rotation and a translation given
            * by the Eigen::Vector3d vector.
            *
            * @param vector
            */
            template<class TYPE>
			void setTranslationAndIdentityRotation(const Eigen::Matrix<TYPE,3,1> &vector)
            {
                setTranslation(vector(0), vector(1), vector(2));
                setRotationToIdentity();
            }

            /**
            * Sets rotation to the identity, does not effect the translational component of the Transform
            *
            */
			void setRotationToIdentity()
            {
                this->mat00 = 1.0;
                this->mat01 = 0.0;
                this->mat02 = 0.0;
                this->mat10 = 0.0;
                this->mat11 = 1.0;
                this->mat12 = 0.0;
                this->mat20 = 0.0;
                this->mat21 = 0.0;
                this->mat22 = 1.0;
            }

            /**
            * Set the rotational component of the transform to the rotation matrix
            * created given an X-Y-Z rotation described by the angles in vector which
            * describe angles of rotation about the X, Y, and Z axis, respectively. The
            * orientation of each rotation is not effected by any of the other
            * rotations. This method sets the translational component of this
            * transform3d to zeros.
            *
            * @param vector
            */
            template<class TYPE>
			void setEulerXYZ(const Eigen::Matrix<TYPE,3,1> &vector)
            {
                setEulerXYZ(vector(0), vector(1), vector(2));
            }

            /**
             * Set the rotational component of the transform to the rotation matrix
             * created given an X-Y-Z rotation described by the angles in vector which
             * describe angles of rotation about the X, Y, and Z axis, respectively. The
             * orientation of each rotation is not effected by any of the other
             * rotations. This method sets the translational component of this
             * transform3d to zeros.
             *
             * @param rotX
             * @param rotY
             * @param rotZ
             */
            template<typename T>
			void setEulerXYZ(const T rotX, const T rotY, const T rotZ)
            {
                double sina = sin(rotX);
                double sinb = sin(rotY);
                double sinc = sin(rotZ);
                double cosa = cos(rotX);
                double cosb = cos(rotY);
                double cosc = cos(rotZ);

                this->mat00 = cosb * cosc;
                this->mat01 = -(cosa * sinc) + (sina * sinb * cosc);
                this->mat02 = (sina * sinc) + (cosa * sinb * cosc);
                this->mat10 = cosb * sinc;
                this->mat11 = (cosa * cosc) + (sina * sinb * sinc);
                this->mat12 = -(sina * cosc) + (cosa * sinb * sinc);
                this->mat20 = -sinb;
                this->mat21 = sina * cosb;
                this->mat22 = cosa * cosb;
                this->mat03 = 0.0;
                this->mat13 = 0.0;
                this->mat23 = 0.0;
            }

            /**
            * Computes the RPY angles from the rotation matrix for rotations about the
            * X, Y, and Z axes respectively. Note that this method is here for the
            * purpose of unit testing the method setEuler. This particular solution is
            * only valid for -pi/2 < vector.y < pi/2 and for vector.y != 0.
            *
            * @param vector
            */
            template<typename TYPE>
			void getEulerXYZ(Eigen::Matrix<TYPE,3,1> &vector) const
            {
                vector(0) = atan2(mat21, mat22);
                vector(1) = atan2(-mat20, sqrt(mat21 * mat21 + mat22 * mat22));
                vector(2) = atan2(mat10, mat00);
            }

            /**
            * Return rotation matrix
            *
            * @param matrix
            */
            template<class TYPE>
			void getRotation(Eigen::Matrix<TYPE,3,3> &matrix) const
            {
                matrix(0, 0) = mat00;
                matrix(0, 1) = mat01;
                matrix(0, 2) = mat02;
                matrix(1, 0) = mat10;
                matrix(1, 1) = mat11;
                matrix(1, 2) = mat12;
                matrix(2, 0) = mat20;
                matrix(2, 1) = mat21;
                matrix(2, 2) = mat22;
            }

            /**
            * Return rotation in quaternion form.
            *
            * @param Eigen::Quaternion quat
            */
            template<class TYPE>
			void getRotation(Eigen::Quaternion<TYPE> &quat) const
            {
                TYPE trace = mat00 + mat11 + mat22;
                TYPE val;

                if (trace > 0.0)
                {
                    val = sqrt(trace + 1.0) * 2.0;
                    quat.x() = (mat21 - mat12) / val;
                    quat.y() = (mat02 - mat20) / val;
                    quat.z() = (mat10 - mat01) / val;
                    quat.w() = 0.25 * val;
                }
                else if (mat11 > mat22)
                {
                    TYPE temp = std::max(0.0, 1.0 + mat11 - mat00 - mat22);
                    val = sqrt(temp) * 2.0;
                    quat.x() = (mat01 + mat10) / val;
                    quat.y() = 0.25 * val;
                    quat.z() = (mat12 + mat21) / val;
                    quat.w() = (mat02 - mat20) / val;
                }
                else if ((mat00 > mat11) && (mat00 > mat22))
                {
                    val = sqrt(1.0 + mat00 - mat11 - mat22) * 2.0;
                    quat.x() = 0.25 * val;
                    quat.y() = (mat01 + mat10) / val;
                    quat.z() = (mat02 + mat20) / val;
                    quat.w() = (mat21 - mat12) / val;
                }
                else
                {
                    val = sqrt(1.0 + mat22 - mat00 - mat11) * 2.0;
                    quat.x() = (mat02 + mat20) / val;
                    quat.y() = (mat12 + mat21) / val;
                    quat.z() = 0.25 * val;
                    quat.w() = (mat10 - mat01) / val;
                }
            }

            /**
            * Return rotation in AxisAngle form.
            *
            * @param axisAngle
            */
            template<class TYPE>
			void getRotation(Eigen::AngleAxis<TYPE> &axisAngle) const
            {
                getRotation(axisAngle,1.0e-12);
            }

            /**
            * Return translational part
            *
            * @param vector
            */
            template<class TYPE>
			void getTranslation(Eigen::Matrix<TYPE,3,1> &vector) const
            {
                vector(0) = mat03;
                vector(1) = mat13;
                vector(2) = mat23;
            }

            /**
            * Return translational part
            *
            * @param point
            */
            template<class TYPE>
			void getTranslation(Point3<TYPE> &point) const
            {
                point.x = mat03;
                point.y = mat13;
                point.z = mat23;
            }

            /**
            * Pack transform into matrix
            *
            * @param matrix
            */
            template<class TYPE>
			void get(Eigen::Matrix<TYPE,4,4> &matrix) const
            {
                matrix(0, 0) = mat00;
                matrix(0, 1) = mat01;
                matrix(0, 2) = mat02;
                matrix(0, 3) = mat03;

                matrix(1, 0) = mat10;
                matrix(1, 1) = mat11;
                matrix(1, 2) = mat12;
                matrix(1, 3) = mat13;

                matrix(2, 0) = mat20;
                matrix(2, 1) = mat21;
                matrix(2, 2) = mat22;
                matrix(2, 3) = mat23;

                matrix(3, 0) = 0;
                matrix(3, 1) = 0;
                matrix(3, 2) = 0;
                matrix(3, 3) = 1;
            }

            template<class TYPE>
			void get(Eigen::Matrix<TYPE,3,3> &matrix, Eigen::Matrix<TYPE,3,1> &vector) const
            {
                getRotation(matrix);
                getTranslation(vector);
            }

            template<class TYPE>
			void get(Eigen::Quaternion<TYPE> &quat, Eigen::Matrix<TYPE,3,1> &vector) const
            {
                getRotation(quat);
                getTranslation(vector);
            }

            template<class TYPE>
			void get(Eigen::Quaternion<TYPE> &quat, Point3<TYPE> &point) const
            {
                getRotation(quat);
                getTranslation(point);
            }

            template<class TYPE>
			void applyTranslation(const Eigen::Matrix<TYPE,3,1> &translation)
            {
                Point3d<TYPE> temp(translation(0),translation(1),translation(2));
                transform(temp);
                mat03 = temp.x;
                mat13 = temp.y;
                mat23 = temp.z;
            }

            /**
            * Transform the Point3d point by this transform and place result back in
            * point.
            *
            * @param point
            */
            template<class TYPE>
			void transform(Point3<TYPE> &point)
            {
                TYPE x = mat00 * point.x + mat01 * point.y + mat02 * point.z + mat03;
                TYPE y = mat10 * point.x + mat11 * point.y + mat12 * point.z + mat13;
                point.z = mat20 * point.x + mat21 * point.y + mat22 * point.z + mat23;

                point.x = x;
                point.y = y;
            }

            /**
            * Transform vector by multiplying it by this transform and put result back
            * into vector.
            *
            * @param vector
            */
            template<class TYPE>
			void transform(Eigen::Matrix<TYPE,4,1> &vector)
            {
                if (vector(3) != 1.0)
                {
                    throw std::runtime_error("Final element of vector must be 1.");
                }

                TYPE x = mat00 * vector(0) + mat01 * vector(1) + mat02 * vector(2) + mat03;
                TYPE y = mat10 * vector(0) + mat11 * vector(1) + mat12 * vector(2) + mat13;
                vector(2) = mat20 * vector(0) + mat21 * vector(1) + mat22 * vector(2) + mat23;
                vector(0) = x;
                vector(1) = y;
                vector(3) = 1.0;
            }

			void transform(Eigen::Vector3d &vector);

			void transform(const Eigen::Vector3d &vectorIn, Eigen::Vector3d &vectorOut);

			void transform(const Eigen::Vector4d &vectorIn, Eigen::Vector4d &vectorOut);

			void transform(const Point3d<double> &pointIn, Point3d<double> &pointOut);
            void transform(const Point3d<float> &pointIn, Point3d<float> &pointOut);

            /**
            *  Apply a x-axis rotation to the current transform.
            */
			template<typename TYPE>
			void applyRotationX(const TYPE angle)
            {
                RigidBodyTransform<T> temp;
                temp.rotX(angle);
                multiply(temp);
            }

            /**
            *  Apply a y-axis rotation to the current transform.
            */
			template<typename TYPE>
			void applyRotationY(const TYPE angle)
            {
                RigidBodyTransform temp;
                temp.rotY(angle);
                multiply(temp);
            }

            /**
            *  Apply a z-axis rotation to the current transform.
            */
			template<typename TYPE>
			void applyRotationZ(const TYPE angle)
            {
                RigidBodyTransform temp;
                temp.rotZ(angle);
                multiply(temp);
            }

			template<typename TYPE>
			void rotX(const TYPE angle)
            {
                double cosAngle = cos(angle);
                double sinAngle = sin(angle);

                this->mat00 = 1.0;
                this->mat01 = 0.0;
                this->mat02 = 0.0;
                this->mat03 = 0.0;
                this->mat10 = 0.0;
                this->mat11 = cosAngle;
                this->mat12 = -sinAngle;
                this->mat13 = 0.0;
                this->mat20 = 0.0;
                this->mat21 = sinAngle;
                this->mat22 = cosAngle;
                this->mat23 = 0.0;
            }

            /**
            * Create RigidBodyTransform with zero translation and the rotation matrix being a
            * rotation about the y-axis by angle.
            *
            * @param angle
            */
			template<typename TYPE>
			void rotY(const TYPE angle)
            {
                double cosAngle = cos(angle);
                double sinAngle = sin(angle);

                mat00 = cosAngle;
                mat01 = 0.0;
                mat02 = sinAngle;
                mat03 = 0.0;
                mat10 = 0.0;
                mat11 = 1.0;
                mat12 = 0.0;
                mat13 = 0.0;
                mat20 = -sinAngle;
                mat21 = 0.0;
                mat22 = cosAngle;
                mat23 = 0.0;
            }

            /**
             * Create RigidBodyTransform with zero translation and the rotation matrix being a
             * rotation about the z-axis by angle.
             *
             * @param angle
             */
			template<typename TYPE>
			void rotZ(const TYPE angle)
            {
                double cosAngle = cos(angle);
                double sinAngle = sin(angle);

                mat00 = cosAngle;
                mat01 = -sinAngle;
                mat02 = 0.0;
                mat03 = 0.0;
                mat10 = sinAngle;
                mat11 = cosAngle;
                mat12 = 0.0;
                mat13 = 0.0;
                mat20 = 0.0;
                mat21 = 0.0;
                mat22 = 1.0;
                mat23 = 0.0;
            }

			void multiply(const RigidBodyTransform &transform);

			void multiply(const RigidBodyTransform &transform1, const RigidBodyTransform &transform);

			bool isRotationMatrixEpsilonIdentity(const double epsilon) const;

			void invert(const RigidBodyTransform &transform);

			void invert();

            /**
             * Invert this assuming an orthogonal rotation portion.
             */
			void invertOrthogonal()

            {
                T tmp01 = mat01;
                T tmp02 = mat02;
                T tmp12 = mat12;

                // For orthogonal matrix, R^{-1} = R^{T}
                mat01 = mat10;
                mat02 = mat20;
                mat12 = mat21;
                mat10 = tmp01;
                mat20 = tmp02;
                mat21 = tmp12;

                // New translation vector becomes -R^{T} * p
                T newTransX = -(mat23 * mat02 + mat00 * mat03 + mat01 * mat13);
                T newTransY = -(mat03 * mat10 + mat23 * mat12 + mat11 * mat13);
                mat23 = -(mat22 * mat23 + mat03 * mat20 + mat13 * mat21);
                mat03 = newTransX;
                mat13 = newTransY;
            }

			void invertRotationButKeepTranslation();

			bool epsilonEquals(const RigidBodyTransform &transform, const double &epsilon) const;

			bool equals(const RigidBodyTransform &transform) const;

            /**
            * Return the determinant of this transform.
            *
            * @return
            */
			T determinant() const
            {
                return (mat00 * (mat11 * mat22 - mat12 * mat21) - mat01 * (mat10 * mat22 - mat12 * mat20) + mat02 * (mat10 * mat21 - mat11 * mat20));
            }

            /**
            * Orthonormalization of the rotation matrix using Gram-Schmidt method.
            */
			void normalize()
            {
                T xdoty = mat00 * mat01 + mat10 * mat11 + mat20 * mat21;
                T xdotx = mat00 * mat00 + mat10 * mat10 + mat20 * mat20;
                T tmp = xdoty / xdotx;

                mat01 -= tmp * mat00;
                mat11 -= tmp * mat10;
                mat21 -= tmp * mat20;

                T zdoty = mat02 * mat01 + mat12 * mat11 + mat22 * mat21;
                T zdotx = mat02 * mat00 + mat12 * mat10 + mat22 * mat20;
                T ydoty = mat01 * mat01 + mat11 * mat11 + mat21 * mat21;

                tmp = zdotx / xdotx;
                T tmp1 = zdoty / ydoty;

                mat02 = mat02 - (tmp * mat00 + tmp1 * mat01);
                mat12 = mat12 - (tmp * mat10 + tmp1 * mat11);
                mat22 = mat22 - (tmp * mat20 + tmp1 * mat21);

                // Compute orthogonalized vector magnitudes and normalize
                T magX = sqrt(mat00 * mat00 + mat10 * mat10 + mat20 * mat20);
                T magY = sqrt(mat01 * mat01 + mat11 * mat11 + mat21 * mat21);
                T magZ = sqrt(mat02 * mat02 + mat12 * mat12 + mat22 * mat22);

                mat00 = mat00 / magX;
                mat10 = mat10 / magX;
                mat20 = mat20 / magX;
                mat01 = mat01 / magY;
                mat11 = mat11 / magY;
                mat21 = mat21 / magY;
                mat02 = mat02 / magZ;
                mat12 = mat12 / magZ;
                mat22 = mat22 / magZ;
            }

			static Eigen::Vector3d getTranslationDifference(const RigidBodyTransform &transform1, const RigidBodyTransform &transform2);

			friend std::ostream &operator<<(std::ostream &os, const RigidBodyTransform &transform)
			{
				os << "[ " << transform.mat00 << ',' << transform.mat01 << "," << transform.mat02 << "," << transform.mat03 << "]" << "\n" <<
				"[ " << transform.mat10 << ',' << transform.mat11 << "," << transform.mat12 << "," << transform.mat13 << "]" << "\n" <<
				"[ " << transform.mat20 << ',' << transform.mat21 << "," << transform.mat22 << "," << transform.mat23 << "]" << "\n" <<
				"[ " << 0 << ',' << 0 << "," << 0 << "," << 1 << "]";
				return os;
			}

			friend RigidBodyTransform operator*(const RigidBodyTransform &transform1, const RigidBodyTransform &transform2)
			{
				RigidBodyTransform tmp;
				tmp = transform1;
				tmp.multiply(transform2);
				return tmp;
			}

			RigidBodyTransform &operator*=(const RigidBodyTransform &transform)
			{
				this->multiply(transform);
				return *this;
			}

		private:
            template<class TYPE>
			void getRotation(Eigen::AngleAxis<TYPE> &axisAngle, const double epsilon) const
            {
                axisAngle.axis()[0] = mat21 - mat12;
                axisAngle.axis()[1] = mat02 - mat20;
                axisAngle.axis()[2] = mat10 - mat01;

                TYPE mag = axisAngle.axis()[0] * axisAngle.axis()[0] + axisAngle.axis()[1] * axisAngle.axis()[1] + axisAngle.axis()[2] * axisAngle.axis()[2];

                if (mag > epsilon)
                {
                    mag = sqrt(mag);
                    TYPE sin = 0.5 * mag;
                    TYPE cos = 0.5 * (mat00 + mat11 + mat22 - 1.0);

                    axisAngle.angle() = atan2(sin, cos);

                    TYPE invMag = 1.0 / mag;
                    axisAngle.axis()[0] = axisAngle.axis()[0] * invMag;
                    axisAngle.axis()[1] = axisAngle.axis()[1] * invMag;
                    axisAngle.axis()[2] = axisAngle.axis()[2] * invMag;
                }
                else
                {
                    if (isRotationMatrixEpsilonIdentity(10.0 * epsilon))
                    {
                        axisAngle.angle() = 0.0;
                        axisAngle.axis()[0] = 1.0;
                        axisAngle.axis()[1] = 0.0;
                        axisAngle.axis()[2] = 0.0;
                        return;
                    }
                    else
                    {
                        axisAngle.angle() = M_PI;

                        TYPE xx = (mat00 + 1.0) / 2.0;
                        TYPE yy = (mat11 + 1.0) / 2.0;
                        TYPE zz = (mat22 + 1.0) / 2.0;
                        TYPE xy = (mat01 + mat10) / 4.0;
                        TYPE xz = (mat02 + mat20) / 4.0;
                        TYPE yz = (mat12 + mat21) / 4.0;
                        TYPE cos45 = cos(M_PI / 4.0);

                        if ((xx > yy) && (xx > zz))
                        {
                            // mat00 is the largest diagonal term
                            if (xx < epsilon)
                            {
                                axisAngle.axis()[0] = 0.0;
                                axisAngle.axis()[1] = cos45;
                                axisAngle.axis()[2] = cos45;
                            }
                            else
                            {
                                axisAngle.axis()[0] = sqrt(xx);
                                axisAngle.axis()[1] = xy / axisAngle.axis()[0];
                                axisAngle.axis()[2] = xz / axisAngle.axis()[0];
                            }
                        }
                        else if (yy > zz)
                        {
                            // mat11 is the largest diagonal term
                            if (yy < epsilon)
                            {
                                axisAngle.axis()[0] = cos45;
                                axisAngle.axis()[1] = 0.0;
                                axisAngle.axis()[2] = cos45;
                            }
                            else
                            {
                                axisAngle.axis()[1] = sqrt(yy);
                                axisAngle.axis()[0] = xy / axisAngle.axis()[1];
                                axisAngle.axis()[2] = yz / axisAngle.axis()[1];
                            }
                        }
                        else
                        {
                            // mat22 is the largest diagonal term
                            if (zz < epsilon)
                            {
                                axisAngle.axis()[0] = cos45;
                                axisAngle.axis()[1] = cos45;
                                axisAngle.axis()[2] = 0.0;
                            }
                            else
                            {
                                axisAngle.axis()[2] = sqrt(zz);
                                axisAngle.axis()[0] = xz / axisAngle.axis()[2];
                                axisAngle.axis()[1] = yz / axisAngle.axis()[2];
                            }
                        }
                    }
                }
            }

			T mat00;
			T mat01;
			T mat02;
			T mat03;
			T mat10;
			T mat11;
			T mat13;
			T mat12;
			T mat20;
			T mat21;
			T mat22;
			T mat23;
		};

	}

}

#endif
