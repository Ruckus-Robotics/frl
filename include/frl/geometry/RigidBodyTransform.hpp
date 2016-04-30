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

			void setRotationAndZeroTranslation(const Eigen::Matrix3d &matrix);
			void setRotationAndZeroTranslation(const Eigen::Matrix3f &matrix);
			void setRotationAndZeroTranslation(const Eigen::Quaterniond &quat);
			void setRotationAndZeroTranslation(const Eigen::Quaternionf &quat);
			void setRotationAndZeroTranslation(const Eigen::AngleAxisd &axisAngle);
			void setRotationAndZeroTranslation(const Eigen::AngleAxisf &axisAngle);

			void setTranslationAndIdentityRotation(const Eigen::Vector3d &vector);

			void setRotationToIdentity();

			void setEuler(const Eigen::Vector3d &vector);

			void setEuler(const double &rotX, const double &rotY, const double &rotZ);

			void getEulerXYZ(Eigen::Vector3d &vector) const;

			void getRotation(Eigen::Matrix3d &matrix) const;

			void getRotation(Eigen::Quaternion<double> &quat) const;

			void getRotation(Eigen::AngleAxis<double> &axisAngle) const;

			void getTranslation(Eigen::Vector3d &vector) const;

			void getTranslation(Point3d<double> &point) const;
            void getTranslation(Point3d<float> &point) const;

			void get(Eigen::Matrix4d &matrix) const;
			void get(Eigen::Matrix3d &matrix, Eigen::Vector3d &vector) const;
			void get(Eigen::Matrix3d &matrix) const;
			void get(Eigen::Vector3d &vector) const;
			void get(Eigen::Quaternion<double> &quat, Eigen::Vector3d &vector) const;
			void get(Eigen::Quaternion<double> &quat, Point3d &point) const;
			void get(Eigen::Quaternion<double> &quat) const;

			void applyTranslation(const Eigen::Vector3d &translation);

			void transform(Point3d<double> &point);
            void transform(Point3d<float> &point);

			void transform(Eigen::Vector4d &vector);

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
			void getRotation(Eigen::AngleAxis<double> &axisAngle, const double epsilon) const;

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
