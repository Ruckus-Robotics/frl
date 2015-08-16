
#include "RigidBodyTransform.hpp"
#include <eigen3/Eigen/Eigen>
#include <math.h>

RigidBodyTransform::RigidBodyTransform()
{
    setIdentity();
}

RigidBodyTransform::RigidBodyTransform(const RigidBodyTransform *rigidBodyTransform)
{
    set(rigidBodyTransform);
}

RigidBodyTransform::RigidBodyTransform(const Eigen::Matrix4d matrix)
{
    set(matrix);
}

RigidBodyTransform::RigidBodyTransform(const Eigen::Matrix4f matrix)
{
    set(matrix);
}

RigidBodyTransform::RigidBodyTransform(const double array[16])
{
    set(array);
}

RigidBodyTransform::RigidBodyTransform(const float array[16])
{
    set(array);
}

RigidBodyTransform::RigidBodyTransform(const Eigen::Matrix3d matrix, const Eigen::Vector3d vector)
{
    set(matrix,vector);
}

RigidBodyTransform::RigidBodyTransform(const Eigen::Matrix3f matrix, const Eigen::Vector3f vector)
{
    set(matrix,vector);
}

void RigidBodyTransform::set(const RigidBodyTransform rigidBodyTransform)
{
    this->mat00 = rigidBodyTransform.mat00;
    this->mat01 = rigidBodyTransform.mat01;
    this->mat02 = rigidBodyTransform.mat02;
    this->mat10 = rigidBodyTransform.mat10;
    this->mat11 = rigidBodyTransform.mat11;
    this->mat12 = rigidBodyTransform.mat12;
    this->mat20 = rigidBodyTransform.mat20;
    this->mat21 = rigidBodyTransform.mat21;
    this->mat22 = rigidBodyTransform.mat22;
    this->mat03 = rigidBodyTransform.mat03;
    this->mat13 = rigidBodyTransform.mat13;
    this->mat23 = rigidBodyTransform.mat23;   
}

void RigidBodyTransform::set(const Eigen::Matrix3d matrix, const Eigen::Vector3d vector)
{
    this->mat00 = matrix(0,0);
    this->mat01 = matrix(0,1);
    this->mat02 = matrix(0,2);
    this->mat10 = matrix(1,0);
    this->mat11 = matrix(1,1);
    this->mat12 = matrix(1,2);
    this->mat20 = matrix(2,0);
    this->mat21 = matrix(2,1);
    this->mat22 = matrix(2,2);
    this->mat03 = vector(0,3);
    this->mat13 = vector(1,3);
    this->mat23 = vector(2,3);
}

void RigidBodyTransform::set(const Eigen::Matrix3f matrix, const Eigen::Vector3f vector)
{
    this->mat00 = (double) matrix(0,0);
    this->mat01 = (double) matrix(0,1);
    this->mat02 = (double) matrix(0,2);
    this->mat10 = (double) matrix(1,0);
    this->mat11 = (double) matrix(1,1);
    this->mat12 = (double) matrix(1,2);
    this->mat20 = (double) matrix(2,0);
    this->mat21 = (double) matrix(2,1);
    this->mat22 = (double) matrix(2,2);
    this->mat03 = (double) vector(0,3);
    this->mat13 = (double) vector(1,3);
    this->mat23 = (double) vector(2,3);
}

void RigidBodyTransform::set(const double array[16])
{
    this->mat00 = array[0];
    this->mat01 = array[1];
    this->mat02 = array[2];
    this->mat03 = array[3];
    this->mat10 = array[4];
    this->mat11 = array[5];
    this->mat12 = array[6];
    this->mat13 = array[7];
    this->mat20 = array[8];
    this->mat21 = array[9];
    this->mat22 = array[10];
    this->mat23 = array[11];
}

void RigidBodyTransform::set(const float array[16])
{
    this->mat00 = (double) array[0];
    this->mat01 = (double) array[1];
    this->mat02 = (double) array[2];
    this->mat03 = (double) array[3];
    this->mat10 = (double) array[4];
    this->mat11 = (double) array[5];
    this->mat12 = (double) array[6];
    this->mat13 = (double) array[7];
    this->mat20 = (double) array[8];
    this->mat21 = (double) array[9];
    this->mat22 = (double) array[10];
    this->mat23 = (double) array[11];
}

void RigidBodyTransform::set(const Eigen::Matrix4d matrix)
{
    this->mat00 = matrix(0,0);
    this->mat01 = matrix(0,1);
    this->mat02 = matrix(0,2);
    this->mat03 = matrix(0,3);
    this->mat10 = matrix(1,0);
    this->mat11 = matrix(1,1);
    this->mat12 = matrix(1,2);
    this->mat13 = matrix(1,3);
    this->mat20 = matrix(2,0);
    this->mat21 = matrix(2,1);
    this->mat22 = matrix(2,2);
    this->mat23 = matrix(2,3);   
}

void RigidBodyTransform::set(const Eigen::Matrix4f matrix)
{
    this->mat00 = (double) matrix(0,0);
    this->mat01 = (double) matrix(0,1);
    this->mat02 = (double) matrix(0,2);
    this->mat03 = (double) matrix(0,3);
    this->mat10 = (double) matrix(1,0);
    this->mat11 = (double) matrix(1,1);
    this->mat12 = (double) matrix(1,2);
    this->mat13 = (double) matrix(1,3);
    this->mat20 = (double) matrix(2,0);
    this->mat21 = (double) matrix(2,1);
    this->mat22 = (double) matrix(2,2);
    this->mat23 = (double) matrix(2,3);   
}

void RigidBodyTransform::setRotation(const Eigen::Matrix3d matrix)
{
    this->mat00 = matrix(0,0);
    this->mat01 = matrix(0,1);
    this->mat02 = matrix(0,2);
    this->mat10 = matrix(1,0);
    this->mat11 = matrix(1,1);
    this->mat12 = matrix(1,2);
    this->mat20 = matrix(2,0);
    this->mat21 = matrix(2,1);
    this->mat22 = matrix(2,2);
}

void RigidBodyTransform::setRotation(const Eigen::Matrix3f matrix)
{
    this->mat00 = (double) matrix(0,0);
    this->mat01 = (double) matrix(0,1);
    this->mat02 = (double) matrix(0,2);
    this->mat10 = (double) matrix(1,0);
    this->mat11 = (double) matrix(1,1);
    this->mat12 = (double) matrix(1,2);
    this->mat20 = (double) matrix(2,0);
    this->mat21 = (double) matrix(2,1);
    this->mat22 = (double) matrix(2,2);
}

void RigidBodyTransform::setTranslation(const double x, const double y, const double z)
{
    this->mat03 = x;
    this->mat13 = y;
    this->mat23 = z;
}

void RigidBodyTransform::setTranslation(const float x, const float y, const float z)
{
    this->mat03 = (double) x;
    this->mat13 = (double) y;
    this->mat23 = (double) z;
}

void RigidBodyTransform::setTranslation(const Eigen::Vector3d vector)
{
    this->setTranslation(vector(0),vector(1),vector(2));
}

void RigidBodyTransform::setTranslation(const Eigen::Vector3f vector)
{
    this->mat03 = (double) vector(0);
    this->mat13 = (double) vector(1);
    this->mat23 = (double) vector(2);
}

void RigidBodyTransform::setRotationAndZeroTranslation(const Eigen::Matrix3d matrix)
{
    setRotation(matrix);
    setTranslation(0.0,0.0,0.0);
}

void RigidBodyTransform::zeroTranslation()
{
    setTranslation(0.0,0.0,0.0);
}

void RigidBodyTransform::setTranslationAndIdentityRotation(const Eigen::Vector3d vector)
{
    setTranslation(vector(0),vector(1),vector(2));
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

void RigidBodyTransform::setTranslationAndIdentityRotation(const Eigen::Vector3f vector)
{
    setTranslation((double) vector(0),(double) vector(1),(double) vector(2));
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

void RigidBodyTransform::setEuler(const Eigen::Vector3d vector)
{
    setEuler(vector(0),vector(1),vector(2));
}

void RigidBodyTransform::setEuler(double rotX, double rotY, double rotZ)
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
*/

void RigidBodyTransform::getEulerXYZ(Eigen::Vector3d &vector)
{
    vector(0) = atan2(mat21, mat22);
    vector(1) = atan2(-mat20, sqrt(mat21 * mat21 + mat22 * mat22));
    vector(2) = atan2(mat10, mat00);
}

void RigidBodyTransform::setIdentity()
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

void RigidBodyTransform::getRotation(Eigen::Matrix3d &matrix)
{
    matrix(0,0) = this->mat00;
    matrix(0,1) = this->mat01;
    matrix(0,2) = this->mat02;
    matrix(1,0) = this->mat10;
    matrix(1,1) = this->mat11;
    matrix(1,2) = this->mat12;
    matrix(2,0) = this->mat20;
    matrix(2,1) = this->mat21;
    matrix(2,2) = this->mat22;
}

void RigidBodyTransform::getRotation(Eigen::Matrix3f &matrix)
{
    matrix(0,0) = (float) this->mat00;
    matrix(0,1) = (float) this->mat01;
    matrix(0,2) = (float) this->mat02;
    matrix(1,0) = (float) this->mat10;
    matrix(1,1) = (float) this->mat11;
    matrix(1,2) = (float) this->mat12;
    matrix(2,0) = (float) this->mat20;
    matrix(2,1) = (float) this->mat21;
    matrix(2,2) = (float) this->mat22;
}

void RigidBodyTransform::getTranslation(Eigen::Vector3d &vector)
{
    vector(0) = this->mat03;
    vector(1) = this->mat13;
    vector(2) = this->mat23;
}

void RigidBodyTransform::getTranslation(Eigen::Vector3f &vector)
{
    vector(0) = (float) this->mat03;
    vector(1) = (float) this->mat13;
    vector(2) = (float) this->mat23;
}

void RigidBodyTransform::get(double (&ret)[16])
{
  ret[0] = this->mat00;
  ret[1] = this->mat01;
  ret[2] = this->mat02;
  ret[3] = this->mat03;
  ret[4] = this->mat10;
  ret[5] = this->mat11;
  ret[6] = this->mat12;
  ret[7] = this->mat13;
  ret[8] = this->mat20;
  ret[9] = this->mat21;
  ret[10] = this->mat22;
  ret[11] = this->mat23;
  ret[12] = 0.0;
  ret[13] = 0.0;
  ret[14] = 0.0;
  ret[15] = 1.0;
}

void RigidBodyTransform::get(float (&ret)[16])
{
  ret[0] = (float) this->mat00;
  ret[1] = (float) this->mat01;
  ret[2] = (float) this->mat02;
  ret[3] = (float) this->mat03;
  ret[4] = (float) this->mat10;
  ret[5] = (float) this->mat11;
  ret[6] = (float) this->mat12;
  ret[7] = (float) this->mat13;
  ret[8] = (float) this->mat20;
  ret[9] = (float) this->mat21;
  ret[10] = (float) this->mat22;
  ret[11] = (float) this->mat23;
  ret[12] = 0.0f;
  ret[13] = 0.0f;
  ret[14] = 0.0f;
  ret[15] = 1.0f;
}

void RigidBodyTransform::get(Eigen::Matrix4d &matrix)
{
    matrix(0,0) = this->mat00;
    matrix(0,1) = this->mat01;
    matrix(0,2) = this->mat02;
    matrix(0,3) = this->mat03;
    matrix(1,0) = this->mat10;
    matrix(1,1) = this->mat11;
    matrix(1,2) = this->mat12;
    matrix(1,3) = this->mat13;
    matrix(2,0) = this->mat20;
    matrix(2,1) = this->mat21;
    matrix(2,2) = this->mat22;
    matrix(2,3) = this->mat23;
    matrix(3,0) = 0.0;
    matrix(3,1) = 0.0;
    matrix(3,2) = 0.0;
    matrix(3,3) = 1.0;
}

void RigidBodyTransform::get(Eigen::Matrix4f &matrix)
{
    matrix(0,0) = (float) this->mat00;
    matrix(0,1) = (float) this->mat01;
    matrix(0,2) = (float) this->mat02;
    matrix(0,3) = (float) this->mat03;
    matrix(1,0) = (float) this->mat10;
    matrix(1,1) = (float) this->mat11;
    matrix(1,2) = (float) this->mat12;
    matrix(1,3) = (float) this->mat13;
    matrix(2,0) = (float) this->mat20;
    matrix(2,1) = (float) this->mat21;
    matrix(2,2) = (float) this->mat22;
    matrix(2,3) = (float) this->mat23;
    matrix(3,0) = 0.0f;
    matrix(3,1) = 0.0f;
    matrix(3,2) = 0.0f;
    matrix(3,3) = 1.0f;
}

void RigidBodyTransform::get(Eigen::Matrix3d &matrix, Eigen::Vector3d &vector)
{
    getRotation(matrix);
    getTranslation(vector);
}

void RigidBodyTransform::get(Eigen::Matrix3f &matrix, Eigen::Vector3f &vector)
{
    getRotation(matrix);
    getTranslation(vector);
}

void RigidBodyTransform::get(Eigen::Matrix3d &matrix)
{
    getRotation(matrix);
}

void RigidBodyTransform::get(Eigen::Matrix3f &matrix)
{
    getRotation(matrix);
}

void RigidBodyTransform::get(Eigen::Vector3d &vector)
{
    getTranslation(vector);
}

void RigidBodyTransform::get(Eigen::Vector3f &vector)
{
    getTranslation(vector);
}   

void RigidBodyTransform::multiply(const RigidBodyTransform transform)
{
    multiply(*this,transform);
}

void RigidBodyTransform::multiply(const RigidBodyTransform transform1, const RigidBodyTransform transform2)
{
    double tmp00 = transform1.mat00 * transform2.mat00 + transform1.mat01 * transform2.mat10 + transform1.mat02 * transform2.mat20;
    double tmp01 = transform1.mat00 * transform2.mat01 + transform1.mat01 * transform2.mat11 + transform1.mat02 * transform2.mat21;
    double tmp02 = transform1.mat00 * transform2.mat02 + transform1.mat01 * transform2.mat12 + transform1.mat02 * transform2.mat22;
    double tmp03 = transform1.mat00 * transform2.mat03 + transform1.mat01 * transform2.mat13 + transform1.mat02 * transform2.mat23 + transform1.mat03;

    double tmp10 = transform1.mat10 * transform2.mat00 + transform1.mat11 * transform2.mat10 + transform1.mat12 * transform2.mat20;
    double tmp11 = transform1.mat10 * transform2.mat01 + transform1.mat11 * transform2.mat11 + transform1.mat12 * transform2.mat21;
    double tmp12 = transform1.mat10 * transform2.mat02 + transform1.mat11 * transform2.mat12 + transform1.mat12 * transform2.mat22;
    double tmp13 = transform1.mat10 * transform2.mat03 + transform1.mat11 * transform2.mat13 + transform1.mat12 * transform2.mat23 + transform1.mat13;

    double tmp20 = transform1.mat20 * transform2.mat00 + transform1.mat21 * transform2.mat10 + transform1.mat22 * transform2.mat20;
    double tmp21 = transform1.mat20 * transform2.mat01 + transform1.mat21 * transform2.mat11 + transform1.mat22 * transform2.mat21;
    double tmp22 = transform1.mat20 * transform2.mat02 + transform1.mat21 * transform2.mat12 + transform1.mat22 * transform2.mat22;
    double tmp23 = transform1.mat20 * transform2.mat03 + transform1.mat21 * transform2.mat13 + transform1.mat22 * transform2.mat23 + transform1.mat23;

    this->mat00 = tmp00;
    this->mat01 = tmp01;
    this->mat02 = tmp02;
    this->mat03 = tmp03;
    this->mat10 = tmp10;
    this->mat11 = tmp11;
    this->mat12 = tmp12;
    this->mat13 = tmp13;
    this->mat20 = tmp20;
    this->mat21 = tmp21;
    this->mat22 = tmp22;
    this->mat23 = tmp23;
}

void RigidBodyTransform::invert(const RigidBodyTransform &transform)
{
  if(&transform !=  this)
  {
     set(transform);
  }
  invert();
}

void RigidBodyTransform::invert()
{
  invertOrthogonal();
}

void RigidBodyTransform::invertOrthogonal()
{
    double tmp01 = this->mat01;
    double tmp02 = this->mat02;
    double tmp12 = this->mat12;

    // For orthogonal matrix, R^{-1} = R^{T}
    this->mat01 = mat10;
    this->mat02 = mat20;
    this->mat12 = mat21;
    this->mat10 = tmp01;
    this->mat20 = tmp02;
    this->mat21 = tmp12;

    // New translation vector becomes -R^{T} * p
    double newTransX = -(this->mat23 * this->mat02 + this->mat00 * this->mat03 + this->mat01 * this->mat13);
    double newTransY = -(this->mat03 * this->mat10 + this->mat23 * this->mat12 + this->mat11 * this->mat13);
    this->mat23 = -(this->mat22 * this->mat23 + this->mat03 * this->mat20 + this->mat13 * this->mat21);
    this->mat03 = newTransX;
    this->mat13 = newTransY;
}

void RigidBodyTransform::rotX(double angle)
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

void RigidBodyTransform::rotY(double angle)
{
    double cosAngle = cos(angle);
    double sinAngle = sin(angle);

    this->mat00 = cosAngle;
    this->mat01 = 0.0;
    this->mat02 = sinAngle;
    this->mat03 = 0.0;
    this->mat10 = 0.0;
    this->mat11 = 1.0;
    this->mat12 = 0.0;
    this->mat13 = 0.0;
    this->mat20 = -sinAngle;
    this->mat21 = 0.0;
    this->mat22 = cosAngle;
    this->mat23 = 0.0;
}

void RigidBodyTransform::rotZ(double angle)
{
    double cosAngle = cos(angle);
    double sinAngle = sin(angle);

    this->  mat00 = cosAngle;
    this->  mat01 = -sinAngle;
    this->  mat02 = 0.0;
    this->  mat03 = 0.0;
    this->  mat10 = sinAngle;
    this->  mat11 = cosAngle;
    this->  mat12 = 0.0;
    this->  mat13 = 0.0;
    this->  mat20 = 0.0;
    this->  mat21 = 0.0;
    this->  mat22 = 1.0;
    this->  mat23 = 0.0;
}

bool RigidBodyTransform::epsilonEquals(double a, double b, double epsilon)
{
    return ((fabs(a-b) < epsilon) ? true : false);
}

bool RigidBodyTransform::epsilonEquals(const RigidBodyTransform &transform, double epsilon)
{
    if (!epsilonEquals(mat00, transform.mat00, epsilon)) return false;
    if (!epsilonEquals(mat01, transform.mat01, epsilon)) return false;
    if (!epsilonEquals(mat02, transform.mat02, epsilon)) return false;
    if (!epsilonEquals(mat03, transform.mat03, epsilon)) return false;
    if (!epsilonEquals(mat10, transform.mat10, epsilon)) return false;
    if (!epsilonEquals(mat11, transform.mat11, epsilon)) return false;
    if (!epsilonEquals(mat12, transform.mat12, epsilon)) return false;
    if (!epsilonEquals(mat13, transform.mat13, epsilon)) return false;
    if (!epsilonEquals(mat20, transform.mat20, epsilon)) return false;
    if (!epsilonEquals(mat21, transform.mat21, epsilon)) return false;
    if (!epsilonEquals(mat22, transform.mat22, epsilon)) return false;
    if (!epsilonEquals(mat23, transform.mat23, epsilon)) return false;
    return true;
}

bool RigidBodyTransform::equals(const RigidBodyTransform &transform)
{
    return epsilonEquals(transform, 1e-10);
}

