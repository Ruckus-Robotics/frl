
#include "RigidBodyTransform.hpp"
#include <eigen3/Eigen/Eigen>

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
    setTranslation(0.0,0.0,0.0);
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
