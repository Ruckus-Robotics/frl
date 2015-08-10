#ifndef RIGID_BODY_TRANSFORM_H
#define RIGID_BODY_TRANSFORM_H

#include <eigen3/Eigen/Eigen>

class RigidBodyTransform
{
public:
	RigidBodyTransform();
	RigidBodyTransform(const RigidBodyTransform *rigidBodyTransform);
    RigidBodyTransform(const Eigen::Matrix4d matrix);
    RigidBodyTransform(const Eigen::Matrix4f matrix);
    RigidBodyTransform(const double array[16]);
    RigidBodyTransform(const float array[16]);
    RigidBodyTransform(const Eigen::Matrix3d matrix, const Eigen::Vector3d vector);
    RigidBodyTransform(const Eigen::Matrix3f matrix, const Eigen::Vector3f vector);
    void set(const RigidBodyTransform rigidBodyTransform);
    void set(const Eigen::Matrix4d matrix);
    void set(const Eigen::Matrix4f matrix);
    void set(const double array[16]);
    void set(const float array[16]);
    void set(const Eigen::Matrix3d matrix, const Eigen::Vector3d vector);
    void set(const Eigen::Matrix3f matrix, const Eigen::Vector3f vector);
    void setRotation(const Eigen::Matrix3d matrix);
    void setRotation(const Eigen::Matrix3f matrix);
    void setTranslation(const double x, const double y, const double z);
    void setTranslation(const float x, const float y, const float z);
    void setTranslation(const Eigen::Vector3d vector);
    void setTranslation(const Eigen::Vector3f vector);
    void setRotationAndZeroTranslation(const Eigen::Matrix3d matrix);
    void zeroTranslation();
    void setTranslationAndIdentityRotation(const Eigen::Vector3d vector);

    double mat00;
	double mat01;
	double mat02;
	double mat03;
	double mat10;
	double mat11;
	double mat13;
	double mat12;
	double mat20;
	double mat21;
	double mat22;
	double mat23;
    

private:

	void setIdentity();
	// void setRotationWithAxisAngle();
	// void invertOrthogonal();
//	static void almostZero(double a);
};

#endif
