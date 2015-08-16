#ifndef RIGID_BODY_TRANSFORM_H
#define RIGID_BODY_TRANSFORM_H

#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/Transform.h>

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
    void setTranslationAndIdentityRotation(const Eigen::Vector3f vector);
    void setEuler(const Eigen::Vector3d vector);
    void setEuler(const double rotX, const double rotY, const double rotZ);
    void getEulerXYZ(Eigen::Vector3d &vector);
    void getRotation(Eigen::Matrix3d &matrix);
    void getRotation(Eigen::Matrix3f &matrix);
    void getTranslation(Eigen::Vector3d &vector);
    void getTranslation(Eigen::Vector3f &vector);
    void get(double (&array)[16]);
    void get(float (&array)[16]);
    void get(Eigen::Matrix4d &matrix);
    void get(Eigen::Matrix4f &matrix);
    void get(Eigen::Matrix3d &matrix, Eigen::Vector3d &vector);
    void get(Eigen::Matrix3f &matrix, Eigen::Vector3f &vector);
    void get(Eigen::Matrix3d &matrix);
    void get(Eigen::Matrix3f &matrix);
    void get(Eigen::Vector3d &vector);
    void get(Eigen::Vector3f &vector);
    void multiply(const RigidBodyTransform transform);
    void multiply(const RigidBodyTransform transform1, const RigidBodyTransform transform2);
    void invert(const RigidBodyTransform &transform);
    void invert();
    void invertOrthogonal();
    void rotX(double angle);
    void rotY(double angle);
    void rotZ(double angle);
    bool epsilonEquals(double a, double b, double epsilon);
    bool epsilonEquals(const RigidBodyTransform &transform, double epsilon);
    bool equals(const RigidBodyTransform &transform);

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
//	static void almostZero(double a);
};

#endif
