#include "ReferenceFrame.hpp"
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <random>
#include <memory>
#include <math.h>

class ReferenceFrameTestHelper
{
public:
    static tf::Transform createRandomTransformationMatrix()
    {
        tf::Quaternion quaternion = generateRandomQuaternion();
        tf::Vector3 translation = generateRandomTranslation();
        tf::Transform transform(quaternion, translation);

        return transform;
    }

    static ReferenceFrame* getRandomFrameFromVectorOfFrames(std::vector<ReferenceFrame*> vectorOfFrames)
    {
        return vectorOfFrames[rand() % 10];
    }

    static double getQuaternionMagnitude(tf::Quaternion q)
    {
        return sqrt(pow(q.getX(), 2) + pow(q.getY(), 2) + pow(q.getZ(), 2) + pow(q.getW(), 2));
    }

    static void printQuaternion(tf::Quaternion q)
    {
        std::cout << "x:" << q.getX() << "y:" << q.getY() << "z:" << q.getZ() << "w:" << q.getW() << std::endl;
    }

    static void printQuaternion(tf::Transform transform)
    {
        tf::Quaternion q = transform.getRotation();
        std::cout << "\n x:" << q.getX() << "\n y:" << q.getY() << "\n z:" << q.getZ() << "\n w:" << q.getW() << std::endl;
    }

private:
    static tf::Vector3 generateRandomTranslation()
    {
        tf::Vector3 vector;
        vector.setX(rand() % 10 - 5);
        vector.setY(rand() % 10 - 5);
        vector.setZ(rand() % 10 - 5);

        return vector;
    }

    static tf::Quaternion generateRandomQuaternion()
    {
        tf::Quaternion quaternion;
        quaternion.setRPY(generateRandomAngle(), generateRandomAngle(), generateRandomAngle());
        quaternion.normalize();

        return quaternion;
    }

    static double generateRandomAngle()
    {
        std::random_device randomDevice;
        std::mt19937 mt(randomDevice());
        std::uniform_real_distribution<double> dist(0, 1);

        return (dist(mt) * 6.2 - 3.1);
        // return 0.0;
    }
};

class RandomlyChangingFrame : public ReferenceFrame
{
public:
    RandomlyChangingFrame() : ReferenceFrame() {}
    RandomlyChangingFrame(const std::string& frameName, ReferenceFrame* parentFrame) : ReferenceFrame(frameName, parentFrame, false, false)
    {

    }

    RandomlyChangingFrame(const std::string& frameName, ReferenceFrame* parentFrame, tf::Transform transformToParent) : ReferenceFrame(frameName, parentFrame, transformToParent, false, false)
    {

    }

    static std::shared_ptr<RandomlyChangingFrame> create(const std::string& frameName, ReferenceFrame* parentFrame)
    {
        tf::Transform randomTransform = ReferenceFrameTestHelper::createRandomTransformationMatrix();
        std::shared_ptr<RandomlyChangingFrame> frame(new RandomlyChangingFrame(frameName, parentFrame, randomTransform));
        return frame;
    }

protected:
    void updateTransformToParent(tf::Transform& transformToParent)
    {
        tf::Transform randomTransform = ReferenceFrameTestHelper::createRandomTransformationMatrix();
        transformToParent = randomTransform;
    }
};

class RandomUnchangingFrame : public ReferenceFrame
{
public:
    RandomUnchangingFrame() : ReferenceFrame() {}

    RandomUnchangingFrame(const std::string& frameName, ReferenceFrame* parentFrame, tf::Transform transformToParent) : ReferenceFrame(frameName, parentFrame, transformToParent, false, false)
    {

    }

    static std::shared_ptr<RandomUnchangingFrame> create(const std::string& frameName, ReferenceFrame* parentFrame)
    {
        tf::Transform randomTransform = ReferenceFrameTestHelper::createRandomTransformationMatrix();

        std::shared_ptr<RandomUnchangingFrame> frame(new RandomUnchangingFrame(frameName, parentFrame, randomTransform));
        return frame;
    }

protected:
    void updateTransformToParent(tf::Transform& transformToParent)
    {

    }
};
