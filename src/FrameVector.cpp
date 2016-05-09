#include "frl/frames/FrameVector.hpp"

namespace frl
{

    namespace frames
    {

        FrameVector::FrameVector(const std::string &name, ReferenceFrame *referenceFrame, const double &x, const double &y, const double &z) : vector(x, y, z)
        {
            if (!referenceFrame)
            {
                throw std::runtime_error("Reference frame cannot be nullptr!");
            }

            this->name = name;
            this->referenceFrame = referenceFrame;
        }

//        FrameVector::FrameVector(const std::string &name, ReferenceFrame *referenceFrame, const Eigen::Vector3d &vector) : vector(vector)
//        {
//            if (!referenceFrame)
//            {
//                throw std::runtime_error("Reference frame cannot be nullptr!");
//            }
//
//            this->name = name;
//            this->referenceFrame = referenceFrame;
//        }

//        void FrameVector::setIncludingFrame(const double &x, const double &y, const double &z, ReferenceFrame *referenceFrame)
//        {
//            if (!referenceFrame)
//            {
//                throw std::runtime_error("Reference frame cannot be nullptr!");
//            }
//
//            this->vector(0) = x;
//            this->vector(1) = y;
//            this->vector(2) = z;
//
//            this->referenceFrame = referenceFrame;
//        }
//
//        void FrameVector::setIncludingFrame(const Eigen::Vector3d &vector, ReferenceFrame *referenceFrame)
//        {
//            if (!referenceFrame)
//            {
//                throw std::runtime_error("Reference frame cannot be nullptr!");
//            }
//
//            setIncludingFrame(vector(0), vector(1), vector(2), referenceFrame);
//        }
//
//        void FrameVector::setAndKeepFrame(const double &x, const double &y, const double &z)
//        {
//            vector(0) = x;
//            vector(1) = y;
//            vector(2) = z;
//        }

//        void FrameVector::setAndKeepFrame(const Eigen::Vector3d vector)
//        {
//            setAndKeepFrame(vector(0), vector(1), vector(2));
//        }

//        double FrameVector::dot(const FrameVector &frameVector) const
//        {
//            checkReferenceFramesMatch(&frameVector);
//            return this->vector.dot(frameVector.getVector());
//        }

//        void FrameVector::cross(const FrameVector &frameVector, FrameVector &frameVectorToPack) const
//        {
//            checkReferenceFramesMatch(&frameVector);
//            checkReferenceFramesMatch(&frameVectorToPack);
//            Eigen::Vector3d tmpVector3d = this->vector.cross(frameVector.getVector());
//            frameVectorToPack.setAndKeepFrame(tmpVector3d(0), tmpVector3d(1), tmpVector3d(2));
//        }

//        Eigen::Vector3d FrameVector::cross(const FrameVector &frameVector) const
//        {
//            checkReferenceFramesMatch(&frameVector);
//            return this->vector.cross(frameVector.getVector());
//        }

//        double FrameVector::getAngleBetweenVectors(const FrameVector &frameVector) const
//        {
//            checkReferenceFramesMatch(&frameVector);
//
//            double vDot = this->vector.dot(frameVector.getVector()) / (this->vector.norm() * frameVector.getVector().norm());
//
//            if (vDot < -1.0)
//            {
//                vDot = -1.0;
//            }
//            else if (vDot > 1.0)
//            {
//                vDot = 1.0;
//            }
//
//            return acos(vDot);
//        }

//        void FrameVector::changeFrame(ReferenceFrame *desiredFrame)
//        {
//            if (desiredFrame != this->referenceFrame)
//            {
//                this->referenceFrame->verifyFramesHaveSameRoot(desiredFrame);
//
//                geometry::RigidBodyTransform thisFramesTransformToRoot, desiredFramesInverseTransformToRoot;
//                thisFramesTransformToRoot = this->referenceFrame->getTransformToRoot();
//                desiredFramesInverseTransformToRoot = desiredFrame->getInverseTransformToRoot();
//
//                if (this->referenceFrame && desiredFrame)
//                {
//                    thisFramesTransformToRoot.transform(vector);
//                    desiredFramesInverseTransformToRoot.transform(vector);
//                    this->referenceFrame = desiredFrame;
//                }
//                else
//                {
//                    throw std::runtime_error("Cannot change the frame of a FrameVector if either this's reference frame or the desired frame is nullptr!");
//                }
//            }
//        }
//    }

}