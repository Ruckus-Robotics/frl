#include "FrameTuple.hpp"
#include "Tuple3d.hpp"

namespace frame_utilities
{

FrameTuple::FrameTuple()
{
	this->name = "";
	this->referenceFrame = nullptr;
	geometry_utilities::Tuple3d tmpTuple;
	this->tuple = tmpTuple;
}

FrameTuple::FrameTuple(const std::string &name, ReferenceFrame* referenceFrame, const geometry_utilities::Tuple3d &tuple)
{
	this->name = name;
	this->referenceFrame = referenceFrame;
	this->tuple = tuple;
}

}