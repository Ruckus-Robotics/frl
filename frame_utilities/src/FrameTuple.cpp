#include "FrameTuple.hpp"

namespace frame_utilities
{

FrameTuple::FrameTuple()
{
	this->name = "";
	this->referenceFrame.reset(nullptr);
	setToZero();
}

FrameTuple::FrameTuple(const std::string &name, ReferenceFrame* referenceFrame, const double &x, const double &y, const double &z)
{
	this->name = name;
	this->referenceFrame.reset(referenceFrame);
	set(x, y, z);
}

FrameTuple::FrameTuple(const std::string &name, ReferenceFrame* referenceFrame)
{
	this->name = name;
	this->referenceFrame.reset(referenceFrame);
	setToZero();
}

FrameTuple::FrameTuple(const std::string &name, ReferenceFrame* referenceFrame, double array[3])
{
	this->name = name;
	this->referenceFrame.reset(referenceFrame);
	set(array);
}

FrameTuple::FrameTuple(const std::string &name, ReferenceFrame* referenceFrame, const std::vector<double> &vector)
{
	this->name = name;
	this->referenceFrame.reset(referenceFrame);
	set(vector);
}

FrameTuple::FrameTuple(const FrameTuple &frameTuple)
{
	this->name = frameTuple.name;
	this->x = frameTuple.x;
	this->y = frameTuple.y;
	this->z = frameTuple.z;
	this->referenceFrame.reset(frameTuple.referenceFrame.get());
}

void FrameTuple::set(const double &x, const double &y, const double &z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}

void FrameTuple::set(double array[3])
{
	set(array[0], array[1], array[2]);
}

void FrameTuple::set(std::vector<double> vector)
{
	set(vector[0], vector[1], vector[2]);
}

void FrameTuple::setIncludingFrame(const FrameTuple &frameTuple)
{
	this->x = frameTuple.x;
	this->y = frameTuple.y;
	this->z = frameTuple.z;
	this->referenceFrame.reset(frameTuple.referenceFrame.get());
}

void FrameTuple::setIncludingFrame(ReferenceFrame* referenceFrame, const double &x, const double &y, const double &z)
{
	this->referenceFrame.reset(referenceFrame);
	this->x = x;
	this->y = y;
	this->z = z;
}

void FrameTuple::setToZero()
{
	this->x = 0.0;
	this->y = 0.0;
	this->z = 0.0;
}

}