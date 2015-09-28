#include "frame_utilities/FrameTuple.hpp"
#include <math.h>
#include <cmath>
#include <stdexcept>

namespace frame_utilities
{

FrameTuple::FrameTuple() : ReferenceFrameHolder()
{
	this->name = "";
	this->referenceFrame = ReferenceFrame::getWorldFrame();
	setToZero();
}

FrameTuple::FrameTuple(const std::string &name) : ReferenceFrameHolder()
{
	this->name = name;
	this->referenceFrame = ReferenceFrame::getWorldFrame();
	setToZero();
}

FrameTuple::FrameTuple(const std::string &name, ReferenceFrame* referenceFrame, const double &x, const double &y, const double &z) : ReferenceFrameHolder()
{
	this->name = name;
	this->referenceFrame = referenceFrame;
	set(x, y, z);
}

FrameTuple::FrameTuple(const std::string &name, ReferenceFrame* referenceFrame) : ReferenceFrameHolder()
{
	this->name = name;
	this->referenceFrame = referenceFrame;
	setToZero();
}

FrameTuple::FrameTuple(const std::string &name, ReferenceFrame* referenceFrame, double array[3]) : ReferenceFrameHolder()
{
	this->name = name;
	this->referenceFrame = referenceFrame;
	set(array);
}

FrameTuple::FrameTuple(const std::string &name, ReferenceFrame* referenceFrame, const std::vector<double> &vector) : ReferenceFrameHolder()
{
	this->name = name;
	this->referenceFrame = referenceFrame;
	set(vector);
}

FrameTuple::FrameTuple(const FrameTuple &frameTuple) : ReferenceFrameHolder()
{
	this->name = frameTuple.name;
	this->x = frameTuple.x;
	this->y = frameTuple.y;
	this->z = frameTuple.z;
	this->referenceFrame = frameTuple.referenceFrame;
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
	if (vector.size() != 3)
	{
		throw std::runtime_error("Vector size is not equal to 3!");
	}

	set(vector[0], vector[1], vector[2]);
}

void FrameTuple::setX(const double &value)
{
	this->x = value;
}

void FrameTuple::setY(const double &value)
{
	this->y = value;
}

void FrameTuple::setZ(const double &value)
{
	this->z = value;
}


void FrameTuple::setIncludingFrame(const FrameTuple &frameTuple)
{
	this->x = frameTuple.x;
	this->y = frameTuple.y;
	this->z = frameTuple.z;
	this->referenceFrame = frameTuple.referenceFrame;
}

void FrameTuple::setIncludingFrame(ReferenceFrame* referenceFrame, const double &x, const double &y, const double &z)
{
	this->referenceFrame = referenceFrame;
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

void FrameTuple::add(const FrameTuple &frameTuple)
{
	this->referenceFrame->checkReferenceFramesMatch(frameTuple.getReferenceFrame());

	this->x += frameTuple.x;
	this->y += frameTuple.y;
	this->z += frameTuple.z;
}

void FrameTuple::add(const FrameTuple &frameTuple1, const FrameTuple &frameTuple2)
{
	setIncludingFrame(frameTuple1);
	add(frameTuple2);
}

void FrameTuple::subtract(const FrameTuple &frameTuple)
{
	this->referenceFrame->checkReferenceFramesMatch(frameTuple.getReferenceFrame());

	this->x -= frameTuple.x;
	this->y -= frameTuple.y;
	this->z -= frameTuple.z;
}

void FrameTuple::subtract(const FrameTuple &frameTuple1, const FrameTuple &frameTuple2)
{
	setIncludingFrame(frameTuple1);
	subtract(frameTuple2);
}

void FrameTuple::negate()
{
	this->x = -this->x;
	this->y = -this->y;
	this->z = -this->z;
}

void FrameTuple::negate(const FrameTuple &frameTuple)
{
	setIncludingFrame(frameTuple);
	negate();
}

void FrameTuple::scale(const double &value)
{
	scaleXYZ(value, value, value);
}

void FrameTuple::scale(const double &value, const FrameTuple &frameTuple)
{
	setIncludingFrame(frameTuple);
	scale(value);
}

void FrameTuple::scaleXYZ(const double &scaleX, const double &scaleY, const double &scaleZ)
{
	this->x *= scaleX;
	this->y *= scaleY;
	this->z *= scaleZ;
}

void FrameTuple::scaleAdd(const double &value, const FrameTuple &frameTuple)
{
	scale(value);
	add(frameTuple);
}

void FrameTuple::scaleAdd(const double &value, const FrameTuple &frameTuple1, const FrameTuple &frameTuple2)
{
	setIncludingFrame(frameTuple1);
	scale(value);
	add(frameTuple2);
}

bool FrameTuple::equals(const FrameTuple &frameTuple)
{
	checkReferenceFramesMatch(frameTuple.getReferenceFrame());
	if (std::isnan(this->x) || std::isnan(this->y) || std::isnan(this->z) || std::isnan(frameTuple.x) || std::isnan(frameTuple.y) || std::isnan(frameTuple.z))
	{
		return false;
	}

	return (this->x == frameTuple.x && this->y == frameTuple.y && this->z == frameTuple.z);
}

bool FrameTuple::epsilonEquals(const FrameTuple &frameTuple, const double &epsilon)
{
	checkReferenceFramesMatch(frameTuple.getReferenceFrame());
	if (std::isnan(this->x) || std::isnan(this->y) || std::isnan(this->z) || std::isnan(frameTuple.x) || std::isnan(frameTuple.y) || std::isnan(frameTuple.z))
	{
		return false;
	}

	return (fabs(this->x - frameTuple.x) < epsilon && fabs(this->y - frameTuple.y) < epsilon && fabs(this->z - frameTuple.z) < epsilon);
}

void FrameTuple::clampMin(const double &min)
{
	if (this->x < min)
	{
		this->x = min;
	}

	if (this->y < min)
	{
		this->y = min;
	}

	if (this->z < min)
	{
		this->z = min;
	}
}

void FrameTuple::clampMax(const double &max)
{
	if (this->x > max)
	{
		this->x = max;
	}

	if (this->y > max)
	{
		this->y = max;
	}

	if (this->z > max)
	{
		this->z = max;
	}
}

void FrameTuple::clampMinMax(const double &min, const double &max)
{
	if (min > max)
	{
		throw std::runtime_error("Invalid bounds!");
	}

	if (this->x < min)
	{
		this->x = min;
	}
	else
		if (this->x > max)
		{
			this->x = max;
		}

	if (this->y < min)
	{
		this->y = min;
	}
	else
		if (this->y > max)
		{
			this->y = max;
		}

	if (this->z < min)
	{
		this->z = min;
	}
	else
		if (this->z > max)
		{
			this->z = max;
		}
}

void FrameTuple::absoluteValue(const FrameTuple &frameTuple)
{
	setIncludingFrame(frameTuple);
	absoluteValue();
}

void FrameTuple::absoluteValue()
{
	this->x = fabs(this->x);
	this->y = fabs(this->y);
	this->z = fabs(this->z);
}


}