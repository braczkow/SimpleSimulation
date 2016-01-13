#pragma once

#include <vector>
namespace robo
{

struct IShape
{
	IShape() {}
	virtual ~IShape() {}
};

struct Vec2d
{
	Vec2d() : x(0), y(0) {}
	Vec2d(float xx, float yy) : x(xx), y(yy) {}

	float x;
	float y;
};

struct Rectangular2D : public IShape
{
	Rectangular2D() {}
	virtual ~Rectangular2D() {}

	std::vector<Vec2d> vertices;
};

struct Circle2D : public IShape
{
	Circle2D() {}
	virtual ~Circle2D() {}

	Vec2d position;
	float radius;
	float angle;
};

struct Color
{
	Color(float rr, float gg, float bb, float aa) :
		r(rr), g(gg), b(bb), a(aa) {}
	float r, g, b, a;
};


}
