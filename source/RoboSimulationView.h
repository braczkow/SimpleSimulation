#pragma once

#include "ConfigReader.h"
#include "RoboParts.h"
#include <memory>

namespace robo
{
class RoboSimulationView
{
public:
	RoboSimulationView(const RoboConfig& rc) :
	roboConfig(rc) {}

	virtual ~RoboSimulationView() {}

	void drawShapes(std::vector<std::shared_ptr<robo::RoboPart> > shapes);

	void drawRectangular(std::shared_ptr<Rectangular2D>, Color c, float scale = 1.0f);

	void drawCircle(std::shared_ptr<Circle2D>, Color c, float scale = 1.0f);

private:
	RoboConfig roboConfig;

};


}
