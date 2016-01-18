#pragma once

#include "RoboSimulationModelBase.h"

namespace robo
{

class ManualRoboSimulationModel : public RoboSimulationModelBase
{
public:
	ManualRoboSimulationModel(const RoboConfig& roboConf);

	virtual ~ManualRoboSimulationModel() {}

	virtual void step();

	virtual void onKeyboardKeyDown(unsigned char aKey);

	virtual void onKeyboardKeyUp(unsigned char aKey);

private:
	void updateMotors();

	float _motorSpeed[2];
};


}