#pragma once

#include "RoboSimulationModelBase.h"

namespace robo
{

class NNRoboSimulationModel : public RoboSimulationModelBase
{
public:
	NNRoboSimulationModel(const RoboConfig& roboConf) : 
	RoboSimulationModelBase(roboConf) {}

	virtual ~NNRoboSimulationModel() {}

	virtual void step() {}

private:
};


}