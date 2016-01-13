#pragma once

#include "RoboSimulationModelBase.h"

namespace robo
{

class NNRoboSimulationModel : public RoboSimulationModelBase
{
public:
	NNRoboSimulationModel() {}

	virtual ~NNRoboSimulationModel() {}

	virtual void step() {}

private:
};


}