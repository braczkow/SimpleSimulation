#pragma once

namespace robo
{
class IRobotSimulationModel
{
public:
	IRobotSimulationModel() {}
	virtual ~IRobotSimulationModel() {}

	virtual void onKeyboardKeyDown(unsigned char aKey) {}

	virtual void onKeyboardKeyUp(unsigned char aKey) {}

	virtual void run() = 0;
};

}
