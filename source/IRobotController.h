#pragma once

namespace robo
{
class IRobotController
{
public:
	IRobotController() {}
	virtual ~IRobotController() {}

	virtual void onKeyboardKeyDown(unsigned char aKey) {}

	virtual void onKeyboardKeyUp(unsigned char aKey) {}

	virtual void run() = 0;
};

}
