#include "ManualRoboSimulationModel.h"


namespace robo
{
ManualRoboSimulationModel::ManualRoboSimulationModel()
{
	_jointA->EnableMotor(true);
	_jointB->EnableMotor(true);

	_jointA->SetMotorSpeed(0.0f);
	_jointB->SetMotorSpeed(0.0f);
}
void ManualRoboSimulationModel::step()
{
	updateMotors();

	float timeStep = 0.05f;
	int velocityIterations = 8;
	int positionIterations = 3;
	_world->Step(timeStep, velocityIterations, positionIterations);
}


void ManualRoboSimulationModel::onKeyboardKeyDown(unsigned char aKey)
{
	int sign = -1;
	float scale = 0.5f;
	switch (aKey)
	{
	case 'a':
		_motorSpeed[0] = -scale * sign;
		break;

	case 'z':
		_motorSpeed[0] = scale * sign;
		break;

	case 's':
		_motorSpeed[1] = -scale * sign;
		break;

	case 'x':
		_motorSpeed[1] = scale * sign;
		break;
	}

}

void ManualRoboSimulationModel::onKeyboardKeyUp(unsigned char aKey)
{
	switch (aKey)
	{
	case 'a':
		_motorSpeed[0] = 0.0f;
		break;

	case 'z':
		_motorSpeed[0] = 0.0f;
		break;

	case 's':
		_motorSpeed[1] = 0.0f;
		break;

	case 'x':
		_motorSpeed[1] = 0.0f;
		break;
	}
}

void ManualRoboSimulationModel::updateMotors()
{
	if (_jointA)
		_jointA->SetMotorSpeed(_motorSpeed[0]);

	if (_jointB)
		_jointB->SetMotorSpeed(_motorSpeed[1]);
}



} //namespace robo

