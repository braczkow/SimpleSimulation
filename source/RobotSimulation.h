#pragma once
#include "IRobotController.h"
#include "Simulation.h"
#include "MotorNeuralNetwork.h"
#include "ActionMachine.h"
#include <iostream>

namespace robo 
{
class ManualRobotController : public IRobotController
{
public:
	ManualRobotController(MotorNeuralNetwork mnn);


	~ManualRobotController()
	{
	}

	virtual void onKeyboardKeyDown(unsigned char aKey);

	virtual void onKeyboardKeyUp(unsigned char aKey);

	virtual void run();
	float getFitness();
	void draw();
	void updateMotors();
	void step();

private:
	void DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color, float scale = 1.0f);
	void DrawPolygon(b2Fixture* fixture, const b2Transform& transform, const b2Color& color);
	void DrawCircle(b2Fixture* fixture, const b2Transform& transform, const b2Color& color, float32 scale);

	void updateFitness();
	float getDistance();
	//void setNewDesiredAngles(pair<int, int> nextState);

	const float roboArmHx;
	const float roboArmHy;

	float _desiredAngleA;
	float _desiredAngleB;


	std::unique_ptr<b2World> _world;
	b2Body* _groundBody;

	b2Body* _staticBox;
	b2Body* _roboMain;
	b2Body* _roboArm1;
	b2Body* _roboArm2;
	b2RevoluteJoint* _jointA;
	b2RevoluteJoint* _jointB;

	b2Body* _roboWheel;
	b2RevoluteJoint* _jointC;

	MotorNeuralNetwork _mnn;
	//ActionMachine _am;
	pair<int, int> _currentState;

	float _fitness;

	float _motorSpeed[2];

};

}

