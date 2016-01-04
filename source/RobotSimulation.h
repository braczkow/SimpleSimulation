#pragma once
#include "Simulation.h"
#include "MotorNeuralNetwork.h"
#include "ActionMachine.h"
#include <iostream>


using namespace std;

class RobotSimulation
{
public:
	RobotSimulation(MotorNeuralNetwork mnn);
	//RobotSimulation(ActionMachine am);

	//RobotSimulation();

	~RobotSimulation()
	{
		delete m_world;
	}

	void run(int nSteps = 1000);
	float getFitness();
	float getV1();
	float getV2();
	void draw();
	void updateMotors();
	void step();

private : 


	void DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color, float scale = 1.0f);
	void DrawPolygon(b2Fixture* fixture, const b2Transform& xf, const b2Color& color);

	void updateFitness();
	float getDistance();
	//void setNewDesiredAngles(pair<int, int> nextState);

	const float roboArmHx;
	const float roboArmHy;

	float _desiredAngleA;
	float _desiredAngleB;


	b2World* m_world;
	b2Body* m_groundBody;

	b2Body* _roboMain;
	b2Body* _roboArm1;
	b2Body* _roboArm2;
	b2RevoluteJoint* _jointA;
	b2RevoluteJoint* _jointB;

	MotorNeuralNetwork _mnn;
	//ActionMachine _am;
	pair<int, int> _currentState;
	

	float _fitness;
};