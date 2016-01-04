#include "RobotSimulation.h"
#include "RandomGenerator.h"

//#include <freeglut\freeglut.h>
#include <iostream>
#include <cmath>


//#define DBPRINT
#ifdef DBPRINT
	#define debug_print(fmt, ...) fprintf(stderr, fmt, __VA_ARGS__)
#else
	#define debug_print(fmt, ...) 
#endif

#define DEG_TO_RAD (2*3.14/360)

using namespace std;

int layerCount = 3;
int neuronsInLayer[] = {5, 8, 2};
extern const int StatesCount;

RobotSimulation::RobotSimulation(MotorNeuralNetwork mnn) : 
	_mnn(mnn), _currentState(0,0), _fitness(0.0f), roboArmHx(1.5f), roboArmHy(0.2)
{
	// Define the gravity vector.
	b2Vec2 gravity(0.0f, -10.0f);

	// Construct a world object, which will hold and simulate the rigid bodies.
	m_world = new b2World(gravity);

	// Define the ground body.
	b2BodyDef groundBodyDef;
	groundBodyDef.position.Set(0.0f, -10.0f);

	m_groundBody = m_world->CreateBody(&groundBodyDef);

	if (m_groundBody == NULL)
	{
		cerr << "groundBody == NULL\n";
	}

	// Define the ground box shape.
	b2PolygonShape groundBox;

	// The extents are the half-widths of the box.
	groundBox.SetAsBox(50.0f, 10.0f);

	// Add the ground fixture to the ground body.
	m_groundBody->CreateFixture(&groundBox, 0.0f);

	{
	 //main robo part
		b2BodyDef bodyDef;
		bodyDef.type = b2_dynamicBody;
		bodyDef.position.Set(0.0f, 1.0f);

		_roboMain = m_world->CreateBody(&bodyDef);

		b2PolygonShape boxShape;
		boxShape.SetAsBox(2.0f, 1.0f);

		b2FixtureDef boxFixtureDef;
		boxFixtureDef.shape = &boxShape;
		boxFixtureDef.density = 0.1f;
		//boxFixtureDef.filter.groupIndex = 1; //no collision


    _roboMain->CreateFixture(&boxFixtureDef);
	}
    
		
	{  //robo arm1
		b2BodyDef bd;
		bd.type = b2_dynamicBody;
		bd.position.Set(2.6f, 3.38f);
		bd.angle = 45.0f * DEG_TO_RAD;

		_roboArm1 = m_world->CreateBody(&bd);

		b2PolygonShape boxShape;
		boxShape.SetAsBox(roboArmHx, roboArmHy);

		b2FixtureDef boxFixtureDef;
		boxFixtureDef.shape = &boxShape;
		boxFixtureDef.density = 1;
		//boxFixtureDef.filter.groupIndex = 1;
		_roboArm1->CreateFixture(&boxFixtureDef);
	}

	{  //robo arm2
		b2BodyDef bd;
		bd.type = b2_dynamicBody;
		bd.position.Set(4.5f, 3.0f);
		bd.angle = -45.0 * DEG_TO_RAD;

		_roboArm2 = m_world->CreateBody(&bd);

		b2PolygonShape boxShape;
		boxShape.SetAsBox(roboArmHx, roboArmHy);

		b2FixtureDef boxFixtureDef;
		boxFixtureDef.shape = &boxShape;
		boxFixtureDef.density = 1;
		//boxFixtureDef.filter.groupIndex = 1;
		_roboArm2->CreateFixture(&boxFixtureDef);
	}

	{	// joint A: roboMain + roboArm1
		b2RevoluteJointDef revJointDef;
		revJointDef.bodyA = _roboMain;
		revJointDef.bodyB = _roboArm1;
		revJointDef.collideConnected = false;

		revJointDef.localAnchorA.Set(2, 1);
		revJointDef.localAnchorB.Set(-1.5, -0.4);

		revJointDef.referenceAngle = 0 * DEG_TO_RAD;
		revJointDef.enableLimit = true;
		revJointDef.lowerAngle = -45 * DEG_TO_RAD;
		revJointDef.upperAngle = 45 * DEG_TO_RAD;

		revJointDef.enableMotor = true;
		revJointDef.maxMotorTorque = 1000;
		revJointDef.motorSpeed = 0;

		_jointA = (b2RevoluteJoint*)m_world->CreateJoint( &revJointDef );
	}

	{	// joint B: roboArm1 + roboArm2
		b2RevoluteJointDef revJointDef;
		revJointDef.bodyA = _roboArm1;
		revJointDef.bodyB = _roboArm2;
		revJointDef.collideConnected = false;

		revJointDef.localAnchorA.Set(1.5, -0.2);
		revJointDef.localAnchorB.Set(-1.5, -0.2);

		revJointDef.referenceAngle = 0;
		revJointDef.enableLimit = true;
		revJointDef.lowerAngle = - 45 * DEG_TO_RAD;
		revJointDef.upperAngle =  45 * DEG_TO_RAD;

			
		revJointDef.enableMotor = true;
		revJointDef.maxMotorTorque = 1000;
		revJointDef.motorSpeed =  0.0f;

		_jointB = (b2RevoluteJoint*)m_world->CreateJoint( &revJointDef );
	}

	//set first state
	//setNewDesiredAngles(pair<int, int>(1,1));
	//_jointA->SetMotorSpeed((_desiredAngleA - _jointA->GetJointAngle() > 0 ? 0.1 : -0.1));
	//_jointB->SetMotorSpeed((_desiredAngleB - _jointB->GetJointAngle() > 0 ? 0.1 : -0.1));
	//_currentState = pair<int, int>(1,1);

}


void RobotSimulation::updateMotors()
{
	vector<float> input;

	input.push_back(sin(_jointA->GetJointAngle()));
	input.push_back(sin(_jointB->GetJointAngle()));
		
	input.push_back(_roboArm2->GetPosition().x - _roboArm1->GetPosition().x);
	input.push_back(_roboArm1->GetPosition().x - _roboMain->GetPosition().x);
	input.push_back(_roboArm2->GetPosition().x - _roboArm1->GetPosition().x);


	vector<float> motorSpeed = _mnn.calculateOutput(input);
	debug_print("Motor speed = (%f, %f)\n", motorSpeed[0]*2*3.14, motorSpeed[1]*2*3.14);
	

	_jointA->SetMotorSpeed(motorSpeed[0]);
	_jointB->SetMotorSpeed(motorSpeed[1]);
	//float angleA = _jointA->GetJointAngle();
	//float angleB = _jointB->GetJointAngle();
	//float deltaA = abs(_desiredAngleA - _jointA->GetJointAngle());
	//float deltaB = abs(_desiredAngleB - _jointB->GetJointAngle());

	//if (deltaA < 0.1)
	//{
	//	_jointA->SetMotorSpeed(0);
	//}
	//else
	//{
	//	_jointA->SetMotorSpeed((_desiredAngleA - _jointA->GetJointAngle()) > 0 ? 0.1 : -0.1);
	//}

	//if (deltaB < 0.1)
	//{
	//	_jointB->SetMotorSpeed(0);
	//}
	//else
	//{
	//	_jointB->SetMotorSpeed((_desiredAngleB - _jointB->GetJointAngle()) > 0 ? 0.1 : -0.1);
	//}



	//if (deltaA < 0.1 &&  deltaB < 0.1)
	//{
	//	float random = RandomGenerator::GenerateUniform();
	//	pair<int, int> nextState = _am.getNextState(_currentState, random);
	//	debug_print("(%d,%d)\n", nextState.first, nextState.second);

	//	//if (nextState == pair<int, int>(-1, -1))
	//	//{
	//	//	throw;
	//	//}

	//	setNewDesiredAngles(nextState);
	//	
	//	//_jointA->SetMotorSpeed((_desiredAngleA - _jointA->GetJointAngle() > 0 ? 0.1 : -0.1));
	//	//_jointB->SetMotorSpeed((_desiredAngleB - _jointB->GetJointAngle() > 0 ? 0.1 : -0.1));

	//	_currentState = nextState;
	//}



}

//void RobotSimulation::setNewDesiredAngles(pair<int, int> nextState)
//{
//	int statesCount = _am.getStatesCount();
//	float lowerLimitA = _jointA->GetLowerLimit();
//	float upperLimitA = _jointA->GetUpperLimit();
//	float angleStepA = (upperLimitA - lowerLimitA)/((float)statesCount-1);
//
//	float lowerLimitB = _jointB->GetLowerLimit();
//	float upperLimitB = _jointB->GetUpperLimit();
//	float angleStepB = (upperLimitB - lowerLimitB)/((float)statesCount-1);
//
//	_desiredAngleA = lowerLimitA + (nextState.first) * angleStepA;
//	_desiredAngleB = lowerLimitB + (nextState.second) * angleStepB;
//}

void RobotSimulation::step()
{
	updateMotors();

	//step
	float32 timeStep = 0.1f;
	int32 velocityIterations = 6;
	int32 positionIterations = 2;
	m_world->Step(timeStep, velocityIterations, positionIterations);

	//update fitness
	updateFitness();

}

void RobotSimulation::run(int nSteps /*=1000*/) 
{
	for (int i = 0; i < nSteps; i++)
	{
		step();
	}

}
void RobotSimulation::updateFitness()
{
	//float bodyY = _roboArm2->GetPosition().y;

	//float angle = _roboArm2->GetAngle();
	//float sinA = sin(angle * 360 /(2*3.14));
	//float cosA = cos(angle * 360 /(2*3.14));
	//float dh = roboArmHx * sinA + roboArmHy*cosA;

	//float roboArm2y = bodyY - dh;

	////if it's touching the ground then add velocity?
	////_fitness += _roboMain->GetLinearVelocity().x; 

	//_fitness += (roboArm2y < 1 ? 1 : 0) * _roboMain->GetLinearVelocity().x;
	////debug_print("roboArm2y = %f, bodyY = %f", roboArm2y, bodyY);

	_fitness += _roboMain->GetLinearVelocity().x;


}

float RobotSimulation::getFitness()
{
	//return _fitness;
	return _roboMain->GetPosition().x;
}

float RobotSimulation::getV1()
{
	return _jointA->GetJointSpeed();
}

float RobotSimulation::getV2()
{
	return _jointB->GetJointSpeed();
}

float RobotSimulation::getDistance()
{
	//debug_print("returning = %f\n", _roboMain->GetPosition().x + _roboArm2->GetPosition().x);
	return _roboMain->GetPosition().x; // + _roboArm2->GetPosition().x;
}


void RobotSimulation::DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color, float scale /*=1.0f*/)
{
	//glEnable(GL_BLEND);
	//glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	//glColor4f(0.5f * color.r, 0.5f * color.g, 0.5f * color.b, 0.5f);
	//glBegin(GL_TRIANGLE_FAN);
	//for (int32 i = 0; i < vertexCount; ++i)
	//{
	//	glVertex2f(vertices[i].x * scale, vertices[i].y * scale);
	//}
	//glEnd();
	//glDisable(GL_BLEND);

	//glColor4f(color.r, color.g, color.b, 1.0f);
	//glBegin(GL_LINE_LOOP);
	//for (int32 i = 0; i < vertexCount; ++i)
	//{
	//	glVertex2f(vertices[i].x, vertices[i].y);
	//}
	//glEnd();
}


void RobotSimulation::DrawPolygon(b2Fixture* fixture, const b2Transform& xf, const b2Color& color)
{

	//b2PolygonShape* poly = (b2PolygonShape*)fixture->GetShape();
	////int32 vertexCount = poly->m_vertexCount;
	//b2Assert(vertexCount <= b2_maxPolygonVertices);
	//b2Vec2 vertices[b2_maxPolygonVertices];

	//for (int32 i = 0; i < vertexCount; ++i)
	//{
	//	vertices[i] = b2Mul(xf, poly->m_vertices[i]);
	//}

	//DrawSolidPolygon(vertices, vertexCount, color, 0.1);
}

void RobotSimulation::draw()
{
	DrawPolygon(&(_roboMain->GetFixtureList()[0]), _roboMain->GetTransform(), b2Color(1, 0, 0)); 
	DrawPolygon(&(_roboArm1->GetFixtureList()[0]), _roboArm1->GetTransform(), b2Color(0, 1, 0)); 
	DrawPolygon(&(_roboArm2->GetFixtureList()[0]), _roboArm2->GetTransform(), b2Color(0, 0, 1)); 
}