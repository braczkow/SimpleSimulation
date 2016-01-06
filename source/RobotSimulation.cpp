#include "RobotSimulation.h"
#include "RandomGenerator.h"

#include "freeglut.h"
#include <iostream>
#include <cmath>


//#define DBPRINT
#ifdef DBPRINT
	#define debug_print(fmt, ...) fprintf(stderr, fmt, __VA_ARGS__)
#else
	#define debug_print(fmt, ...) 
#endif

#define DEG_TO_RAD (2*3.14f/360)

using namespace std;

int layerCount = 3;
int neuronsInLayer[] = {5, 8, 2};
extern const int StatesCount;

namespace robo
{

ManualRobotController::ManualRobotController(MotorNeuralNetwork mnn) :
	_mnn(mnn), _currentState(0, 0), _fitness(0.0f), roboArmHx(1.5f), roboArmHy(0.2f)
{
	// Define the gravity vector.
	b2Vec2 gravity(0.0f, -10.0f);

	// Construct a world object, which will hold and simulate the rigid bodies.
	_world = std::make_unique<b2World>(gravity);

	// Define the ground body.
	b2BodyDef groundBodyDef;
	groundBodyDef.position.Set(0.0f, -10.0f);

	_groundBody = _world->CreateBody(&groundBodyDef);

	if (_groundBody == NULL)
	{
		cerr << "groundBody == NULL\n";
	}

	// Define the ground box shape.
	b2PolygonShape groundBox;

	// The extents are the half-widths of the box.
	groundBox.SetAsBox(50.0f, 10.0f);

	// Add the ground fixture to the ground body.
	_groundBody->CreateFixture(&groundBox, 0.0f);

	{
		//static box
		b2BodyDef bodyDef;
		bodyDef.type = b2_staticBody;
		bodyDef.position.Set(-2.0f, 0.5f);

		_staticBox = _world->CreateBody(&bodyDef);

		b2PolygonShape boxShape;
		boxShape.SetAsBox(0.5f, 0.5f);

		b2FixtureDef boxFixtureDef;
		boxFixtureDef.shape = &boxShape;

		_staticBox->CreateFixture(&boxFixtureDef);
	}

	{
		//main robo part
		b2BodyDef bodyDef;
		bodyDef.type = b2_dynamicBody;
		bodyDef.position.Set(0.0f, 1.0f);

		_roboMain = _world->CreateBody(&bodyDef);

		b2PolygonShape boxShape;
		boxShape.SetAsBox(2.0f, 1.0f);

		b2FixtureDef boxFixtureDef;
		boxFixtureDef.shape = &boxShape;
		boxFixtureDef.density = 1.0f;
		boxFixtureDef.friction = 0.05f;
		//boxFixtureDef.filter.groupIndex = 1; //no collision


		_roboMain->CreateFixture(&boxFixtureDef);
	}


	{  //robo arm1
		b2BodyDef bd;
		bd.type = b2_dynamicBody;
		bd.position.Set(2.6f, 3.38f);
		bd.angle = 45.0f * DEG_TO_RAD;

		_roboArm1 = _world->CreateBody(&bd);

		b2PolygonShape boxShape;
		boxShape.SetAsBox(roboArmHx, roboArmHy);

		b2FixtureDef boxFixtureDef;
		boxFixtureDef.shape = &boxShape;
		boxFixtureDef.density = 0.1f;
		boxFixtureDef.friction = 1.0f;
		//boxFixtureDef.filter.groupIndex = 1;
		_roboArm1->CreateFixture(&boxFixtureDef);
	}

	{  //robo arm2
		b2BodyDef bd;
		bd.type = b2_dynamicBody;
		bd.position.Set(4.5f, 3.0f);
		bd.angle = -45.0f * DEG_TO_RAD;

		_roboArm2 = _world->CreateBody(&bd);

		b2PolygonShape boxShape;
		boxShape.SetAsBox(roboArmHx, roboArmHy);

		b2FixtureDef boxFixtureDef;
		boxFixtureDef.shape = &boxShape;
		boxFixtureDef.density = 0.1f;
		boxFixtureDef.friction = 1.0f;
		//boxFixtureDef.filter.groupIndex = 1;
		_roboArm2->CreateFixture(&boxFixtureDef);
	}

	{	// joint A: roboMain + roboArm1
		b2RevoluteJointDef revJointDef;
		revJointDef.bodyA = _roboMain;
		revJointDef.bodyB = _roboArm1;
		revJointDef.collideConnected = false;

		revJointDef.localAnchorA.Set(2, 1);
		revJointDef.localAnchorB.Set(-1.5f, -0.4f);

		revJointDef.referenceAngle = 0 * DEG_TO_RAD;
		revJointDef.enableLimit = true;
		revJointDef.lowerAngle = -45 * DEG_TO_RAD;
		revJointDef.upperAngle = 90 * DEG_TO_RAD;

		revJointDef.enableMotor = true;
		revJointDef.maxMotorTorque = 100;
		revJointDef.motorSpeed = 0;

		_jointA = (b2RevoluteJoint*)_world->CreateJoint(&revJointDef);
	}

	{	// joint B: roboArm1 + roboArm2
		b2RevoluteJointDef revJointDef;
		revJointDef.bodyA = _roboArm1;
		revJointDef.bodyB = _roboArm2;
		revJointDef.collideConnected = false;

		revJointDef.localAnchorA.Set(1.5f, -0.2f);
		revJointDef.localAnchorB.Set(-1.5f, -0.2f);

		revJointDef.referenceAngle = 0;
		revJointDef.enableLimit = true;
		revJointDef.lowerAngle = -120 * DEG_TO_RAD;
		revJointDef.upperAngle = 45 * DEG_TO_RAD;


		revJointDef.enableMotor = true;
		revJointDef.maxMotorTorque = 100;
		revJointDef.motorSpeed = 0.0f;

		_jointB = (b2RevoluteJoint*)_world->CreateJoint(&revJointDef);
	}

	_motorSpeed[0] = 0.0f;
	_motorSpeed[1] = 0.0f;

}

void ManualRobotController::onKeyboardKeyDown(unsigned char aKey) 
{
	int sign = -1;
	float scale = 0.5f;
	switch (aKey)
	{
	case 'a' : 
		_motorSpeed[0] = -scale * sign;
		break;

	case 'z' :
		_motorSpeed[0] = scale * sign;
		break;

	case 's' :
		_motorSpeed[1] = -scale * sign;
		break;

	case 'x' :
		_motorSpeed[1] = scale * sign;
		break;
	}

}

void ManualRobotController::onKeyboardKeyUp(unsigned char aKey) 
{
	switch (aKey)
	{
	case 'a':
		_motorSpeed[0] =  0.0f;
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


void ManualRobotController::updateMotors()
{
	_jointA->SetMotorSpeed(_motorSpeed[0]);
	_jointB->SetMotorSpeed(_motorSpeed[1]);
}


void ManualRobotController::step()
{
	updateMotors();

	//step
	float32 timeStep = 0.05f;
	int32 velocityIterations = 8;
	int32 positionIterations = 3;
	_world->Step(timeStep, velocityIterations, positionIterations);

	//update fitness
	updateFitness();
}

void ManualRobotController::run()
{
	int nSteps = 1;
	for (int i = 0; i < nSteps; i++)
	{
		step();
	}

}
void ManualRobotController::updateFitness()
{
	_fitness += _roboMain->GetLinearVelocity().x;
}

float ManualRobotController::getFitness()
{
	//return _fitness;
	return _roboMain->GetPosition().x;
}

float ManualRobotController::getDistance()
{
	//debug_print("returning = %f\n", _roboMain->GetPosition().x + _roboArm2->GetPosition().x);
	return _roboMain->GetPosition().x; // + _roboArm2->GetPosition().x;
}

void ManualRobotController::DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color, float scale /*=1.0f*/)
{
	//glEnable(GL_BLEND);
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	//glColor4f(0.5f * color.r, 0.5f * color.g, 0.5f * color.b, 0.5f);
	//glBegin(GL_TRIANGLE_FAN);
	//for (int32 i = 0; i < vertexCount; ++i)
	//{
	//	glVertex2f(vertices[i].x * scale, vertices[i].y * scale);
	//}
	//glEnd();
	//glDisable(GL_BLEND);

	glColor4f(color.r, color.g, color.b, 1.0f);
	glBegin(GL_POLYGON);
	for (int32 i = 0; i < vertexCount; ++i)
	{
		glVertex2f(vertices[i].x * scale, vertices[i].y * scale);
	}
	glEnd();
}

void ManualRobotController::DrawPolygon(b2Fixture* fixture, const b2Transform& xf, const b2Color& color)
{

	b2PolygonShape* poly = (b2PolygonShape*)fixture->GetShape();
	int32 vertexCount = poly->GetVertexCount();
	//b2Assert(vertexCount <= b2_maxPolygonVertices);
	b2Vec2 vertices[b2_maxPolygonVertices];

	for (int32 i = 0; i < vertexCount; ++i)
	{
		vertices[i] = b2Mul(xf, poly->m_vertices[i]);
	}

	DrawSolidPolygon(vertices, vertexCount, color, 0.1f);
}

void ManualRobotController::draw()
{
	DrawPolygon(&(_staticBox->GetFixtureList()[0]), _staticBox->GetTransform(), b2Color(1, 1, 0));
	DrawPolygon(&(_roboMain->GetFixtureList()[0]), _roboMain->GetTransform(), b2Color(1, 0, 0));
	DrawPolygon(&(_roboArm1->GetFixtureList()[0]), _roboArm1->GetTransform(), b2Color(0, 1, 0));
	DrawPolygon(&(_roboArm2->GetFixtureList()[0]), _roboArm2->GetTransform(), b2Color(0, 0, 1));
}

} //namespace robo

