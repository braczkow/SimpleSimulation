#include "RoboSimulationModelBase.h"
#include "RandomGenerator.h"
#include "Box2D/Box2D.h"

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

RoboSimulationModelBase::RoboSimulationModelBase() :
	 roboArmHx(1.5f), roboArmHy(0.2f),
	_roboMain(nullptr), _roboArm1(nullptr), _roboArm2(nullptr), _jointA(nullptr), _jointB(nullptr),
	_roboWheel(nullptr), _jointC(nullptr)
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

	bool enableRobo = true;

	if (enableRobo)
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

	if (enableRobo)
	{
		//main robo part
		b2BodyDef bodyDef;
		bodyDef.type = b2_dynamicBody;
		bodyDef.position.Set(2.0f, 1.0f);

		_roboMain = _world->CreateBody(&bodyDef);

		b2PolygonShape boxShape;
		boxShape.SetAsBox(2.0f, 1.0f);

		b2FixtureDef boxFixtureDef;
		boxFixtureDef.shape = &boxShape;
		boxFixtureDef.density = 1.0f;
		boxFixtureDef.friction = 1.0f;

		_roboMain->CreateFixture(&boxFixtureDef);
	}

	if(enableRobo)
	{  //robo arm1
		b2BodyDef bd;
		bd.type = b2_dynamicBody;
		bd.position.Set(4.6f, 3.38f);
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

	if (enableRobo)
	{  //robo arm2
		b2BodyDef bd;
		bd.type = b2_dynamicBody;
		bd.position.Set(6.5f, 3.0f);
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
	
	if (enableRobo)
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

		revJointDef.enableMotor = false;
		revJointDef.maxMotorTorque = 100;
		revJointDef.motorSpeed = 0;

		_jointA = (b2RevoluteJoint*)_world->CreateJoint(&revJointDef);
	}

	if(enableRobo)
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


		revJointDef.enableMotor = false;
		revJointDef.maxMotorTorque = 100;
		revJointDef.motorSpeed = 0.0f;

		_jointB = (b2RevoluteJoint*)_world->CreateJoint(&revJointDef);
	}


	if (enableRobo)
	{  //robo wheel
		b2Vec2 position(0.0f, 0.5f);

		b2BodyDef bd;
		bd.type = b2_dynamicBody;
		bd.position.Set(position.x, position.y);

		_roboWheel = _world->CreateBody(&bd);

		b2CircleShape circleShape;
		circleShape.m_radius = 0.5f;

		b2FixtureDef boxFixtureDef;
		boxFixtureDef.shape = &circleShape;
		boxFixtureDef.density = 1.0f;
		boxFixtureDef.friction = 1.0f;
		_roboWheel->CreateFixture(&boxFixtureDef);
	}

	if (enableRobo)
	{	// joint X: carBox + carWheel
		b2RevoluteJointDef revJointDef;
		revJointDef.bodyA = _roboMain;
		revJointDef.bodyB = _roboWheel;
		revJointDef.collideConnected = false;

		revJointDef.localAnchorA.Set(-2, -1);
		revJointDef.localAnchorB.Set(0, 0);

		//revJointDef.enableMotor = true;
		//revJointDef.maxMotorTorque = 10000;
		//revJointDef.motorSpeed = 1;

		_jointC = (b2RevoluteJoint*)_world->CreateJoint(&revJointDef);
	}

}

void RoboSimulationModelBase::onKeyboardKeyDown(unsigned char aKey) 
{

}

void RoboSimulationModelBase::onKeyboardKeyUp(unsigned char aKey) 
{

}


std::vector<std::shared_ptr<IShape>> RoboSimulationModelBase::getShapes()
{
	std::vector<std::shared_ptr<IShape>> shapes;

	auto wheelShape = std::make_shared<Circle2D>();
	wheelShape->angle = _roboWheel->GetTransform().q.GetAngle();
	wheelShape->position.x = _roboWheel->GetTransform().p.x;
	wheelShape->position.y = _roboWheel->GetTransform().p.y;
	wheelShape->radius = ((b2CircleShape*)_roboWheel->GetFixtureList()[0].GetShape())->m_radius;
	shapes.push_back(wheelShape);

	auto mainShape = makeRectangularShape(_roboMain);
	shapes.push_back(mainShape);

	auto arm1Shape = makeRectangularShape(_roboArm1);
	shapes.push_back(arm1Shape);

	auto arm2Shape = makeRectangularShape(_roboArm2);
	shapes.push_back(arm2Shape);

	auto staticShape = makeRectangularShape(_staticBox);
	shapes.push_back(staticShape);

	return shapes;
}

std::shared_ptr<Rectangular2D> RoboSimulationModelBase::makeRectangularShape(b2Body* body)
{
	auto recShape = std::make_shared<Rectangular2D>();
	auto fixture = (b2PolygonShape*)body->GetFixtureList()[0].GetShape();
	auto trans = body->GetTransform();

	for (int32 i = 0; i < fixture->GetVertexCount(); ++i)
	{
		auto v = b2Mul(trans, fixture->m_vertices[i]);
		recShape->vertices.push_back(robo::Vec2d(v.x, v.y));
	}

	return recShape;
}

//void RoboSimulationModelBase::updateFitness()
//{
//	//_fitness += _roboMain->GetLinearVelocity().x;
//}
//
//float RoboSimulationModelBase::getFitness()
//{
//	//return _fitness;
//	return _roboMain->GetPosition().x;
//}
//
//float RoboSimulationModelBase::getDistance()
//{
//	//debug_print("returning = %f\n", _roboMain->GetPosition().x + _roboArm2->GetPosition().x);
//	return _roboMain->GetPosition().x; // + _roboArm2->GetPosition().x;
//}

//void RoboSimulationModelBase::DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color, float scale /*=1.0f*/)
//{
//	glColor4f(color.r, color.g, color.b, 1.0f);
//	glBegin(GL_POLYGON);
//	for (int32 i = 0; i < vertexCount; ++i)
//	{ 
//		glVertex2f(vertices[i].x * scale, vertices[i].y * scale);
//	}
//	glEnd();
//}

//void RoboSimulationModelBase::DrawPolygon(b2Fixture* fixture, const b2Transform& transform, const b2Color& color)
//{
//
//	b2PolygonShape* poly = (b2PolygonShape*)fixture->GetShape();
//	int32 vertexCount = poly->GetVertexCount();
//	b2Vec2 vertices[b2_maxPolygonVertices];
//
//	for (int32 i = 0; i < vertexCount; ++i)
//	{
//		vertices[i] = b2Mul(transform, poly->m_vertices[i]);
//	}
//
//	DrawSolidPolygon(vertices, vertexCount, color, 0.1f);
//}
//
//void RoboSimulationModelBase::DrawCircle(b2Fixture* fixture, const b2Transform& transform, const b2Color& color, float32 scale)
//{
//	auto circle = (b2CircleShape*)fixture->GetShape();
//
//	auto radius = circle->m_radius * scale;
//	auto circlePosition = transform.p;
//
//	float32 x = circlePosition.x * scale;
//	float32 y = circlePosition.y * scale;
//
//	int trianglesCount = 20;
//	GLfloat twicePi = 2.0f * 3.14f;
//
//
//	auto angle = transform.q.GetAngle();
//
//	glColor4f(color.r, color.g, color.b, 1.0f);
//
//	glBegin(GL_TRIANGLE_FAN);
//	glVertex2f(x, y); // center of circle
//	for (auto i = 0; i <= trianglesCount; i++) 
//	{
//		glVertex2f(
//			x + (radius * cos(i *  twicePi / trianglesCount + angle) ),
//			y + (radius * sin(i * twicePi / trianglesCount + angle)  )
//			);
//
//		if (i == trianglesCount / 2 - 1)
//		{
//			glColor4f(color.g, color.b, color.r, 1.0f);
//		}
//	}
//	glEnd();
//
//}
//
//void RoboSimulationModelBase::draw()
//{
//	if (_staticBox)
//		DrawPolygon(&(_staticBox->GetFixtureList()[0]), _staticBox->GetTransform(), b2Color(1, 1, 0));
//
//
//	DrawPolygon(&(_roboMain->GetFixtureList()[0]), _roboMain->GetTransform(), b2Color(1, 0, 0));
//	DrawPolygon(&(_roboArm1->GetFixtureList()[0]), _roboArm1->GetTransform(), b2Color(0, 1, 0));
//	DrawPolygon(&(_roboArm2->GetFixtureList()[0]), _roboArm2->GetTransform(), b2Color(0, 0, 1));
//
//
//	if (_roboWheel)
//		DrawCircle(&(_roboWheel->GetFixtureList()[0]), _roboWheel->GetTransform(), b2Color(1, 0, 1), 0.1f);
//
//}

} //namespace robo

