#pragma once
#include "IRoboSimulationModel.h"
#include "RoboParts.h"

#include "Box2D/Box2D.h"

#include <iostream>
#include <vector>
#include <memory>

namespace robo 
{

class RoboSimulationModelBase //: public IRobotSimulationModel
{
public:
	RoboSimulationModelBase();

	~RoboSimulationModelBase() { }

	virtual void onKeyboardKeyDown(unsigned char aKey);

	virtual void onKeyboardKeyUp(unsigned char aKey);

	virtual void step() = 0;

	std::vector<std::shared_ptr<RoboPart>> getShapes();

	//float getFitness();
	//void draw();
	//void updateMotors();
	//void step();

protected:
	//void DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color, float scale = 1.0f);
	//void DrawPolygon(b2Fixture* fixture, const b2Transform& transform, const b2Color& color);
	//void DrawCircle(b2Fixture* fixture, const b2Transform& transform, const b2Color& color, float32 scale);

	//void updateFitness();
	//float getDistance();
	//void setNewDesiredAngles(pair<int, int> nextState);

	std::shared_ptr<Rectangular2D> makeRoboRectangularPartDesc(b2Body* body, const std::string name);
	std::shared_ptr<Circle2D> makeRoboCirclePartDesc(b2Body* body, const std::string name);

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

	//float _fitness;

	//float _motorSpeed[2];

};

}

