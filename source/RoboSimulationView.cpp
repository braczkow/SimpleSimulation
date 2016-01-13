#include "RoboSimulationView.h"

#include "freeglut.h"

namespace robo
{
void RoboSimulationView::drawShapes(std::vector<std::shared_ptr<robo::IShape>> shapes)
{
	for (auto s : shapes)
	{
		if (auto rec = std::dynamic_pointer_cast<Rectangular2D>(s))
		{
			drawRectangular(rec, Color(1, 1, 0, 0), 0.1f);
		}
		else if (auto circ = std::dynamic_pointer_cast<Circle2D>(s))
		{
			drawCircle(circ, Color(1, 0, 0, 0), 0.1f);
		}

	}

}

void RoboSimulationView::drawRectangular(std::shared_ptr<Rectangular2D> rec, Color c, float scale)
{
	glColor4f(c.r, c.g, c.b, 1.0f);
	glBegin(GL_POLYGON);

	for (auto v : rec->vertices)
	{
		glVertex2f(v.x * scale, v.y * scale);
	}

	glEnd();
}

void RoboSimulationView::drawCircle(std::shared_ptr<Circle2D> circle, Color c, float scale)
{
	auto x = circle->position.x * scale;
	auto y = circle->position.y * scale;
	auto radius = circle->radius * scale;
	auto angle = circle->angle;

	int trianglesCount = 20;
	GLfloat twicePi = 2.0f * 3.14f;
	
	glColor4f(c.r, c.g, c.b, 1.0f);
	
	glBegin(GL_TRIANGLE_FAN);
	glVertex2f(x, y); // center of circle
	for (auto i = 0; i <= trianglesCount; i++) 
	{
		glVertex2f(
			x + (radius * cos(i *  twicePi / trianglesCount + angle) ),
			y + (radius * sin(i * twicePi / trianglesCount + angle)  )
			);
	
		if (i == trianglesCount / 2 - 1)
		{
			glColor4f(c.g, c.b, c.r, 1.0f);
		}
	}
	glEnd();
}

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

//
//}


}



