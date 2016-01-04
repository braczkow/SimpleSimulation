#include "Simulation.h"
#include "NeuralNetwork.h"
#include "MotorNeuralNetwork.h"
#include "RobotSimulation.h"
#include "GeneticAlgorithm.h"
#include "TestSimulation.h"
#include "ActionMachine.h"
#include "RandomGenerator.h"

#include <iostream>
#include <vector>
#include <Box2D/Box2D.h>
#include "freeglut.h"
#include <time.h>

using namespace std;

namespace
{
	int32 width = 640;
	int32 height = 480;
	int32 framePeriod = 16;
	int32 mainWindow;
	float settingsHz = 60.0;
	float32 viewZoom = 1.0f;
	int tx, ty, tw, th;
	bool rMouseDown;
	b2Vec2 lastp;
}

static RobotSimulation * robotSimulation;




vector<float> generate(int count, float min = 0.0f, float max=1.0f)
{
	vector<float> result;
	for (int i=0; i<count; i++)
	{
		result.push_back(RandomGenerator::GenerateUniform(min, max));
	}

	return result;
}

static void Timer(int)
{
	//cout << "Timer\n";
	robotSimulation->step();

	//cout << "V1: " << robotSimulation->getV1() << "\tV2: " << robotSimulation->getV2()  <<endl;

	glutSetWindow(mainWindow);
	glutPostRedisplay();
	glutTimerFunc(framePeriod, Timer, 0);
}

static void SimulationLoop()
{
	//cout << "SimulationLoop\n";
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//glMatrixMode(GL_MODELVIEW);
	//glLoadIdentity();

	////draw the stuff!
	//robotSimulation->draw();

	//glutSwapBuffers();
}

const int StatesCount = 5;

int main(int argc, char** argv)
{

	const int generationCount = 10;
	const int membersInGeneration = 48;
	//srand(222);

	vector<GAMember> topMembers;
	GeneticAlgorithm* ga = new GeneticAlgorithm(membersInGeneration, 1);

	for (int i = 0; i < generationCount; i++)
	{
		topMembers.push_back(ga->getTopMember());
		cout <<i <<": " << ga->getTopMember().getFitness() << endl;
		ga->evolve();
	}

	GAMember topMember = topMembers[0];
	for (int i=0; i<topMembers.size(); i++)
	{
		if (topMembers[i].getFitness() > topMember.getFitness())
		{
			topMember = topMembers[i];
		}
	}
	robotSimulation = new RobotSimulation(MotorNeuralNetwork(layerCount, neuronsInLayer, topMember.getWeights()));
	//robotSimulation = new RobotSimulation(ActionMachine(StatesCount, topMember.getWeights()));

	//robotSimulation = new RobotSimulation(MotorNeuralNetwork(layerCount, 
	//	neuronsInLayer, generate(MotorNeuralNetwork::calculateNeuronCount(layerCount, neuronsInLayer), -1.0f, 1.0f)));

	/*************************************************************************
	/we have the special one. let's see how it moves
	**************************************************************************/
	cout << "Learning procedure has finished. Type any character to procede to graphic presentation.\n";
	char c;
	cin >> c;

	//glutInit(&argc, argv);
	//glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	//glutInitWindowSize(width, height);
	//char title[32];
	//sprintf(title, "Box2D Version %d.%d.%d", b2_version.major, b2_version.minor, b2_version.revision);
	//mainWindow = glutCreateWindow(title);

	//glutDisplayFunc(SimulationLoop);
	//// Use a timer to control the frame rate.
	//glutTimerFunc(framePeriod, Timer, 0);

	//glutMainLoop();

	system("pause");

	delete robotSimulation;
	return 0;
}