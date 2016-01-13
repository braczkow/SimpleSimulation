#include "Simulation.h"
#include "NeuralNetwork.h"
#include "MotorNeuralNetwork.h"
#include "RoboSimulationModelBase.h"
#include "ManualRoboSimulationModel.h"
#include "RoboSimulationView.h"
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

static std::unique_ptr<robo::RoboSimulationModelBase> roboSimModel;

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
	roboSimModel->step();


	glutSetWindow(mainWindow);
	glutPostRedisplay();
	glutTimerFunc(framePeriod, Timer, 0);
}

static void processKeyDown(unsigned char key, int x, int y)
{
	//cout << "Key: " << key << " down.";

	if (roboSimModel)
	{
		roboSimModel->onKeyboardKeyDown(key);
	}

	if (key == 'u')
	{
		roboSimModel->step();
	}
}

static void processKeyUp(unsigned char key, int x, int y)
{
//	cout << "Key: " << key << " up.";

	if (roboSimModel)
	{
		roboSimModel->onKeyboardKeyUp(key);
	}

}

static void SimulationLoop()
{
	//cout << "SimulationLoop\n";

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//draw the stuff!
	robo::RoboSimulationView view;
	auto shapes = roboSimModel->getShapes();
	view.drawShapes(shapes);

	glutSwapBuffers();
}

const int StatesCount = 5;

int main(int argc, char** argv)
{

	const int generationCount = 10;
	const int membersInGeneration = 48;
	//srand(222);

	//vector<GAMember> topMembers;
	//GeneticAlgorithm* ga = new GeneticAlgorithm(membersInGeneration, 1);

	//for (int i = 0; i < generationCount; i++)
	//{
	//	topMembers.push_back(ga->getTopMember());
	//	cout <<i <<": " << ga->getTopMember().getFitness() << endl;
	//	ga->evolve();
	//}

	//GAMember topMember = topMembers[0];
	//for (size_t i=0; i<topMembers.size(); i++)
	//{
	//	if (topMembers[i].getFitness() > topMember.getFitness())
	//	{
	//		topMember = topMembers[i];
	//	}
	//}
	//robotSimulation = new RobotSimulation(MotorNeuralNetwork(layerCount, neuronsInLayer, topMember.getWeights()));
	//robotSimulation = new RobotSimulation(ActionMachine(StatesCount, topMember.getWeights()));

	//robotSimulation = new RobotSimulation(MotorNeuralNetwork(layerCount, 
	//	neuronsInLayer, generate(MotorNeuralNetwork::calculateNeuronCount(layerCount, neuronsInLayer), -1.0f, 1.0f)));

	/*************************************************************************
	/we have the special one. let's see how it moves
	**************************************************************************/
	//cout << "Learning procedure has finished. Type any character to proceed to graphic presentation.\n";
	//char c;
	//cin >> c;

	//robotSimulation = std::make_unique<robo::RoboSimulationModelBase>(MotorNeuralNetwork(layerCount,
	//	neuronsInLayer, generate(MotorNeuralNetwork::calculateNeuronCount(layerCount, neuronsInLayer), -1.0f, 1.0f)));

	roboSimModel = std::make_unique<robo::ManualRoboSimulationModel>();

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	glutInitWindowSize(width, height);
	char title[32];
	sprintf(title, "Box2D Version %d.%d.%d", b2_version.major, b2_version.minor, b2_version.revision);
	mainWindow = glutCreateWindow(title);

	glutKeyboardFunc(processKeyDown);
	glutKeyboardUpFunc(processKeyUp);

	glutDisplayFunc(SimulationLoop);
	// Use a timer to control the frame rate.
	glutTimerFunc(framePeriod, Timer, 0);

	glutMainLoop();

	system("pause");

	return 0;
}