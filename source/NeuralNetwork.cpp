#include "NeuralNetwork.h"

using namespace std;

NeuralNetwork::~NeuralNetwork(void)
{
	for (size_t i = 0; i<_layers.size(); i++)
	{
		delete _layers[i];
	}

	
}

