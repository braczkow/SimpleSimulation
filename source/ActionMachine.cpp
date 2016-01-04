#include "ActionMachine.h"

using namespace std;

ActionMachine::ActionMachine(int statesCount, vector<float> probabilities) :
	_statesCount(statesCount), _states(statesCount, vector<State>(statesCount))
{
	int currentIndex = 0;
	for (int i=0; i<statesCount; i++)
	{
		for (int j=0; j<statesCount; j++)
		{
			State s;
			s.left = (j-1 > -1 ? abs(probabilities[currentIndex++]) : 0);
			s.right = (j+1 < statesCount ? abs(probabilities[currentIndex++]) : 0);
			s.top = (i-1 > -1 ? abs(probabilities[currentIndex++]) : 0);
			s.bottom = (i+1 < statesCount ? abs(probabilities[currentIndex++]) : 0);

			_states[i][j] = s;
		}
	}
}


ActionMachine::~ActionMachine(void)
{
}

int ActionMachine::getStatesCount()
{
	return _statesCount;
}

pair<int, int> ActionMachine::getNextState(pair<int, int> currentState, float random)
{

	State s = _states[currentState.first][currentState.second];
	float pSum = (s.left + s.right + s.top + s.bottom);

	pair<int, int> result;
	random -= s.left/pSum;
	if (random < 0)
	{
		return pair<int, int>(currentState.first, currentState.second-1);
	}

	random -= s.right/pSum;
	if (random < 0)
	{
		return pair<int, int>(currentState.first, currentState.second+1);
	}

	random -= s.top/pSum;
	if (random < 0)
	{
		return pair<int, int>(currentState.first-1, currentState.second);
	}

	random -= s.bottom/pSum;
	if (random < 0)
	{
		return pair<int, int>(currentState.first+1, currentState.second);
	}

	return pair<int, int> (-1, -1);

	//if (s.left > s.right && s.left > s.top && s.left > s.bottom)
	//{
	//	return pair<int, int>(currentState.first, currentState.second-1);
	//}
	//if (s.right > s.left && s.right > s.top && s.right > s.bottom)
	//{
	//	return pair<int, int>(currentState.first, currentState.second+1);
	//}
	//if (s.top > s.right && s.top > s.left && s.top > s.bottom)
	//{
	//	return pair<int, int>(currentState.first-1, currentState.second);
	//}
	//if (s.bottom > s.right && s.bottom > s.top && s.bottom > s.left)
	//{
	//	return pair<int, int>(currentState.first+1, currentState.second);
	//}

	//return pair<int, int> (-1, -1);
}
