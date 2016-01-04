#pragma once
#include <vector>
#include <map>
#include <utility>

using namespace std;

class State
{
public:
	State(float l = 0, float r = 0, float t = 0, float b = 0) :
		left(l), right(r), top(t), bottom(b) {}

	float left;
	float right;
	float top;
	float bottom;

};

class ActionMachine
{
public:
	ActionMachine(int statesCount, vector<float> probabilities);
	pair<int, int> getNextState(pair<int, int> currentState, float random);
	int getStatesCount();

	static int calculateProbabilitiesCount (int statesCount) {return (4*statesCount*statesCount - 4*statesCount);}
	~ActionMachine(void);

private:
	vector<vector<State> > _states;
	int _statesCount;
};

