#ifndef BRIDGE_H
#define BRIDGE_H

#include "core/pomdp.h"

/* =============================================================================
 * BridgeState class
 * =============================================================================*/

class BridgeState: public State {
public:
	int position;

	BridgeState();
	BridgeState(int _position);

	string text() const;
};

/* =============================================================================
 * Bridge class
 * =============================================================================*/
/**
 * An agent is at the left end of a bridge of length 10, and he only knows his
 * position (takes values from 0 to 9, with left end = 0) within length 1.
 * He can move left, move right, or call for help, with reward -1, -1, and
 * -position-1-20, with the exception that when at 9, move right has reward 0,
 *  and terminates the game. Calling for help also terminates the game.
 */

class Bridge: public BeliefMDP {
private:
	static int LEFT, RIGHT, HELP, BRIDGELENGTH;
	mutable MemoryPool<BridgeState> memory_pool_;

public:
	Bridge();

	bool Step(State& s, double random_num, int action, double& reward,
		OBS_TYPE& obs) const;
	int NumStates() const;
	int NumActions() const;
	double ObsProb(OBS_TYPE obs, const State& s, int a) const;

	State* CreateStartState(string type = "DEFAULT") const;
	Belief* InitialBelief(const State* start, string type = "DEFAULT") const;

	inline double GetMaxReward() const {
		return 0;
	}

	inline ValuedAction GetMinRewardAction() const {
		return ValuedAction(LEFT, -1);
	}
	ScenarioLowerBound* CreateScenarioLowerBound(string name = "DEFAULT",
		string particle_bound_name = "DEFAULT") const;

	void PrintState(const State& state, ostream& out = cout) const;
	void PrintBelief(const Belief& belief, ostream& out = cout) const;
	void PrintObs(const State& state, OBS_TYPE obs, ostream& out = cout) const;
	void PrintAction(int action, ostream& out = cout) const;

	State* Allocate(int state_id = -1, double weight = 0.0) const;
	State* Copy(const State* particle) const;
	void Free(State* particle) const;
	int NumActiveParticles() const;

	Belief* Tau(const Belief* belief, int action, OBS_TYPE obs) const;
	void Observe(const Belief* belief, int action,
		map<OBS_TYPE, double>& obss) const;
	double StepReward(const Belief* belief, int action) const;

	POMCPPrior* CreatePOMCPPrior(string name = "DEFAULT") const;
};

#endif
