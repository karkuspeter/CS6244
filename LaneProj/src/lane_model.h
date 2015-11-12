#ifndef LANEMODEL_H
#define LANEMODEL_H

#include "core/pomdp.h"
#include "core/mdp.h"
#include "util/coord.h"
#include "util/random.h"

#include "lane_state.h"

#define SPEED_COUNT 3   // no car; speed1; speed2
#define LANE_COUNT 2    // starting in largest lane, want to reach lane 0
#define CELL_COUNT 30

/* =============================================================================
 * LaneModel class
 * =============================================================================*/

class LaneModel: public DSPOMDP {
protected:
	mutable MemoryPool<LaneState> memory_pool_;

	vector<LaneState*> states_;

	mutable vector<ValuedAction> mdp_policy_;

public:
	int lane_count, cell_count, speed_count;

	enum { // action
		A_SWITCH = 0, A_SAME = 1, A_DEC = 2, A_ACC = 3
	};

public:
	LaneModel();

    /* Cell level functions */
    double CellObsProbability(const LaneState& state, int *pos, int speed_obs);
    double CellTransProbability(const LaneState& state, int *pos, int speed_new);
    
    int SampleCellObs(const LaneState& state, int *pos, double rand_num);
    int SampleCellVal(const LaneState& state, int *pos, double rand_num);
    
	/* Returns total number of actions.*/
	virtual int NumActions() const;

	/* Deterministic simulative model.*/
	virtual bool Step(State& state, double rand_num, int action, double& reward,
		OBS_TYPE& obs) const;

	/* Functions related to beliefs and starting states.*/
	virtual double ObsProb(OBS_TYPE obs, const State& state, int action) const;
	State* CreateStartState(string type = "DEFAULT") const;
	Belief* InitialBelief(const State* start, string type = "DEFAULT") const;

	/* Bound-related functions.*/
	double GetMaxReward() const;
	ScenarioUpperBound* CreateScenarioUpperBound(string name = "DEFAULT",
		string particle_bound_name = "DEFAULT") const;
	ValuedAction GetMinRewardAction() const;
	ScenarioLowerBound* CreateScenarioLowerBound(string name = "DEFAULT",
		string particle_bound_name = "DEFAULT") const;

	/* Memory management.*/
	State* Allocate(int state_id, double weight) const;
	State* Copy(const State* particle) const;
	void Free(State* particle) const;
	int NumActiveParticles() const;

	/* Display.*/
	void PrintState(const State& state, ostream& out = cout) const;
	void PrintBelief(const Belief& belief, ostream& out = cout) const;
	void PrintObs(const State& state, OBS_TYPE observation,
		ostream& out = cout) const;
	void PrintAction(int action, ostream& out = cout) const;
};

#endif