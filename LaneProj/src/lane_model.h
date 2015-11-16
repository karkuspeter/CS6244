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


typedef struct {int lane; int cell; int speed;} CarPos;

/* =============================================================================
 * LaneState class
 * =============================================================================*/

class LaneState: public State {
public:
	//vector< CarPos > cars; // index by lane/cell
	CarPos cars[4];
	int speed;
	int lane;

	LaneState();
	LaneState(vector< CarPos > _cars, int _speed, int _lane);
	~LaneState();

	string text() const;
};

class LaneModel;

class LaneBelief: public Belief {
protected:
	const LaneModel* lanemodel_;
public:
	vector<ParticleBelief* > car_beliefs;

	LaneBelief(vector< vector<State*> > particles, const DSPOMDP* model, Belief* prior, bool split);
	LaneBelief(vector<ParticleBelief* > car_beliefs_, const LaneModel* lanemod);
	void Update(int action, OBS_TYPE obs);

	virtual ~LaneBelief();
	virtual vector<State*> Sample(int num) const;
	virtual Belief* MakeCopy() const;
	virtual string text() const;
};


/* =============================================================================
 * LaneModel class
 * =============================================================================*/

class LaneModel: public DSPOMDP {
protected:
	mutable MemoryPool<LaneState> memory_pool_;

	vector<LaneState*> states_;

	mutable vector<ValuedAction> mdp_policy_;

public:
	int lane_count, cell_count, cell_offset, max_cell, speed_count, car_count, starting_lane;

	enum { // action
		A_SWITCH = 0, A_KEEP = 1, A_DEC = 2, A_ACC = 3
	};

public:
	LaneModel();
	//~LaneModel();

    /* Cell level functions */
    /*double CellObsProbability(const LaneState& state, int *pos, int speed_obs);
    double CellTransProbability(const LaneState& state, int *pos, int speed_new);
    
    int SampleCellObs(const LaneState& state, int *pos, double rand_num);
    int SampleCellVal(const LaneState& state, int *pos, double rand_num);*/

	CarPos PickRandomCarPos(Random &random) const;
	OBS_TYPE EncodeObs(vector<CarPos> observation) const;
	vector<CarPos> DecodeObs(OBS_TYPE observation) const;

	vector< vector<int> > ObserveState(State& state, Random random);
    
	/* Returns total number of actions.*/
	virtual int NumActions() const;

	/* Deterministic simulative model.*/
	virtual bool Step(State& state, double rand_num, int action, double& reward,
		OBS_TYPE& obs) const;

	/* Functions related to beliefs and starting states.*/
	virtual double ObsProb(OBS_TYPE obs, const State& state, int action) const;
	State* RandomState(const LaneState* start) const;
	State* CreateStartState(string type = "DEFAULT") const;
	Belief* InitialBelief(const State* start, string type = "DEFAULT") const;

	/* Bound-related functions.*/
	double GetMaxReward() const;
	ValuedAction GetMinRewardAction() const;


	ScenarioUpperBound* CreateScenarioUpperBound(string name = "DEFAULT",
		string particle_bound_name = "DEFAULT") const;
	ScenarioLowerBound* CreateScenarioLowerBound(string name = "DEFAULT",
		string particle_bound_name = "DEFAULT") const;


	/* Memory management.*/
	State* Allocate(int state_id, double weight) const;
	State* Copy(const State* particle) const;
	void Free(State* particle) const;
	int NumActiveParticles() const;

	/* Display.*/
	void PrintCars(const CarPos cars[4], ostream& out = cout, int myspeed = 0) const;
	void PrintState(const State& state, ostream& out = cout) const;
	void PrintBelief(const Belief& belief, ostream& out = cout) const;
	void PrintObs(const State& state, OBS_TYPE observation,
		ostream& out = cout) const;
	void PrintAction(int action, ostream& out = cout) const;

	void UnitTest();

	inline virtual DSPOMDP* MakeCopy() const {
		return new LaneModel();
	}
};

#endif
