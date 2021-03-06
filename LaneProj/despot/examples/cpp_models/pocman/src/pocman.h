#ifndef POCMAN_H
#define POCMAN_H

#include "core/pomdp.h"
#include "solver/pomcp.h"
#include "util/coord.h"
#include "util/grid.h"

/* ==============================================================================
 * AdventurerState class
 * ==============================================================================*/

class PocmanState: public State {
public:
	Coord pocman_pos;
	vector<Coord> ghost_pos;
	vector<int> ghost_dir;
	vector<bool> food; // bit vector
	int num_food;
	int power_steps;
};

/* ==============================================================================
 * PocmanBelief class
 * ==============================================================================*/

class Pocman;
class PocmanBelief: public ParticleBelief {
protected:
	const Pocman* pocman_;
public:
	static int num_particles;

	PocmanBelief(vector<State*> particles, const DSPOMDP* model, Belief* prior =
		NULL);
	void Update(int action, OBS_TYPE obs);
};

/* ==============================================================================
 * Pocman class
 * ==============================================================================*/
/**
 * The implementation is adapted from that included in the POMCP software.
 */

class Pocman: public DSPOMDP {
public:
	virtual bool Step(State& state, double rand_num, int action, double& reward,
		OBS_TYPE& observation) const;
	virtual void Validate(const State& state) const;
	int NumActions() const;
	virtual double ObsProb(OBS_TYPE obs, const State& state, int action) const;

	virtual State* CreateStartState(string type = "DEFAULT") const;
	virtual Belief* InitialBelief(const State* start,
		string type = "PARTICLE") const;

	inline double GetMaxReward() const {
		return reward_clear_level_;
	}
	ScenarioUpperBound* CreateScenarioUpperBound(string name = "DEFAULT",
		string particle_bound_name = "DEFAULT") const;

	inline ValuedAction GetMinRewardAction() const {
		return ValuedAction(0, reward_hit_wall_);
	}
	ParticleLowerBound* CreateParticleLowerBound(string name = "DEFAULT") const;
	ScenarioLowerBound* CreateScenarioLowerBound(string name = "DEFAULT",
		string particle_bound_name = "DEFAULT") const;

	POMCPPrior* CreatePOMCPPrior(string name = "DEFAULT") const;

	virtual void PrintState(const State& state, ostream& out = cout) const;
	virtual void PrintObs(const State& state, OBS_TYPE observation,
		ostream& out = cout) const;
	void PrintBelief(const Belief& belief, ostream& out = cout) const;
	virtual void PrintAction(int action, ostream& out = cout) const;

	State* Allocate(int state_id, double weight) const;
	virtual State* Copy(const State* particle) const;
	virtual void Free(State* particle) const;
	int NumActiveParticles() const;

	bool LocalMove(State& state, const History& history, int obs) const;

public:
	Pocman(int xsize, int ysize);

	enum {
		E_PASSABLE, E_SEED, E_POWER
	};

	Grid<int> maze_;
	int num_ghosts_, passage_y_, ghost_range_, smell_range_, hear_range_;
	Coord pocman_home_, ghost_home_;
	double food_prob_, chase_prob_, defensive_slip_;
	double reward_clear_level_, reward_default_, reward_die_;
	double reward_eat_food_, reward_eat_ghost_, reward_hit_wall_;
	int power_num_steps_;
	Coord NextPos(const Coord& from, int dir) const;

private:
	void MoveGhost(PocmanState& pocstate, int g, Random random) const;
	void MoveGhostAggressive(PocmanState& pocstate, int g, Random random) const;
	void MoveGhostDefensive(PocmanState& pocstate, int g, Random random) const;
	void MoveGhostRandom(PocmanState& pocstate, int g, Random random) const;
	void NewLevel(PocmanState& pocstate) const;
	int SeeGhost(const PocmanState& pocstate, int action) const;
	bool HearGhost(const PocmanState& pocstate) const;
	bool SmellFood(const PocmanState& pocstate) const;
	bool Passable(const Coord& pos) const {
		return CheckFlag(maze_(pos), E_PASSABLE);
	}
	int MakeObservations(const PocmanState& pocstate) const;

	mutable MemoryPool<PocmanState> memory_pool_;
};

class MicroPocman: public Pocman {
public:
	MicroPocman();
};

class MiniPocman: public Pocman {
public:
	MiniPocman();
};

class FullPocman: public Pocman {
public:
	FullPocman();
};

#endif
