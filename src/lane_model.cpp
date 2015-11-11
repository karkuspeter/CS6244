#include "lane_model.h"

using namespace std;

/* =============================================================================
 * LaneState class
 * =============================================================================*/

LaneState::LaneState() {
}

LaneState::LaneState(int* _cells, int* _pos) {
    // Pos of our car and whole world
    cells = _cells
    pos = _pos
    // TODO deep copy
}

LaneState::~LaneState() {
}

string LaneState::text() const {
	return "robot position = " + to_string(pos[0]) + " " + to_string(pos[1]);
}

/* =============================================================================
 * LaneModel class
 * =============================================================================*/

LaneModel::LaneModel() {
}

double LaneModel::CellObsProbability(const LaneState& state, int *pos, int speed_obs){
    // compute the probability that we ovserve [speed_obs] at cell [pos] given the current state [state]
    return 0;
}

double LaneModel::CellTransProbability(const LaneState& state, int *pos, int speed_new){
    // compute the probability that the next cell value is [speed_new] at cell [pos] given the last state [state]
    return 0;
}

int LaneModel::SampleCellObs(const LaneState& state, int *pos, double rand_num){
    // sample from the possible observations at cell [pos] given the current state [state]
    // use CellObsProbability
    return 0;
}

int LaneModel::SampleCellVal(const LaneState& state, int *pos, double rand_num){
    // sample from the possible spped values at cell [pos] given the last state [state]
    // use CellTransProbability
    return 0;
}


/* ======
 * Action
 * ======*/

int LaneModel::NumActions() const {
	return SPEED_COUNT;
}

/* ==============================
 * Deterministic simulative model
 * ==============================*/

bool LaneModel::Step(State& state, double rand_num, int action,
	double& reward, OBS_TYPE& obs) const {
	
	LaneState& lane_state = static_cast<LaneState&>(state);

    // create enough number of random numbers
    // one for each lane/cell for the transition
    // one for each lane/cell for observation
    double rand_t[LANE_COUNT][CELL_COUNT];
    double rand_o[LANE_COUNT][CELL_COUNT];
    
    int next_cells[LANE_COUNT][CELL_COUNT];
    int next_pos[2]
    int obs_cells[LANE_COUNT][CELL_COUNT];
    
    for (int l = 0; l < LANE_COUNT; l++){
        for(int c = 0; c < CELL_COUNT; c++){
            next_cells[l][c] = SampleCellVal(lane_state, {l, c}, rand_t[l][c])
        }
    }
    if action == 0 {
        //switch lane
        next_pos[0] = lane_state.pos[0] == 0 ? 0 : lane_speed.pos[0]-1
        next_pos[1] = lane_state.pos[1]+1
        // probably not good to allow lane switch action if at 0 lane 
    } else {
        next_pos[0] = lane_state.pos[0]
        next_pos[1] = lane_state.pos[1]+action
    }
    
	obs = 1; // default observation
	if (rover_position == 0) {
		if (action == A_SAMPLE) {
			reward = rock_status ? 10 : -10;
			rock_status = 0;
		} else if (action == A_CHECK) {
			reward = 0;
			obs = rock_status;
		} else if (action == A_WEST) {
			reward = -100;
			rover_position = 2;
		} else {
			reward = 0;
			rover_position = 1;
		}
	} else if (rover_position == 1) {
		if (action == A_SAMPLE) {
			reward = -100;
			rover_position = 2;
		} else if (action == A_CHECK) {
			reward = 0;
			obs = (rand_num > 0.20) ? rock_status : (1 - rock_status);
		} else if (action == A_WEST) {
			reward = 0;
			rover_position = 0;
		} else {
			reward = 10;
			rover_position = 2;
		}
	} else {
		reward = 0;
	}

	return rover_position == 2;
}

/* ================================================
 * Functions related to beliefs and starting states
 * ================================================*/

double LaneModel::ObsProb(OBS_TYPE obs, const State& state,
	int action) const {
	if (action == A_CHECK) {
		const LaneState& simple_state = static_cast<const LaneState&>(state);
		int rover_position = simple_state.rover_position;
		int rock_status = simple_state.rock_status;

		if (rover_position == 0) {
			return obs == rock_status;
		} else if (rover_position == 1) {
			return (obs == rock_status) ? 0.8 : 0.2;
		}
	}

	return obs == 1;
}

State* LaneModel::CreateStartState(string type) const {
	return new LaneState(1, Random::RANDOM.NextInt(2));
}

Belief* LaneModel::InitialBelief(const State* start, string type) const {
	if (type == "DEFAULT" || type == "PARTICLE") {
		vector<State*> particles;

		LaneState* good_rock = static_cast<LaneState*>(Allocate(-1, 0.5));
		good_rock->rover_position = 1;
		good_rock->rock_status = 1;
		particles.push_back(good_rock);

		LaneState* bad_rock = static_cast<LaneState*>(Allocate(-1, 0.5));
		bad_rock->rover_position = 1;
		bad_rock->rock_status = 0;
		particles.push_back(bad_rock);

		return new ParticleBelief(particles, this);
	} else {
		cerr << "[LaneModel::InitialBelief] Unsupported belief type: " << type << endl;
		exit(1);
	}
}

/* ========================
 * Bound-related functions.
 * ========================*/

double LaneModel::GetMaxReward() const {
	return 10;
}

class LaneModelParticleUpperBound: public ParticleUpperBound {
protected:
	// upper_bounds_[pos][status]:
	//   max possible reward when rover_position = pos, and rock_status = status.
	vector<vector<double> > upper_bounds_;

public:
	LaneModelParticleUpperBound(const DSPOMDP* model) {
		upper_bounds_.resize(3);
		upper_bounds_[0].push_back(Discount(1) * 10);
		upper_bounds_[0].push_back(10 + Discount(2) * 10);
		upper_bounds_[1].push_back(10);
		upper_bounds_[1].push_back(Discount(1) * 10 + Discount(3) * 10);
		if (upper_bounds_[1][1] < 10)
			upper_bounds_[1][1] = 10;
		upper_bounds_[2].push_back(0);
		upper_bounds_[2].push_back(0);
	}

	double Value(const State& s) const {
		const LaneState& state = static_cast<const LaneState&>(s);
		return upper_bounds_[state.rover_position][state.rock_status];
	}
};

ScenarioUpperBound* LaneModel::CreateScenarioUpperBound(string name,
	string particle_bound_name) const {
	ScenarioUpperBound* bound = NULL;
	if (name == "TRIVIAL" || name == "DEFAULT") {
		bound = new TrivialParticleUpperBound(this);
	} else if (name == "MAX") {
		bound = new LaneModelParticleUpperBound(this);
	} else {
		cerr << "Unsupported base upper bound: " << name << endl;
		exit(0);
	}
	return bound;
}

ValuedAction LaneModel::GetMinRewardAction() const {
	return ValuedAction(A_EAST, 0);
}

class LaneModelEastPolicy: public Policy {
public:
	LaneModelEastPolicy(const DSPOMDP* model, ParticleLowerBound* bound) :
		Policy(model, bound) {
	}

	int Action(const vector<State*>& particles, RandomStreams& streams,
		History& history) const {
		return 1; // move east
	}
};

ScenarioLowerBound* LaneModel::CreateScenarioLowerBound(string name,
	string particle_bound_name) const {
	ScenarioLowerBound* bound = NULL;
	if (name == "TRIVIAL" || name == "DEFAULT") {
		bound = new TrivialParticleLowerBound(this);
	} else if (name == "EAST") {
		bound = new LaneModelEastPolicy(this,
			CreateParticleLowerBound(particle_bound_name));
	} else {
		cerr << "Unsupported lower bound algorithm: " << name << endl;
		exit(0);
	}
	return bound;
}

/* =================
 * Memory management
 * =================*/

State* LaneModel::Allocate(int state_id, double weight) const {
	LaneState* state = memory_pool_.Allocate();
	state->state_id = state_id;
	state->weight = weight;
	return state;
}

State* LaneModel::Copy(const State* particle) const {
	LaneState* state = memory_pool_.Allocate();
	*state = *static_cast<const LaneState*>(particle);
	state->SetAllocated();
	return state;
}

void LaneModel::Free(State* particle) const {
	memory_pool_.Free(static_cast<LaneState*>(particle));
}

int LaneModel::NumActiveParticles() const {
	return memory_pool_.num_allocated();
}

/* =======
 * Display
 * =======*/

void LaneModel::PrintState(const State& state, ostream& out) const {
	const LaneState& simple_state = static_cast<const LaneState&>(state);

	out << "Rover = " << simple_state.rover_position << "; Rock = "
		<< (simple_state.rock_status ? "GOOD" : "BAD") << endl;
}

void LaneModel::PrintObs(const State& state, OBS_TYPE observation,
	ostream& out) const {
	out << (observation ? "GOOD" : "BAD") << endl;
}

void LaneModel::PrintBelief(const Belief& belief, ostream& out) const {
	const vector<State*>& particles =
		static_cast<const ParticleBelief&>(belief).particles();

	double rock_status = 0;
	vector<double> pos_probs(3);
	for (int i = 0; i < particles.size(); i++) {
		State* particle = particles[i];
		const LaneState* state = static_cast<const LaneState*>(particle);
		rock_status += state->rock_status * particle->weight;
		pos_probs[state->rover_position] += particle->weight;
	}

	out << "Rock belief: " << rock_status << endl;

	out << "Position belief:" << " LEFT" << ":" << pos_probs[0] << " MIDDLE"
		<< ":" << pos_probs[1] << " RIGHT" << ":" << pos_probs[2] << endl;
}

void LaneModel::PrintAction(int action, ostream& out) const {
	if (action == A_SAMPLE)
		out << "Sample" << endl;
	if (action == A_CHECK)
		out << "Check" << endl;
	if (action == A_EAST)
		out << "EAST " << endl;
	if (action == A_WEST)
		out << "West" << endl;
}
