#include "lane_model.h"

#define OBS_CORR 0.8
#define OBS_WRONG 0.2
#define SPEED_SAME 0.8
#define SPEED_CHANGE 0.2

using namespace std;

/* =============================================================================
 * LaneState class
 * =============================================================================*/

LaneState::LaneState() {
}

LaneState::LaneState(vector< vector<int> > _cells, RobotPos _pos){
    // Pos of our car and whole world
    cells = _cells;
    pos = _pos;
}

LaneState::~LaneState() {
}

string LaneState::text() const {
	return "robot position = " + to_string(pos[0]) + " " + to_string(pos[1]) + " " + to_string(pos[2]);
}

/* =============================================================================
 * LaneModel class
 * =============================================================================*/

LaneModel::LaneModel() {
    lane_count = LANE_COUNT;
    cell_count = CELL_COUNT;
    speed_count = SPEED_COUNT;
}

double LaneModel::CellObsProbability(const LaneState& state, int *pos, int speed_obs){
    // compute the probability that we observe [speed_obs] at cell [pos] given the current state [state]
    int val = state[pos[0]][pos[1]];
    double Pcorr = 0.8, Pfalse=0.2;
    if (val == 0 || speed_obs == 0) {
    	return double(val == speed_obs);
    } else if (val == 1){
		return (val == speed_obs ? 1 : 0 );
    }
}

double LaneModel::CellTransProbability(const LaneState& state, int *pos, int speed_new){
    // compute the probability that the next cell value is [speed_new] at cell [pos] given the last state [state]
    return 0;
}

int LaneModel::SampleCellObs(const LaneState& state, int *pos, double rand_num){
    // sample from the possible observations at cell [pos] given the current state [state]
    // use CellObsProbability
	vector<double> probs;
	double prob, normalizer = 0, sum = 0;

	for (int i = 0; i < speed_count; i++){
		prob = CellObsProbability(state, pos, i);
		probs.push_back(prob);
		normalizer += prob;
	}
	for (int i = 0; i < speed_count; i++){
		sum += probs[i]/normalizer;
		if (rand_num < sum){
			return i;
		}
	}

    assert(0);
    return -1;
}

int LaneModel::SampleCellVal(const LaneState& state, int *pos, double rand_num){
    // sample from the possible speed values at cell [pos] given the last state [state]
    // use CellTransProbability
	vector<double> probs;
	double prob, normalizer = 0, sum = 0;

	for (int i = 0; i < speed_count; i++){
		prob = CellTransProbability(state, pos, i);
		probs.push_back(prob);
		normalizer += prob;
	}
	for (int i = 0; i < speed_count; i++){
		sum += probs[i]/normalizer;
		if (rand_num < sum){
			return i;
		}
	}

    assert(0);
    return -1;
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

bool LaneModel::Step(State& state, double rand_num, int action,	double& reward, OBS_TYPE& obs) const {
	
	LaneState& lane_state = static_cast<LaneState&>(state);
	vector< vector<int> >& cells = lane_state.cells;
	Random random(rand_num);

	int new_c, speed, new_speed;
	bool terminate = false;

	//evolve state, from top
	for (int c = cell_count-1; c >= 0; c--){
		for (int l = 0; l < lane_count; l++){
			if ((speed = cells[l][c]) > 0) {
				cells[l][c] = 0;

				// stochastic change of speed
				rand_num = random.NextDouble();
				if (rand_num < SPEED_SAME)
					new_speed = speed;
				else if(speed == 1 || (speed < speed_count-1 && rand_num < SPEED_SAME + SPEED_CHANGE/2))
					new_speed = speed + 1;
				else
					new_speed = speed -1;

				// determine the new position of this car
				new_c = c + new_speed;

				// skip if out of field
				if (new_c > cell_count -1)
					continue;

				// avoid cars overlapping
				while (cells[l][new_c] > 0){
					new_c--;
					new_speed--;
				}
				assert(new_speed>0);

				cells[l][new_c] = new_speed;
			}
		}
	}

	// move robot
    // TODO probably not good to allow lane switch action if at 0 lane
	// should disable invalid actions
	if (action == A_SWITCH) {
        lane_state.pos.l = (lane_state.pos.l == 0 ? 0 : lane_state.pos.l-1);
    } else if (action == A_ACC){
    	if (lane_state.pos.s < speed_count - 1)
    		lane_state.pos.s++;
    } else if (action == A_DEC){
    	if (lane_state.pos.s > 1)
    		lane_state.pos.s--;
    }
    lane_state.pos.c = lane_state.pos.c+lane_state.pos.s;

    // compute reward
    reward = -1;
    if (cells[lane_state.pos.l][lane_state.pos.c] > 0){
    	// collision
    	reward = -500;
    	terminate = true;
    } else if (lane_state.pos.l == 0) {
    	reward = 100;
    	terminate = true;
    } else if (lane_state.pos.l > 0 && lane_state.pos.c >= cell_count - 1) {
    	reward = -100;
    	terminate = true;
    }

    // get observation
    obs = lane_state;  //deep copy?

	for (int c = cell_count-1; c >= 0; c--){
		for (int l = 0; l < lane_count; l++){
			if ((obs.cells[l][c]) > 0) {
				// stochastic observation
				rand_num = random.NextDouble();
				if (rand_num > OBS_CORR)
					if(obs.cells[l][c] == 1 && (obs.cells[l][c] < speed_count-1 || rand_num < OBS_CORR + OBS_WRONG/2))
						obs.cells[l][c]++;
					else
						obs.cells[l][c]--;
			}
		}
	}

	return terminate;
}

/* ================================================
 * Functions related to beliefs and starting states
 * ================================================*/

double LaneModel::ObsProb(OBS_TYPE obs, const State& state,
	int action) const {
	/*if (action == A_CHECK) {
		const LaneState& simple_state = static_cast<const LaneState&>(state);
		int rover_position = simple_state.rover_position;
		int rock_status = simple_state.rock_status;

		if (rover_position == 0) {
			return obs == rock_status;
		} else if (rover_position == 1) {
			return (obs == rock_status) ? 0.8 : 0.2;
		}
	}
	*/
	return obs == 1;
}

State* LaneModel::CreateStartState(string type) const {
	//return new LaneState(1, Random::RANDOM.NextInt(2));
	return new LaneState();
}

Belief* LaneModel::InitialBelief(const State* start, string type) const {
	if (type == "DEFAULT" || type == "PARTICLE") {
		vector<State*> particles;

		LaneState* good_rock = static_cast<LaneState*>(Allocate(-1, 0.5));
		/*good_rock->rover_position = 1;
		good_rock->rock_status = 1;
		particles.push_back(good_rock);

		LaneState* bad_rock = static_cast<LaneState*>(Allocate(-1, 0.5));
		bad_rock->rover_position = 1;
		bad_rock->rock_status = 0;
		particles.push_back(bad_rock);
		*/
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
		return 10;
		//return upper_bounds_[state.rover_position][state.rock_status];
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
	return ValuedAction(2, 0);
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

	out << "Rover = " << simple_state.pos[0] << " " << simple_state.pos[1];
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
		//rock_status += state->rock_status * particle->weight;
		//pos_probs[state->rover_position] += particle->weight;
	}

	out << "Rock belief: " << rock_status << endl;

	out << "Position belief:" << " LEFT" << ":" << pos_probs[0] << " MIDDLE"
		<< ":" << pos_probs[1] << " RIGHT" << ":" << pos_probs[2] << endl;
}

void LaneModel::PrintAction(int action, ostream& out) const {
	if (action == 0)
		out << "Switch" << endl;
	else
		out << "Go " << action << endl;

}
