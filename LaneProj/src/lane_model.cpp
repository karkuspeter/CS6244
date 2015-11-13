#include "lane_model.h"

#define OBS_CORR 0.8
#define OBS_WRONG 0.2

#define SPEED_KEEP 0.8
#define SPEED_ACC 0.1
#define SPEED_DEC 0.1

#define P_ENABLE 0.3

#define P_DISABLE 0.2

#define R_COLLISION -500
#define R_STEP -1
#define R_SUCCESS 100

using namespace std;

/* =============================================================================
 * LaneState class
 * =============================================================================*/

LaneState::LaneState() {
}

LaneState::LaneState(vector< CarPos > _cars, int _speed, int _lane){
    // Pos of our car and whole world
    cars = _cars;
    speed = _speed;
    lane = _lane;
}

LaneState::~LaneState() {
}

string LaneState::text() const {
	return "lane " + to_string(lane) + " speed " + to_string(speed);
}

/* =============================================================================
 * LaneModel class
 * =============================================================================*/

LaneModel::LaneModel() {
    lane_count = 2;
    cell_count = 16;
    cell_offset = 8;
    max_cell = 7;

    speed_count = 3;
    car_count = 4;

    starting_lane = 5;
}

CarPos LaneModel::PickRandomCarPos(Random random) const{
	CarPos pos;
	pos.lane = 0;
	if (random.NextDouble() < P_DISABLE){
		pos.speed = 0;
		pos.cell = 0;
	} else {
		pos.speed = random.NextInt(1, speed_count);
		pos.cell = random.NextInt(cell_offset-max_cell, cell_offset+max_cell);
	}
	return pos;
}

/* ======
 * Action
 * ======*/

int LaneModel::NumActions() const {
	return 4;
}


/* ==============================
 * Deterministic simulative model
 * ==============================*/

bool LaneModel::Step(State& state, double rand_num, int action,	double& reward, OBS_TYPE& obs) const {
	
	LaneState& lane_state = static_cast<LaneState&>(state);
	vector< CarPos >& cars = lane_state.cars;
	Random random(rand_num);

	bool terminate = false;

	// move robot
    // TODO probably not good to allow lane switch action if at 0 lane
	// should disable invalid actions
	if (action == A_SWITCH) {
        lane_state.lane--;
        // shift all cars
        for (int i = 0; i < car_count; i++){
        	if (cars[i].lane < lane_count -1 && cars[i].cell != 0)
        		cars[i].lane++;
        	else {
        		cars[i] = PickRandomCarPos(random);
        		cars[i].lane = 0;
        	}
        }
    } else if (action == A_ACC){
    	if (lane_state.speed < speed_count)
    		lane_state.speed++;
    } else if (action == A_DEC){
    	if (lane_state.speed > 1)
    		lane_state.speed--;
    }

	int decision;
	//evolve car positions
	for (int i = 0; i < car_count; i++){

		if (cars[i].cell > 0){
			// car is on field
			rand_num = random.NextDouble();
			decision = 0;
			if (rand_num < SPEED_ACC)
				if (cars[i].speed < speed_count)
					decision = 1;
			else if (rand_num < SPEED_ACC + SPEED_DEC)
				if (cars[i].speed > 1)
					decision = -1;

			//cars try to avoid collision with us, when not switching lanes
			if (cars[i].lane == 1 && action != A_SWITCH)
				if ( (cars[i].cell == cell_offset-1)
						|| (cars[i].cell == cell_offset-2 && cars[i].speed > 1)
						|| (cars[i].cell == cell_offset-3 && cars[i].speed > 2))
					decision = -1;

			cars[i].speed = cars[i].speed + decision;
			cars[i].cell = cars[i].cell + cars[i].speed - lane_state.speed;
			//disable if out of range
			if (cars[i].cell < 1 || cars[i].cell > cell_count-1){
				cars[i].cell = 0;
				cars[i].lane = 0;
				cars[i].speed = 0;
			}


		} else {
			// car is disabled
			rand_num = random.NextDouble();
			if (rand_num < P_ENABLE){
				cars[i].speed = random.NextInt(1, speed_count);
				if (cars[i].speed > lane_state.speed)
					cars[i].cell = cell_offset - max_cell;
				else
					cars[i].cell = cell_offset + max_cell;
				cars[i].lane = random.NextInt(0, 1);
			}
		}
	}

    // compute reward
    reward = R_STEP;
    if (lane_state.lane == 0){
    	reward = R_SUCCESS;
    	terminate = true;
    }
    // check if colliding with other car
    // TODO this allows to jump over a speed 1 car with speed 3
    for (int i = 0; i < car_count; i++){
    	if (cars[i].lane == 1 && cars[i].cell == cell_offset){
    		reward = R_COLLISION;
    		terminate = true;
    	}
    }

    // get observation
    vector<CarPos> observation = cars;
    for(int i=0; i<car_count; i++){
    	rand_num = random.NextDouble();
    	if (rand_num >= OBS_WRONG)
    		continue;
    	if ((rand_num < OBS_WRONG / 2 && observation[i].cell>1) || observation[i].cell == cell_count-1){
    		observation[i].cell--;
    	} else {
    		observation[i].cell++;
    	}
    }

    obs = EncodeObs(observation);

	return terminate;
}

OBS_TYPE LaneModel::EncodeObs(vector<CarPos> observation) const{
	OBS_TYPE raw = 0;
	for (int i=0; i<car_count; i++){
		raw |= (observation[i].lane == 1 ? 0b10000 : 0b00000);
		raw |= observation[i].cell;
		assert(observation[i].cell < 0b10000);
		raw = raw << 5;
	}
	return raw;
}


vector<CarPos> LaneModel::DecodeObs(OBS_TYPE raw) const{
	vector<CarPos> observation(car_count);

	for (int i=car_count-1; i>=0; i--){
		observation[i].speed = 0;
		observation[i].cell = raw & 0b01111;
		observation[i].lane = (raw & 0b10000 ? 1 : 0);
		raw = raw >> 5;
	}
	return observation;
}

/* ================================================
 * Functions related to beliefs and starting states
 * ================================================*/

double LaneModel::ObsProb(OBS_TYPE obs, const State& state,	int action) const {
	double prob = 1;
	vector<CarPos> cars = static_cast<const LaneState&>(state).cars;
	vector<CarPos> observation = DecodeObs(obs);

	for (int i =0; i<car_count; i++){
		if (cars[i].lane != observation[i].lane)
			return 0;
		if (cars[i].cell == observation[i].cell)
			prob *= OBS_CORR;
		else if (abs(cars[i].cell - observation[i].cell) == 1)
			prob *= OBS_WRONG;
		else
			return 0;
	}

	return prob;
}

State* LaneModel::CreateStartState(string type) const {
	if (type == "T1" || type == "DEFAULT"){
		vector<CarPos> cars(car_count);

		cars[0] = {0, 8, 2};
		cars[1] = {1, 3, 2};
		cars[2] = {0, 0, 0};
		cars[3] = {0, 0, 0};

		LaneState* startState = memory_pool_.Allocate();
		startState->lane = starting_lane;
		startState->speed = 2;
		startState->cars = cars;

		return startState;

	} else if (type == "RANDOM"){
		return RandomState();
	} else {
		cerr << "[LaneModel::CreateStartState] Unsupported start state type: " << type << endl;
		exit(1);
	}
}

State* LaneModel::RandomState() const {
	vector<CarPos> cars(car_count);
	for (int i=0; i<car_count; i++){
		cars[i] = PickRandomCarPos(Random::RANDOM);
	}

	LaneState* startState = memory_pool_.Allocate();
	startState->lane = starting_lane;
	startState->speed = 2;
	startState->cars = cars;

	return startState;
}

Belief* LaneModel::InitialBelief(const State* start, string type) const {
	if (type == "DEFAULT" || type == "PARTICLE") {
		int N = 1000;
		vector<State*> particles(N);

		for (int i = 0; i < N; i++) {
			particles[i] = CreateStartState();
			particles[i]->weight = 1.0 / N;
		}

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
	return R_SUCCESS;
}

ValuedAction LaneModel::GetMinRewardAction() const {
	return ValuedAction(A_DEC, R_STEP);
}

class LaneModelParticleUpperBound: public ParticleUpperBound {
protected:
	// upper_bounds_[pos][status]:
	//   max possible reward when rover_position = pos, and rock_status = status.
	vector<vector<double> > upper_bounds_;

public:
	LaneModelParticleUpperBound(const DSPOMDP* model) {
	}

	double Value(const State& s) const {
		const LaneState& state = static_cast<const LaneState&>(s);
		int steps_to_go = state.lane-1;
		double disc = Discount(steps_to_go);

		// best possibility that we switch lanes, collecting sum(R_STEP*gamma^i) reward
		// and then succeeding that gives R_SUCCES*gamma^i reward
		return disc*R_SUCCESS + R_STEP*(1-disc)/(1-Discount());
	}
};

ScenarioUpperBound* LaneModel::CreateScenarioUpperBound(string name,
	string particle_bound_name) const {
	ScenarioUpperBound* bound = NULL;
	if (name == "TRIVIAL") {
		bound = new TrivialParticleUpperBound(this);
	} else if (name == "MAX" || name == "DEFAULT") {
		bound = new LaneModelParticleUpperBound(this);
	} else {
		cerr << "Unsupported base upper bound: " << name << endl;
		exit(0);
	}
	return bound;
}

class LaneModelSwitchPolicy: public Policy {
public:
	LaneModelSwitchPolicy(const DSPOMDP* model, ParticleLowerBound* bound) :
		Policy(model, bound) {
	}

	int Action(const vector<State*>& particles, RandomStreams& streams,
		History& history) const {
		return LaneModel::A_SWITCH; // switch
	}
};


ScenarioLowerBound* LaneModel::CreateScenarioLowerBound(string name,
	string particle_bound_name) const {
	ScenarioLowerBound* bound = NULL;
	if (name == "TRIVIAL" || name == "DEFAULT") {
		bound = new TrivialParticleLowerBound(this);
	} else if (name == "SWITCH") {
		bound = new LaneModelSwitchPolicy(this,
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
void LaneModel::PrintCars(const vector<CarPos>& cars, ostream& out, int myspeed) const {
	vector< vector<int> > lanes(lane_count);
	for (int i =0; i<lane_count; i++){
		lanes[i].resize(cell_count);
		for (int j=1; j<cell_count; j++)
			lanes[i][j] = -1;
	}

	for (int i = 0; i < car_count; i++){
		if(cars[i].cell == 0) lanes[cars[i].lane][0]++;
		else lanes[cars[i].lane][cars[i].cell] = cars[i].speed;
	}

	out << "ME   " << "(" << lanes[0][0] << ")" << lanes[1][0];
	for (int i = 1+2; i < cell_offset; i++)
		out << "  ";
	out << " " << myspeed << endl;

	for (int j = lane_count-1; j >= 0; j--){
		out << "LANE" << j;
		for (int i = 1; i < cell_count; i++){
			out << ((lanes[j][i] > -1) ? "|"+to_string(lanes[j][i]) : "| ");
		}
		out << "|" << endl;
	}


}

void LaneModel::PrintState(const State& state, ostream& out) const {
	const LaneState& lane_state = static_cast<const LaneState&>(state);

	out << "Lane " << lane_state.lane << " Speed " << lane_state.speed << endl;
	PrintCars(lane_state.cars, out, lane_state.speed);
}

void LaneModel::PrintObs(const State& state, OBS_TYPE observation,
	ostream& out) const {
	const LaneState& lane_state = static_cast<const LaneState&>(state);
	vector<CarPos> obscars = DecodeObs(observation);

	out << endl;
	PrintCars(obscars, out, lane_state.speed);
}

void LaneModel::PrintBelief(const Belief& belief, ostream& out) const {
	/*const vector<State*>& particles =
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
		<< ":" << pos_probs[1] << " RIGHT" << ":" << pos_probs[2] << endl;*/
	out << "Printing Belief" << endl;
}

void LaneModel::PrintAction(int action, ostream& out) const {
	if (action == A_SWITCH)
		out << "Switch" << endl;
	else if (action == A_KEEP)
		out << "Keep" << endl;
	else if (action == A_ACC)
		out << "Accelerate" << endl;
	else if (action == A_DEC)
		out << "Decelerate" << endl;
	else
		assert(false);
}
