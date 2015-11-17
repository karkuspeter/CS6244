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
 * LaneBelief class
 * =============================================================================*/


LaneBelief::LaneBelief(vector< vector<State*> > particles, const DSPOMDP* model,
		Belief* prior = NULL, bool split = true):Belief(model){
	lanemodel_ = ((LaneModel*)model);
	int car_count = lanemodel_->car_count;

	DSPOMDP *new_model = model->MakeCopy();
	((LaneModel*)new_model)->car_count = 1;

	car_beliefs.resize(car_count);
	for (int i=0; i<car_count; i++){
		car_beliefs[i] = new ParticleBelief(particles[i], new_model, prior, split);
	}
}

LaneBelief::LaneBelief(vector<ParticleBelief* > car_beliefs_, const LaneModel* lanemod):Belief(lanemod){
	car_beliefs = car_beliefs_;
	lanemodel_ = lanemod;
}

void LaneBelief::Update(int action, OBS_TYPE obs) {
	vector<CarPos> observation = lanemodel_->DecodeObs(obs);
	history_.Add(action, obs);

	vector<CarPos> car_obs(1);
	for (int i=0; i<lanemodel_->car_count; i++){
		car_obs[0] = observation[i];
		car_beliefs[i]->Update(action, ((LaneModel*)(car_beliefs[i]->model_))->EncodeObs(car_obs));
	}

}

LaneBelief::~LaneBelief() {
	for (int i=0; i<lanemodel_->car_count; i++){
		car_beliefs[i]->~Belief();
	}
}

vector<State*> LaneBelief::Sample(int num) const {
	vector< vector<State*> > car_samples(lanemodel_->car_count);
	vector<State*> joint_samples(num);

	for (int i=0; i<lanemodel_->car_count; i++){
		car_samples[i] = car_beliefs[i]->Sample(num);
	}

	for (int i=0; i<num; i++){
		joint_samples[i] = model_->Copy(car_samples[0][i]);
		for (int j=1; j<lanemodel_->car_count; j++){
			((LaneState*)joint_samples[i])->cars[j] = (((LaneState*)car_samples[j][i])->cars[0]);
			assert(((LaneState*)car_samples[j][i])->speed == ((LaneState*)car_samples[0][i])->speed);
			assert(((LaneState*)car_samples[j][i])->lane == ((LaneState*)car_samples[0][i])->lane);
			assert((car_samples[j].size()) == car_samples[0].size());
		}
	}

	return joint_samples;
}

Belief* LaneBelief::MakeCopy() const {
	vector< ParticleBelief* > copy(lanemodel_->car_count);
	for (int i=0; i<lanemodel_->car_count; i++){
		copy[i] = (ParticleBelief*)(car_beliefs[i]->MakeCopy());
	}

	return new LaneBelief(copy, lanemodel_);
}

string LaneBelief::text() const {
	string cumm = "";
	for (int i=0; i<lanemodel_->car_count; i++){
		cumm += car_beliefs[i]->text();
	}
	return cumm;
}

/* =============================================================================
 * LaneState class
 * =============================================================================*/

LaneState::LaneState() {
	//cars.resize(4);
	speed = 0;
	lane = 0;
}

LaneState::LaneState(vector< CarPos > _cars, int _speed, int _lane){
    // Pos of our car and whole world
    /*cars.resize(4);
    for (int i=0; i<_cars.size(); i++){
    	cars[i] = _cars[i];
    }*/
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

//LaneModel::~LaneModel(){
//}

LaneModel::LaneModel() {
	//memory_pool_ = *(new MemoryPool<LaneState>());

    lane_count = 3;
    cell_count = 16;
    cell_offset = 8;
    max_cell = cell_count-cell_offset-1;

    speed_count = 4;
    car_count = 10;

    starting_lane = 5;
}

CarPos LaneModel::PickRandomCarPos(Random &random) const{
	CarPos pos;
	if (random.NextDouble() < P_DISABLE){
		pos.speed = 0;
		pos.cell = 0;
		pos.lane = 0;
	} else {
		pos.speed = random.NextInt(1, speed_count+1);
		pos.cell = random.NextInt(cell_offset-max_cell, cell_offset+max_cell+1);
		pos.lane = random.NextInt(0, lane_count);
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
	//vector< CarPos >& cars(car_count);
	//for (int i=0; i<car_count; i++)
	//	 cars[i] = lane_state.cars[i];
	Random random(rand_num);

	bool terminate = false;

	// move robot
    // TODO probably not good to allow lane switch action if at 0 lane
	// should disable invalid actions
	if (action == A_SWITCH) {
        lane_state.lane--;
        // shift all cars
        for (int i = 0; i < car_count; i++){
        	if (lane_state.cars[i].lane < lane_count -1 && lane_state.cars[i].cell != 0)
        		lane_state.cars[i].lane++;
        	else {
        		lane_state.cars[i] = PickRandomCarPos(random);
        		lane_state.cars[i].lane = 0;
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
	vector<CarPos> cars_prev = lane_state.cars;

	//evolve car positions
	for (int i = 0; i < car_count; i++){

		if (lane_state.cars[i].cell > 0){
			// car is on field
			rand_num = random.NextDouble();
			decision = 0;
			if (rand_num < SPEED_ACC){
				if (lane_state.cars[i].speed < speed_count)
					decision = 1;
			}
			else if (rand_num < SPEED_ACC + SPEED_DEC){
				if (lane_state.cars[i].speed > 1)
					decision = -1;
			}

			//cars try to avoid collision with us, when not switching lanes
			if (lane_state.cars[i].lane == lane_count-1 && action != A_SWITCH)
				if ( (lane_state.cars[i].cell == cell_offset-1)
						|| (lane_state.cars[i].cell == cell_offset-2 && lane_state.cars[i].speed > 1)
						|| (lane_state.cars[i].cell == cell_offset-3 && lane_state.cars[i].speed > 2))
					decision = -1;

			lane_state.cars[i].speed = lane_state.cars[i].speed + decision;
			lane_state.cars[i].cell = lane_state.cars[i].cell + lane_state.cars[i].speed - lane_state.speed;
			//disable if out of range
			if (lane_state.cars[i].cell < 1 || lane_state.cars[i].cell > cell_count-1){
				lane_state.cars[i].cell = 0;
				lane_state.cars[i].lane = 0;
				lane_state.cars[i].speed = 0;
			}


		} else {
			// car is disabled
			rand_num = random.NextDouble();
			if (rand_num < P_ENABLE){
				lane_state.cars[i].speed = random.NextInt(1, speed_count+1);
				if (lane_state.cars[i].speed > lane_state.speed)
					lane_state.cars[i].cell = cell_offset - max_cell;
				else
					lane_state.cars[i].cell = cell_offset + max_cell;
				lane_state.cars[i].lane = random.NextInt(0, lane_count);
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
    	if (lane_state.cars[i].lane == lane_count-1){
    		if (lane_state.cars[i].cell == cell_offset ||
    				(lane_state.cars[i].cell <= cell_offset && cars_prev[i].cell > cell_offset) ||
					(lane_state.cars[i].cell >= cell_offset && cars_prev[i].cell < cell_offset) ){
    			assert(lane_state.cars[i].cell != 0);
    			reward = R_COLLISION;
    			terminate = true;
    		}
    	}
    }

    // get observation

    vector<CarPos> observation(car_count);
    for(int i=0; i<car_count; i++){
    	observation[i] = lane_state.cars[i];
    	rand_num = random.NextDouble();
    	if (rand_num >= OBS_WRONG || observation[i].cell == 0)
    		continue;
    	if ((rand_num < OBS_WRONG / 2.0 && observation[i].cell>1) || observation[i].cell == cell_count-1){
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
	int lane_bits = ceil(log2(lane_count));
	int cell_bits = ceil(log2(cell_count));

	for (int i=0; i<car_count; i++){
		raw = raw << (lane_bits + cell_bits);
		raw |= (observation[i].lane << cell_bits);
		raw |= observation[i].cell;
	}
	return raw;
}


vector<CarPos> LaneModel::DecodeObs(OBS_TYPE raw) const{
	vector<CarPos> observation(car_count);
	int lane_bits = ceil(log2(lane_count));
	int cell_bits = ceil(log2(cell_count));

	for (int i=car_count-1; i>=0; i--){
		observation[i].speed = 0;
		observation[i].cell = raw & ((int)pow(2,cell_bits)-1);
		observation[i].lane = (raw >> cell_bits) & ((int)pow(2,lane_bits)-1);
		raw = raw >> (cell_bits + lane_bits);
	}
	return observation;
}

/* ================================================
 * Functions related to beliefs and starting states
 * ================================================*/

double LaneModel::ObsProb(OBS_TYPE obs, const State& state,	int action) const {
	double prob = 1;
	vector<CarPos> cars(car_count);
	for (int i=0; i<car_count; i++){
		cars[i] = static_cast<const LaneState&>(state).cars[i];
	}
	vector<CarPos> observation = DecodeObs(obs);

	for (int i =0; i<car_count; i++){
		if (cars[i].lane != observation[i].lane)
			return 0;
		if (cars[i].cell == observation[i].cell){
			if (cars[i].cell != 0)
				prob *= OBS_CORR;
		} else if (abs(cars[i].cell - observation[i].cell) == 1){
			if (cars[i].cell != 0 && observation[i].cell != 0)
				prob *= OBS_WRONG;
			else
				return 0;
		} else
			return 0; //0.000001;
	}

	return prob;
}

State* LaneModel::CreateStartState(string type) const {
	vector<CarPos> init_cars(6), cars(car_count);

	init_cars[0] = {0, 8, 2};
	init_cars[1] = {1, 3, 2};
	init_cars[2] = {0, 0, 0};
	init_cars[3] = {0, 0, 0};
	init_cars[4] = {0, 0, 0};
	init_cars[5] = {0, 0, 0};
	for (int i=0; i<car_count; i++)
		cars[0] = init_cars[0];

	LaneState startState(cars, 2, starting_lane);

	if (type == "T1"){
		LaneState* startStatePtr = memory_pool_.Allocate();
		startStatePtr->lane = startState.lane;
		startStatePtr->speed = startState.speed;
		for(int i=0; i<car_count; i++){
			startStatePtr->cars[i] = startState.cars[i];
		}
		return startStatePtr;

	} else if (type == "RANDOM" || type == "DEFAULT"){
		return RandomState(&startState);
	} else {
		cerr << "[LaneModel::CreateStartState] Unsupported start state type: " << type << endl;
		exit(1);
	}
}

State* LaneModel::RandomState(const LaneState* start) const {
	vector<CarPos> cars(car_count);
	for (int i=0; i<car_count; i++){
		cars[i] = PickRandomCarPos(Random::RANDOM);
		if(cars[i].lane ==  lane_count-1){
			if(cars[i].cell == cell_offset ||
					(cell_offset > cars[i].cell && cell_offset-cars[i].cell < cars[i].speed -start->speed) ||
					(cell_offset < cars[i].cell && cars[i].cell-cell_offset < start->speed - cars[i].speed)){
				i--;
			}
		}
	}

	LaneState* startState = memory_pool_.Allocate();
	startState->lane = start->lane;
	startState->speed = start->speed;
	startState->cars = cars;
	/*for (int i=0; i<car_count; i++){
		startState->cars[i] = cars[i];
	}*/

	return startState;
}

Belief* LaneModel::InitialBelief(const State* start, string type) const {
	if (type == "DEFAULT" || type == "PARTICLE") {
		int N = Globals::config.num_scenarios;
		vector< vector<State*> > particles(car_count);

		for (int i = 0; i < car_count; i++) {
			particles[i].resize(N);
			for (int j = 0; j < N; j++) {
				particles[i][j] = RandomState(static_cast<const LaneState*>(start));
				particles[i][j]->weight = 1.0 / N;
			}
		}

		if (car_count == 1){
			return new ParticleBelief(particles[0], this);
		} else {
			return new LaneBelief(particles, this);
		}

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
void LaneModel::PrintCars(const vector< CarPos > cars, ostream& out, int myspeed) const {
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
	/*CarPos cars[4];
	for (int i=0; i<car_count; i++){
		cars[i] = obscars[i];
	}*/

	out << endl;
	PrintCars(obscars, out, lane_state.speed);
}

void LaneModel::CollectBelief(vector<double> &speed_prob, vector<double> &lane_prob, vector< vector< vector<double> > > &probs) const{

}

void LaneModel::PrintBelief(const Belief& belief, ostream& out) const {
	/*
	if (car_count != 1){
		//out << "Joint belief, cant print" << endl;
		const LaneBelief& lanebelief = static_cast<const LaneBelief&>(belief);
		for (int i=0; i<car_count; i++){
			lanebelief.car_beliefs[i]->model_->PrintBelief(*(lanebelief.car_beliefs[i]), out);
		}
		return;
	}
	*/
	vector<double> speed_prob(speed_count+1);
	vector<double> lane_prob(starting_lane+1);

	vector< vector< vector<double> > > probs(lane_count); //lane, cell, speed
	for (int i=0; i<lane_count; i++){
		probs[i].resize(cell_count);
		for (int j=0; j<cell_count; j++){
			probs[i][j].resize(speed_count+1);
		}
	}

	int particles_sum = 0;
	if (car_count == 1){
		const vector<State*>& particles =
			static_cast<const ParticleBelief&>(belief).particles();

		for (int i = 0; i < particles.size(); i++) {
			State* particle = particles[i];
			const LaneState* state = static_cast<const LaneState*>(particle);

			speed_prob[state->speed] += particle->weight;
			lane_prob[state->lane] += particle->weight;
			for (int j=0; j<car_count; j++){
				probs[state->cars[j].lane][state->cars[j].cell][state->cars[j].speed] += particle->weight;
			}
		}
	} else {
		for (int j=0; j<car_count; j++){
			const vector<State*>& particles =
				static_cast<const LaneBelief&>(belief).car_beliefs[j]->particles();

			for (int i = 0; i < particles.size(); i++) {
				State* particle = particles[i];
				const LaneState* state = static_cast<const LaneState*>(particle);

				speed_prob[state->speed] += particle->weight;
				lane_prob[state->lane] += particle->weight;
				probs[state->cars[0].lane][state->cars[0].cell][state->cars[0].speed] += particle->weight;
			}
		}
	}

	out << "Lane  ";
	for (int i =0; i<starting_lane+1; i++)
		out << lane_prob[i] << " ";
	out << "Speed ";
	for (int i =0; i<speed_count+1; i++)
		out << speed_prob[i] << " ";
	out << "Particles " << particles_sum << endl;

	char s[5];
	double all_sum = 0;

	out << 	"(" << probs[0][0][0] << ")" << probs[1][0][0] << endl;
	for (int i=lane_count-1; i>=0; i--){
		out << "Lane " << i << endl;
		vector<double> cummulated(cell_count);
		for (int k=0; k<speed_count+1; k++){
			out << "SPEED " << k << " ";
			for (int j=0; j<cell_count; j++){
				sprintf(s, "%.3f", probs[i][j][k]);
				out << "|" << s;
				cummulated[j] += probs[i][j][k];
				all_sum += probs[i][j][k];
			}
			out << "|" << endl;
		}
		out << "        ";
		for (int j=0; j<cell_count; j++){
			sprintf(s, "%.3f", cummulated[j]);
			out << "|" << s;
		}
		out << "|" << endl;

	}

	out << "Sum " << all_sum << endl;

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

void LaneModel::UnitTest() {
	vector<CarPos> cars(car_count);
	double prob;

	return;

	cars[0] = {0, 8, 2};
	cars[1] = {1, 3, 2};
	cars[2] = {1, 1, 2};
	cars[3] = {0, 11, 2};

	LaneState* startState = memory_pool_.Allocate();
	startState->lane = starting_lane;
	startState->speed = 2;
	//startState->cars = cars;

	cout << "Unit test" << endl;

	PrintState(*startState, cout);
	Belief* belief = InitialBelief(startState, "DEFAULT");

	PrintBelief(*belief, cout);
	//OBS_TYPE xxx = EncodeObs(startState->cars);
	//belief->Update(A_KEEP, xxx);

	cout << "few simulations" << endl;
	for (int i=0; i<5; i++){
		double rew;
		OBS_TYPE obs;

		Step(*startState, 0.5, A_KEEP, rew, obs);
		PrintState(*startState, cout);
		PrintObs(*startState, obs, cout);
	}

	PrintBelief(*belief, cout);

	cout << EncodeObs(cars) << "=" << EncodeObs(DecodeObs(EncodeObs(cars))) << endl;
	for (int i=0; i<car_count; i++){
		if (cars[i].cell != DecodeObs(EncodeObs(cars))[i].cell)
			cout << "Coding Error !";
	}

	prob = ObsProb(EncodeObs(cars), *startState, A_ACC);
	cout << prob << "=" << (OBS_CORR*OBS_CORR) << endl;

	cars[1] = {1, 2, 2};
	prob = ObsProb(EncodeObs(cars), *startState, A_ACC);
	cout << prob << "=" << OBS_CORR*OBS_WRONG << endl;

	cars[1] = {1, 1, 2};
	prob = ObsProb(EncodeObs(cars), *startState, A_ACC);
	cout << prob << "=" << 0 << endl;

	cars[0] = {0, 8, 2};
	cars[1] = {1, 3, 2};
	cars[2] = {1, 1, 3};
	cars[3] = {1, 8, 1};
	//startState->cars = cars;
	prob = ObsProb(EncodeObs(cars), *startState, A_ACC);
	cout << prob << "=" << OBS_CORR*OBS_CORR*OBS_CORR*OBS_CORR << endl;

	cars[3] = {0, 0, 0};
	prob = ObsProb(EncodeObs(cars), *startState, A_ACC);
	cout << prob << "=" << 0 << endl;

	cars[3] = {0, 8, 0};
	prob = ObsProb(EncodeObs(cars), *startState, A_ACC);
	cout << prob << "=" << 0 << endl;

	cars[3] = {1, 7, 0};
	prob = ObsProb(EncodeObs(cars), *startState, A_ACC);
	cout << prob << "=" << OBS_WRONG*OBS_CORR*OBS_CORR*OBS_CORR << endl;



	memory_pool_.Free(startState);
}
