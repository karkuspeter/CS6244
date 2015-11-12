/*
 * lane_state.h
 *
 *  Created on: 12 Nov 2015
 *      Author: karkus
 */

#ifndef SRC_LANE_STATE_H_
#define SRC_LANE_STATE_H_

typedef struct {int l; int c; int s;} RobotPos;


/* =============================================================================
 * LaneState class
 * =============================================================================*/

class LaneState: public State {
public:
	vector< vector<int> > cells; // index by lane/cell
	RobotPos pos; // position of robot, [lane, cell]

	LaneState();
	LaneState(vector< vector<int> > _cells, RobotPos _pos);
	~LaneState();

	std::ostream& operator<<(std::ostream& out, const LaneState& f);


	string text() const;
};

typedef LaneState OBS_TYPE;



#endif /* SRC_LANE_STATE_H_ */
