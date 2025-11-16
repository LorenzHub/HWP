#include "labyrinth.h"
#include <math.h>

/*exploreMaze <-> Statemachine*/

LabyrinthPose_t labyrinthPose = {4,4,1};

void exploreMaze(void) {

}

void setLabyrinthPose(Pose_t pose) {
    labyrinthPose.x = (uint8_t)(pose.x / 256.9f)+4.0f; //Cell size 256.9mm with wall
    labyrinthPose.y = (uint8_t)(pose.y / 256.9f)+4.0f;
    labyrinthPose.cardinalDirection = pose.theta;

    float t = pose.theta + M_PI_4; //?
	t = fmodf(t, 2.0f * M_PI);
	if (t < 0.0f)
		t += 2.0f * M_PI;

	int idx = (int) floorf(t / (M_PI_2));
	const Direction_t map[4] = { DIRECTION_NORTH, DIRECTION_EAST, DIRECTION_SOUTH, DIRECTION_WEST };
	labyrinthPose.cardinalDirection = map[idx & 0x3];
}