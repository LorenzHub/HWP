#ifndef LABYRINTH_H
#define LABYRINTH_H

#include "position.h"
#include <packetTypes.h>

typedef struct{
    uint8_t  x;
    uint8_t y;
    uint8_t cardinalDirection;
}LabyrinthPose_t;

void exploreMaze();

void setLabyrinthPose(Pose_t pose);

#endif // LABYRINTH_H