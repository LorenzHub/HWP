#ifndef LABYRINTH_H
#define LABYRINTH_H

#include "position.h"
#include <communication/packetTypes.h>
#include <stdio.h>
#include <stdbool.h>
#include "packetTypes.h"

typedef struct{
    uint8_t  x;
    uint8_t y;
    Direction_t cardinalDirection; //0=NORTH,1=EAST,2=SOUTH,3=WEST
}LabyrinthPose_t;

typedef struct {
    bool north, south, east, west;  // isWall?
    uint8_t dirNorth, dirSouth, dirEast, dirWest; //0,1,2 sign for Algorithm of Tremaux
} Cell;

void exploreMaze();

void setLabyrinthPose(Pose_t pose);

void checkWalls(uint8_t* availableDirections);

bool isPlace();

Direction_t choosePlaceDirection();

Direction_t chooseWayDirection();

Direction_t getCardinalDirectionfromLookingDirection(Direction_t dirLooking);

static uint8_t* dirCountPtr(Cell *c, uint8_t dir);

void setNoWall(Direction_t cardinalDirection);

bool hasWall(Direction_t cardinalDirection);

Direction_t leastVisitedDirection();

void DriveDirection(Direction_t nextDirection);

#endif // LABYRINTH_H