#include "labyrinth.h"
#include <math.h>
#include <communication/communication.h>
#include <inttypes.h>
#include "io/adc/adc.h"
#include <time.h>

/*exploreMaze <-> Statemachine*/
//where call setLabyrinthPose()

//implement DriveDirection!

LabyrinthPose_t labyrinthPose = {4,4,1};
Cell maze[7][7]; 
static uint8_t fromDirection = 4; //0=NORTH,1=EAST,2=SOUTH,3=WEST,4=initial
static Direction_t nextDirection = DIRECTION_NORTH;

void exploreMaze() {

    //Initialization
    static uint8_t initialized = 0;
    if(!initialized){
        srand(time(NULL));  // intialize random generator
        for(uint8_t i=0;i<7;i++){
            for(uint8_t j=0;j<7;j++){
                maze[i][j].north=true; maze[i][j].south=true; maze[i][j].east=true; maze[i][j].west=true;
                maze[i][j].dirNorth=0; maze[i][j].dirSouth=0; maze[i][j].dirEast=0; maze[i][j].dirWest=0;
            }
        }
        initialized=1;
    }

    if(isPlace()){
        choosePlaceDirection();
    }

    else{
        chooseWayDirection();
    }

    fromDirection = nextDirection;

    DriveDirection(nextDirection);
}

void checkWalls(uint8_t* availableDirections) {
    *availableDirections = 0;
    if(ADC_getFilteredValue(2) < 250) { //front
        *availableDirections += 1;    
        setNoWall(getCardinalDirectionfromLookingDirection(DIRECTION_NORTH));
    }
    if(ADC_getFilteredValue(0) < 250) { //right front
        *availableDirections += 1;
        setNoWall(getCardinalDirectionfromLookingDirection(DIRECTION_EAST));
    }
    if(ADC_getFilteredValue(3) < 250) { //left front
        *availableDirections += 1;
        setNoWall(getCardinalDirectionfromLookingDirection(DIRECTION_WEST));
    }
}

bool isPlace(){
    uint8_t availableDirectionsCounter;
    checkWalls(&availableDirectionsCounter);
    return (availableDirectionsCounter >= 2);
}

Direction_t choosePlaceDirection(){
    //Increment fromDirection counter
    *dirCountPtr(&maze[labyrinthPose.x][labyrinthPose.y], fromDirection) += 1;

    //Place is unknown    
    if (maze[labyrinthPose.x][labyrinthPose.y].dirNorth == 0 && maze[labyrinthPose.x][labyrinthPose.y].dirSouth == 0 &&
        maze[labyrinthPose.x][labyrinthPose.y].dirEast == 0 && maze[labyrinthPose.x][labyrinthPose.y].dirWest == 0){
        //choose random direction, but not backwards
        do{
            nextDirection = (Direction_t)(rand() % 4);
        }while(fromDirection == nextDirection || hasWall(nextDirection));
        *dirCountPtr(&maze[labyrinthPose.x][labyrinthPose.y], nextDirection) += 1; //increment chosen direction counter
        return nextDirection;
    }
    
    //Place is known but fromDir is unknown
    else if (*dirCountPtr(&maze[labyrinthPose.x][labyrinthPose.y], fromDirection) == 0){
        //turn around
        *dirCountPtr(&maze[labyrinthPose.x][labyrinthPose.y], (Direction_t)((fromDirection + 2) % 4)) += 1; //increment chosen direction counter
        return nextDirection = (Direction_t)((fromDirection + 2) % 4);
    }

    else {
        //choose least visited direction, but not backwards
        *dirCountPtr(&maze[labyrinthPose.x][labyrinthPose.y], leastVisitedDirection()) += 1; //increment chosen direction counter
        return leastVisitedDirection();
    }
}

Direction_t chooseWayDirection(){
    Direction_t front = getCardinalDirectionfromLookingDirection(DIRECTION_NORTH);
    Direction_t left = getCardinalDirectionfromLookingDirection(DIRECTION_EAST);
    Direction_t right = getCardinalDirectionfromLookingDirection(DIRECTION_WEST);

    if(hasWall(front) && hasWall(left) && hasWall(right)){ //Dead End
        //turn around
        return (Direction_t)((fromDirection + 2) % 4); 
    }
    else{
        return leastVisitedDirection();
    }
}

static uint8_t* dirCountPtr(Cell *c, uint8_t dir) {
    if (c == NULL) return NULL;
    switch (dir) {
        case 0: return &c->dirNorth; /* NORTH */
        case 1: return &c->dirEast;  /* EAST  */
        case 2: return &c->dirSouth; /* SOUTH */
        case 3: return &c->dirWest;  /* WEST  */
        default: return NULL;        /* should not happen */
    }
}

Direction_t getCardinalDirectionfromLookingDirection(Direction_t dirLooking) {
    Direction_t dirCardinal;
    switch (labyrinthPose.cardinalDirection) {
        case DIRECTION_NORTH:
            dirCardinal = dirLooking;
            break;
        case DIRECTION_EAST:
            dirCardinal = (Direction_t)((dirLooking + 1) % 4);
            break;
        case DIRECTION_SOUTH:
            dirCardinal = (Direction_t)((dirLooking + 2) % 4);
            break;
        case DIRECTION_WEST:
            dirCardinal = (Direction_t)((dirLooking + 3) % 4);
            break;
        default:
            dirCardinal = dirLooking; // should not happen
            break;
    }
    return dirCardinal;
}

void setNoWall(Direction_t cardinalDirection){
    switch(cardinalDirection){
        case DIRECTION_NORTH:
            maze[labyrinthPose.x][labyrinthPose.y].north = false;
            break;
        case DIRECTION_EAST:
            maze[labyrinthPose.x][labyrinthPose.y].east = false;
            break;
        case DIRECTION_SOUTH:
            maze[labyrinthPose.x][labyrinthPose.y].south = false;
            break;
        case DIRECTION_WEST:
            maze[labyrinthPose.x][labyrinthPose.y].west = false;
            break;
        default:
            break;
    }
}

bool hasWall(Direction_t cardinalDirection){
    switch(cardinalDirection){
        case DIRECTION_NORTH:
            return maze[labyrinthPose.x][labyrinthPose.y].north;
        case DIRECTION_EAST:
            return maze[labyrinthPose.x][labyrinthPose.y].east;
        case DIRECTION_SOUTH:
            return maze[labyrinthPose.x][labyrinthPose.y].south;
        case DIRECTION_WEST:
            return maze[labyrinthPose.x][labyrinthPose.y].west;
        default:
            return true;
    }
}

/* Returns the least visited direction, which is not fromDirection */
Direction_t leastVisitedDirection(){
        uint8_t min = 3; //max 
        if(fromDirection != DIRECTION_NORTH && !hasWall(DIRECTION_NORTH)){
            nextDirection = DIRECTION_NORTH;
            min = maze[labyrinthPose.x][labyrinthPose.y].dirNorth;
        }
        if(maze[labyrinthPose.x][labyrinthPose.y].dirEast <= min &&
            fromDirection != DIRECTION_EAST && !hasWall(DIRECTION_EAST)){
            nextDirection = DIRECTION_EAST;
            min = maze[labyrinthPose.x][labyrinthPose.y].dirEast;
        }
        if(maze[labyrinthPose.x][labyrinthPose.y].dirSouth <= min &&
                fromDirection != DIRECTION_SOUTH && !hasWall(DIRECTION_SOUTH)){
            nextDirection = DIRECTION_SOUTH;
            min = maze[labyrinthPose.x][labyrinthPose.y].dirSouth;
        }
        if(maze[labyrinthPose.x][labyrinthPose.y].dirWest <= min &&
            fromDirection != DIRECTION_WEST && !hasWall(DIRECTION_WEST)){
            nextDirection = DIRECTION_WEST;
        }
        return nextDirection;
}

void DriveDirection(Direction_t nextDirection){
    int8_t diff = (int8_t)nextDirection - (int8_t)labyrinthPose.cardinalDirection;
    diff = (int8_t)((diff + 4) % 4);   // diff in 0..3
    if (diff == 3) diff = -1;          // 3 means -1 (left)
    // diff == 0,1,2,-1
    switch (diff) {
        case 0:
            // forwards
            break;
        case 1:
            // turn right 90°
            break;
        case -1:
            // turn left 90°
            break;
        case 2:
            // U‑Turn 180°
            break;
    }
}

void setLabyrinthPose(Pose_t pose) {
    labyrinthPose.x = (uint8_t)(pose.x / 256.9f)+4.0f; //Cell size 256.9mm with wall
    labyrinthPose.y = (uint8_t)(pose.y / 256.9f)+4.0f;

    float t = pose.theta + M_PI_4; //range
	t = fmodf(t, 2.0f * M_PI);
	if (t < 0.0f)
		t += 2.0f * M_PI;

	int idx = (int) floorf(t / (M_PI_2));
	const Direction_t map[4] = { DIRECTION_EAST,DIRECTION_NORTH, DIRECTION_WEST, DIRECTION_SOUTH };
	labyrinthPose.cardinalDirection = map[idx & 0x3];
}