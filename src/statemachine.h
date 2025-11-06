#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_

void stateMachine();

typedef enum {
    IDLE,
    Drive_Forward,
    Drive_Forward_5sec
} state;

static state currentState = IDLE;

void setState(state newState);

void drive_Forward_5sec();

#endif /* STATEMACHINE_H_ */