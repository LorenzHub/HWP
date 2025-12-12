#include "statemachine.h"
#include "calibration.h"
#include "encoder.h"
#include <motor/motor.h>
#include <tools/timeTask/timeTask.h>
#include <communication/communication.h>
#include <inttypes.h>
#include "calcPathCommand.h"
#include "labyrinth.h"
#include "position.h"
#include "io/adc/adc.h"
#include "ir_sensors.h"
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>
#include <math.h>



/* Define the current state here (single-definition) */
state currentState = IDLE;

// Statische Variable für Ziel-Distanz in mm
static uint16_t targetDistance_mm = 100;  // Default: 100mm

// Statische Variable für Ziel-Ticks
static int16_t targetTicks = 1000;  // Default: 1000 Ticks

// Wall-Following Konfigurationsvariablen
static int16_t wall_detection_threshold_mm_config = 45;
static int16_t wall_correction_threshold_mm_config = 50;
static int16_t wall_distance_gain_config = 30;
static int16_t wall_parallel_gain_config = 50;
static int16_t center_target_distance_mm_config = 60;

// Wandkorrektur Ein/Aus (1 = aktiviert, 0 = deaktiviert)
static uint8_t wall_correction_enabled = 0;  // Default: DEAKTIVIERT

// Statische Variable für PWM-Wert des rechten Motors
static int16_t targetPWM = 3000;  // Default: 3000 PWM

// Statische Variable für Ziel-Winkel in Grad
static int16_t targetAngle_degrees = 90;  // Default: 90°
static uint8_t initialized_drive = 0;

state nextState;

// Vorwärtsdeklaration
void correctRotationMovement_Wait(void);

void stateMachine() {
    switch(currentState){
        case IDLE:
            Motor_stopAll();
            break;
        case Turn_On_Spot:
            /* rotate in-place */
            Motor_setPWM(3000, -3000);
            break;
        case Calibrate_Distance:
            calibration_run();
            break;
        case Drive_Forward_Distance:
            drive_Forward_distance_mm(targetDistance_mm, targetPWM);
            break;
        case Drive_Forward_Ticks:
            drive_Forward_ticks(targetTicks, targetPWM);
            break;
        case Turn_On_Spot_Degrees:
            turn_On_Spot_degrees(targetAngle_degrees, targetPWM);
            break;
        case ExploreMaze:
            mazeExplore();
            break;
        case drive_Forward_distance_then_explore:
            drive_Forward_distance_mm_then_explore(targetDistance_mm, targetPWM);
            break;
        case turn_On_Spot_degrees_then_drive:
            turn_degrees_then_drive(targetAngle_degrees, targetPWM);
            break;    
        case correctOrientation_done:
            doAfterCorrectOrientation(targetAngle_degrees, targetPWM);
            break;    
        case FollowThePath:
            break;
        case CorrectRotationMovement:
            correctRotationMovement();
            break;
        case CorrectRotationMovement_Wait:
            correctRotationMovement_Wait();
            break;
        case waitAndGetAprilTagPose:
            WaitAndGetAprilTagPose();
            break;    
        case waitForFirstAprilTagPose:
            nextState = ExploreMaze;
            WaitAndGetAprilTagPose();
            nextState = ExploreMaze;
            break;
    }
}

int AprilTagReceived = 0;

static timeTask_time_t waitStart;

static Aprilinitialized = 0;

int checkIfAprilUpdateAvailableInShortFuture(){
    if (Aprilinitialized == 0) return 0; //only after first april tag pose is received
    static uint8_t initialized = 0;
    
    if (!initialized) {
        timeTask_getTimestamp(&waitStart);
        initialized = 1;
    }
    
    timeTask_time_t now;
    timeTask_getTimestamp(&now);
    
    // Warte 23 Sekunde
    const uint32_t waitTime_us = 20000000UL;
    
    if (timeTask_getDuration(&waitStart, &now) >= waitTime_us) {
        initialized = 0;
        return 1;
    }
    return 0;
}

void WaitAndGetAprilTagPose(void){
    if (AprilTagReceived == 1) {
        communication_log(LEVEL_INFO, "AprilTagPose received!");
        position_calculatePoseDifference();
        timeTask_getTimestamp(&waitStart);
        setState(nextState);
        AprilTagReceived = 0;
        Aprilinitialized = 1;
    }
}

void setNextState(state s){
    nextState = s;
}

void mazeExplore(void) {
    // KEINE Odometrie mehr verwenden - Position wird nur durch manuelles Zählen aktualisiert
    exploreMaze();
}

void drive_Forward_distance_mm_then_explore(uint16_t distance_mm, int16_t pwmRight){
    initialized_drive = 1;
    
    // Starte Bewegung
    drive_Forward_distance_mm(distance_mm, pwmRight);
    //correctRotationMovement(); is done in drive_Forwar_distance_mm
// Wenn Bewegung normal abgeschlossen ist (IDLE)
if(currentState == IDLE) { //cant happen anymore
    // Manuelle Positionsaktualisierung nach vollständiger Vorwärtsbewegung
    //TODO Schluss
    //pdateLabyrinthPosition();
    setState(ExploreMaze);
    
}
// Wenn State bereits auf ExploreMaze gesetzt wurde (Wand erkannt in drive_Forward_ticks),
// dann wurde die Position bereits dort aktualisiert

    
}

void doAfterCorrectOrientation(int16_t angle_degrees, int16_t pwm){
    //communication_log(LEVEL_INFO, "initialized_drive in doAfterCorrectOrientation %" PRIu8 " ", initialized_drive);
    if(initialized_drive == 1){ //after drive forward is done
        //communication_log(LEVEL_INFO, "state Turn_On_Spot_degrees set");
        turn_On_Spot_degrees(angle_degrees, pwm);
    } 
    else if(initialized_drive == 2){ //after turn degrees is done
        //communication_log(LEVEL_INFO, "state drive_Forward_ticks set");
        turn_On_Spot_degrees(angle_degrees, pwm);
        if (currentState == CorrectRotationMovement_Wait){
            setState(correctOrientation_done);
            initialized_drive = 3;
        }
    }
    else if(initialized_drive == 3){
        drive_Forward_distance_mm(246, 4000);
    }
    if (currentState == CorrectRotationMovement_Wait) {
            //communication_log(LEVEL_INFO, "ExploreMaze set");
            //setState(ExploreMaze);
            initialized_drive = 1;
        }
}

void turn_degrees_then_drive(int16_t angle_degrees, int16_t pwm){
    initialized_drive = 2;
    turn_On_Spot_degrees(angle_degrees, pwm);
    //position_updateExpectedPose();

    if(currentState == IDLE){ //cant happen anymore
        setState(drive_Forward_distance_then_explore);
    }
    
}

void correctRotationMovement_Wait(void) {
    static uint8_t initialized = 0;
    static timeTask_time_t waitStart;
    
    if (!initialized) {
        timeTask_getTimestamp(&waitStart);
        initialized = 1;
        communication_log(LEVEL_INFO, "Waiting 1 seconds after correction turn for sensor stabilization...");
    }
    
    timeTask_time_t now;
    timeTask_getTimestamp(&now);
    
    // Warte 1 Sekunde
    const uint32_t waitTime_us = 1000000UL;
    
    if (timeTask_getDuration(&waitStart, &now) >= waitTime_us) {
        communication_log(LEVEL_INFO, "Wait completed, checking correction again...");
        initialized = 0;
        setState(CorrectRotationMovement);
    }
}

/**
 * Berechnet den Korrekturwinkel basierend auf der Sensor-Differenz
 * @param diff Differenz zwischen den Sensoren in mm
 * @param sensorDistance_mm Abstand zwischen den Sensoren in mm
 * @return Korrekturwinkel in Grad (als int16_t)
 */
static int16_t calculateCorrectionAngle(int16_t diff, float sensorDistance_mm) {
    // Berechne Winkel: atan(diff / sensorDistance) in Grad
    float angle_rad = atanf((float)diff / sensorDistance_mm);
    float angle_deg_float = angle_rad * 180.0f / 3.14159265359f * 0.8f;
    int16_t angle_deg = (int16_t)angle_deg_float;
    
    // Konvertiere Float zu Integer für Log-Ausgabe (AVR unterstützt kein Float-Format)
    int16_t angle_rad_mrad = (int16_t)(angle_rad * 1000.0f);  // Radiant in Milliradiant
    int16_t angle_deg_cdeg = (int16_t)(angle_deg_float * 100.0f);  // Grad in Zentigrad
    int16_t angle_deg_neg_cdeg = (int16_t)(-angle_deg_float * 100.0f);  // Negierter Winkel in Zentigrad
    communication_log(LEVEL_INFO, "Calculated correction angle: %d mrad = %d cdeg (int: %d deg, negated: %d cdeg)", 
                     angle_rad_mrad, angle_deg_cdeg, angle_deg, angle_deg_neg_cdeg);
    
    return angle_deg;
}

void correctRotationMovement(void) {
    communication_log(LEVEL_INFO, "TRYING Correcting rotation movement");
    communication_log(LEVEL_INFO, "RIGHT_FRONT: %d RIGHT_BACK: %d FRONT: %d LEFT_FRONT: %d LEFT_BACK: %d", 
        CalibrateIRSensors(0), CalibrateIRSensors(1), CalibrateIRSensors(2), CalibrateIRSensors(3), CalibrateIRSensors(4));
    // Hole Sensorwerte
    uint16_t rightFrontADC = ADC_getFilteredValue(0);  // IR1 - Right Front
    uint16_t leftFrontADC = ADC_getFilteredValue(3);   // IR4 - Left Front
    const int16_t correctionDistance_mm = 3;
    const int16_t correctionPWM = 4000;
    const float sensorDistance_mm = 90.0f; // Abstand zwischen den Sensoren in mm
    
    int16_t angle_deg;
    Pose_t* currentPose = position_getCurrentPose();
    
    if(CalibrateIRSensors(0) < 60) { // Rechte Wand in gutem Abstand
        int16_t diff = CalibrateIRSensors(1) - CalibrateIRSensors(0);
        
        if(diff < -correctionDistance_mm) {
            communication_log(LEVEL_INFO, "difference: %d - correcting right", diff);
            angle_deg = calculateCorrectionAngle(diff, sensorDistance_mm);
            statemachine_setTargetAngle(-angle_deg);
            statemachine_setTargetPWM(correctionPWM);
            setState(Turn_On_Spot_Degrees);
            currentPose->theta += (float)-angle_deg * 3.14f / 180.0f;
        }
        else if(diff > correctionDistance_mm) {
            communication_log(LEVEL_INFO, "difference: %d - correcting left", diff);
            angle_deg = calculateCorrectionAngle(diff, sensorDistance_mm);
            statemachine_setTargetAngle(-angle_deg);
            statemachine_setTargetPWM(correctionPWM);
            setState(Turn_On_Spot_Degrees);
            currentPose->theta += (float)-angle_deg * 3.14f / 180.0f;
        }
        else {
            communication_log(LEVEL_INFO, "no correction needed - difference: %d", diff);
            communication_log(LEVEL_INFO, "initialized_drive %" PRIu8 " ", initialized_drive);
            if(initialized_drive == 1){
                //updateLabyrinthPosition();
                setState(ExploreMaze);
                initialized_drive = 0;
            }
            else if(initialized_drive == 2){
                setState(drive_Forward_distance_then_explore);
                initialized_drive = 0;
            }
            else{
                setState(ExploreMaze);
                initialized_drive = 0;
            }
        }
    }
    else if(CalibrateIRSensors(3) < 60) { // Linke Wand in gutem Abstand
        int16_t diff = CalibrateIRSensors(4) - CalibrateIRSensors(3);
        
        if(diff < -correctionDistance_mm) {
            communication_log(LEVEL_INFO, "difference: %d - correcting left", diff);
            angle_deg = calculateCorrectionAngle(diff, sensorDistance_mm);
            statemachine_setTargetAngle(-angle_deg);
            statemachine_setTargetPWM(correctionPWM);
            setState(Turn_On_Spot_Degrees);
            currentPose->theta += (float)-angle_deg * 3.14f / 180.0f;
        }
        else if(diff > correctionDistance_mm) {
            communication_log(LEVEL_INFO, "difference: %d - correcting right", diff);
            angle_deg = calculateCorrectionAngle(diff, sensorDistance_mm);
            statemachine_setTargetAngle(-angle_deg);
            statemachine_setTargetPWM(correctionPWM);
            setState(Turn_On_Spot_Degrees);
            currentPose->theta += (float)-angle_deg * 3.14f / 180.0f;
        }
        else {
            communication_log(LEVEL_INFO, "no correction needed - difference: %d", diff);
            communication_log(LEVEL_INFO, "initialized_drive %" PRIu8 " ", initialized_drive);
            if(initialized_drive == 1){
                //updateLabyrinthPosition();
                setState(ExploreMaze);
                initialized_drive = 0;
            }
            else if(initialized_drive == 2){
                setState(drive_Forward_distance_then_explore);
                initialized_drive = 0;
            }
            else{
                setState(ExploreMaze);
                initialized_drive = 0;
            }
        }
    }
    else {
        // Keine Wand erkannt 
        communication_log(LEVEL_INFO, "no wall found for correction (R=%u, L=%u)", rightFrontADC, leftFrontADC);
        communication_log(LEVEL_INFO, "initialized_drive %" PRIu8 " ", initialized_drive);
        
        if(OdomErr() > 0.05f /*&&initialized_drive == 1*/ ){
            if(initialized_drive == 1){ //called after drive forward is done
                communication_log(LEVEL_INFO, "correctOrientation called after drive forward");
                correctOrientation();
            }
            else if(initialized_drive == 2){ //called after turn degrees is done
                communication_log(LEVEL_INFO, "correctOrientation called after turn degrees");
                correctOrientation();
            }
        }
        else{ 
             if(initialized_drive == 1){
                //updateLabyrinthPosition();
                setState(ExploreMaze);
                initialized_drive = 0;
            }
            else if(initialized_drive == 2){
                setState(drive_Forward_distance_then_explore);
                initialized_drive = 0;
            }
        }
    }
}

// Fahre eine bestimmte Anzahl von Encoder-Ticks vorwärts mit kontinuierlicher Korrektur
void drive_Forward_ticks(int32_t targetTicksValue, int16_t pwmRight) {
    #define communication_log(level, ...) ((void)0)
    // ==================== STATISCHE VARIABLEN ====================
    // Initialisierung und Zustand
    static uint8_t initialized = 0;
    static uint8_t phase = 0;  // 0 = Soft-Start, 1 = Fahren, 2 = Abbremsen
    
    // Encoder-Werte
    static int16_t startEncoderR = 0;
    static int16_t startEncoderL = 0;
    static int16_t lastEncoderR = 0;
    static int16_t lastEncoderL = 0;
    static int16_t lastEncoderR_acc = 0;  // Für Overflow-sichere Akkumulation
    static int16_t lastEncoderL_acc = 0;
    static int32_t accumulatedDeltaR = 0; // Akkumulierte Ticks (Overflow-sicher)
    static int32_t accumulatedDeltaL = 0;
    
    // PWM-Werte
    static int16_t targetPWMLeft = 0;
    static int16_t targetPWMRight = 0;
    static int16_t currentSoftStartPWM = 0;
    static int16_t lastPWMLeft = 0;
    static int16_t lastPWMRight = 0;
    
    // Timing
    static timeTask_time_t startOfDrive;
    static timeTask_time_t softStartTime;
    static timeTask_time_t controlLoopTime;
    static timeTask_time_t lastLogTime;
    
    // PI-Regler
    static int32_t integralError = 0;
    static int16_t speedFilterR[3] = {0, 0, 0};
    static int16_t speedFilterL[3] = {0, 0, 0};
    static uint8_t filterIndex = 0;
    static uint8_t stallCounter = 0;
    
    // ==================== KONSTANTEN ====================
    // Soft-Start Parameter
    const int16_t softStartMinPWM = 1000;       // Start-PWM für Soft-Start
    const int16_t softStartStep = 200;          // PWM-Schrittweite für Soft-Start
    const uint32_t softStartInterval = 50000UL; // 50ms zwischen Soft-Start-Schritten
    
    // OPTIMIERTE PI-Regler Parameter für bessere Performance bei Asymmetrien
    const int16_t Kp = 100;                     // ERHÖHT: Proportional-Verstärkung (+100% für stärkere Korrektur)
    const int16_t Ki = 10;                      // AKTIVIERT: Integral-Verstärkung für Langzeit-Korrektur
    const int32_t maxIntegral = 3000;           // REDUZIERT: Anti-Windup Grenze für stabileren I-Anteil
    const int16_t deadZone = 1;                 // REDUZIERT: Totzone für sensitivere Regelung
    const int16_t maxAdjustment = 1000;         // ERHÖHT: Maximale PWM-Korrektur (+100% für stärkere Asymmetrien)
    const int16_t maxSlewRate = 150;            // ERHÖHT: Maximale PWM-Änderung pro Zyklus (+50%)
    const int16_t positionCorrectionThreshold = 3; // REDUZIERT: Schwelle für frühere Positions-Korrektur

    // Timing Parameter
    const uint32_t controlLoopInterval = 20000UL; // 20ms Regelzyklus
    // const uint32_t logInterval = 100000UL;        // 100ms Log-Intervall - TEMPORÄR DEAKTIVIERT
    const int16_t minSpeedTicks = 2;              // REDUZIERT: Mindest-Ticks für frühere Regelung
    
    // Abbremsen Parameter
    const int16_t rampDownThreshold = 100;      // Ticks vor Ziel: Abbremsen beginnen
    
    // ==================== WAND-ERKENNUNG ====================
    if (currentState == drive_Forward_distance_then_explore || currentState == correctOrientation_done) {
        if (ADC_getFilteredValue(2) > 730) {
            Motor_stopAll();
            #undef communication_log
            communication_log(LEVEL_INFO, "Front wall detected - stopping movement");
            communication_log(LEVEL_INFO, "CorrectRotationMovement_Wait state in driveForward_ticks set");
            #define communication_log(level, ...) ((void)0)
            setState(CorrectRotationMovement_Wait);
            initialized = 0;
            return;
        }
    }
    
    // ==================== INITIALISIERUNG ====================
    if (!initialized) {
        timeTask_getTimestamp(&startOfDrive);
        
        // Encoder-Startwerte speichern
        startEncoderR = encoder_getCountR();
        startEncoderL = encoder_getCountL();
        lastEncoderR = startEncoderR;
        lastEncoderL = startEncoderL;
        // Akkumulierte Deltas für Overflow-sichere Messung initialisieren
        lastEncoderR_acc = startEncoderR;
        lastEncoderL_acc = startEncoderL;
        accumulatedDeltaR = 0;
        accumulatedDeltaL = 0;
        communication_log(LEVEL_INFO, "Start-Encoder: R=%" PRId16 " L=%" PRId16, startEncoderR, startEncoderL);
        
        // PWM-Werte setzen
        targetPWMRight = pwmRight;
        targetPWMLeft = calibration_getPWMLeft(pwmRight);
        
        // Sicherheitsprüfung
        if (targetPWMLeft <= 0) {
            communication_log(LEVEL_WARNING, "PWM Left ungültig (%" PRId16 "), verwende %" PRId16, targetPWMLeft, pwmRight);
            targetPWMLeft = pwmRight;
        }
        if (targetPWMRight <= 0) {
            communication_log(LEVEL_WARNING, "PWM Right ungültig (%" PRId16 "), verwende 3000", targetPWMRight);
            targetPWMRight = 3000;
        }
        
        // Soft-Start beginnen
        currentSoftStartPWM = softStartMinPWM;
        Motor_setPWM(currentSoftStartPWM, currentSoftStartPWM);
        timeTask_getTimestamp(&softStartTime);
        timeTask_getTimestamp(&controlLoopTime);
        timeTask_getTimestamp(&lastLogTime);
        
        // Reset PI-Regler
        integralError = 0;
        filterIndex = 0;
        stallCounter = 0;
        lastPWMLeft = 0;
        lastPWMRight = 0;
        for (uint8_t i = 0; i < 3; i++) {
            speedFilterR[i] = 0;
            speedFilterL[i] = 0;
        }
        
        phase = 0;
        initialized = 1;
        communication_log(LEVEL_INFO, "Fahre %" PRId32 " Ticks, Ziel-PWM L=%" PRId16 " R=%" PRId16, 
                         targetTicksValue, targetPWMLeft, targetPWMRight);
    }
    
    // ==================== AKTUELLE WERTE LESEN ====================
    timeTask_time_t now;
    timeTask_getTimestamp(&now);
    
    int16_t currentEncoderR = encoder_getCountR();
    int16_t currentEncoderL = encoder_getCountL();
    
    // Overflow-sichere Delta-Berechnung durch Akkumulation kleiner Inkremente
    // Die Subtraktion von int16_t-Werten behandelt Overflow automatisch korrekt (2er-Komplement)
    int16_t incR = currentEncoderR - lastEncoderR_acc;
    int16_t incL = currentEncoderL - lastEncoderL_acc;
    accumulatedDeltaR += incR;
    accumulatedDeltaL += incL;
    lastEncoderR_acc = currentEncoderR;
    lastEncoderL_acc = currentEncoderL;
    
    int32_t deltaR = accumulatedDeltaR;
    int32_t deltaL = accumulatedDeltaL;
    int32_t avgDelta = (deltaL + deltaR) / 2;
    int32_t remainingTicks = targetTicksValue - avgDelta;
    int16_t positionDiff = (int16_t)(deltaR - deltaL);  // Positions-Differenz zwischen Rädern
    
    // ==================== ZIEL ERREICHT? ====================
    if (avgDelta >= targetTicksValue) {
        Motor_stopAll();
        
        int32_t encoderDiff = deltaR - deltaL;
        int32_t absEncoderDiff = (encoderDiff > 0) ? encoderDiff : -encoderDiff;
        uint16_t actualDistance_mm = (uint16_t)(avgDelta * 688 / 10000);
        uint32_t duration_ms = timeTask_getDuration(&startOfDrive, &now) / 1000;
        
        communication_log(LEVEL_INFO, "=== Fahrt abgeschlossen ===");
        communication_log(LEVEL_INFO, "Ziel: %" PRId32 " Ticks, Erreicht: %" PRId32 " Ticks", targetTicksValue, avgDelta);
        communication_log(LEVEL_INFO, "Encoder L=%" PRId32 " R=%" PRId32 " Diff=%" PRId32, deltaL, deltaR, encoderDiff);
        communication_log(LEVEL_INFO, "Distanz: ~%u mm, Zeit: %lu ms", actualDistance_mm, (unsigned long)duration_ms);
        
        if (absEncoderDiff <= 2) {
            communication_log(LEVEL_INFO, "Kalibrierung: EXZELLENT");
        } else if (absEncoderDiff <= 5) {
            communication_log(LEVEL_INFO, "Kalibrierung: GUT");
        } else if (absEncoderDiff <= 10) {
            communication_log(LEVEL_INFO, "Kalibrierung: AKZEPTABEL");
        } else {
            communication_log(LEVEL_WARNING, "Kalibrierung: SCHLECHT - Neu kalibrieren!");
        }
        communication_log(LEVEL_INFO, "===========================");
        
        //setState(IDLE);
        #undef communication_log
        communication_log(LEVEL_INFO, "CorrectRotationMovement_Wait state in driveForward_ticks set");
        #define communication_log(level, ...) ((void)0)
        setState(CorrectRotationMovement_Wait);
        initialized = 0;
        return;
    }
    
    // ==================== PHASE 0: SOFT-START ====================
    if (phase == 0) {
        if (timeTask_getDuration(&softStartTime, &now) >= softStartInterval) {
            currentSoftStartPWM += softStartStep;
            
            int16_t minTargetPWM = (targetPWMLeft < targetPWMRight) ? targetPWMLeft : targetPWMRight;
            if (currentSoftStartPWM >= minTargetPWM) {
                // Soft-Start abgeschlossen -> Phase 1
                Motor_setPWM(targetPWMLeft, targetPWMRight);
                lastEncoderR = currentEncoderR;
                lastEncoderL = currentEncoderL;
                timeTask_getTimestamp(&controlLoopTime);
                phase = 1;
                communication_log(LEVEL_INFO, "Soft-Start fertig, fahre mit PWM L=%" PRId16 " R=%" PRId16, 
                                 targetPWMLeft, targetPWMRight);
            } else {
                Motor_setPWM(currentSoftStartPWM, currentSoftStartPWM);
                timeTask_getTimestamp(&softStartTime);
            }
        }
        return;
    }
    
    // ==================== PHASE 1 & 2: FAHREN MIT PI-REGLER ====================
    if (timeTask_getDuration(&controlLoopTime, &now) < controlLoopInterval) {
        return;  // Noch nicht Zeit für nächsten Regelzyklus
    }
    
    // Prüfe ob Abbremsen beginnen soll
    if (phase == 1 && remainingTicks <= (int32_t)rampDownThreshold) {
        phase = 2;
        communication_log(LEVEL_INFO, "Beginne Abbremsen, verbleibend: %" PRId32 " Ticks", remainingTicks);
    }
    
    // Geschwindigkeit berechnen
    int16_t speedR = currentEncoderR - lastEncoderR;
    int16_t speedL = currentEncoderL - lastEncoderL;
    
    // Gleitender Durchschnitt (3er Filter)
    speedFilterR[filterIndex] = speedR;
    speedFilterL[filterIndex] = speedL;
    filterIndex = (filterIndex + 1) % 3;
    
    int16_t filteredSpeedR = (speedFilterR[0] + speedFilterR[1] + speedFilterR[2]) / 3;
    int16_t filteredSpeedL = (speedFilterL[0] + speedFilterL[1] + speedFilterL[2]) / 3;
    int16_t absSpeedR = (filteredSpeedR > 0) ? filteredSpeedR : -filteredSpeedR;
    int16_t absSpeedL = (filteredSpeedL > 0) ? filteredSpeedL : -filteredSpeedL;
    
    // Stall-Erkennung
    if (absSpeedR < minSpeedTicks && absSpeedL < minSpeedTicks) {
        stallCounter++;
        if (stallCounter > 10) {
            communication_log(LEVEL_WARNING, "Stall erkannt! Speed L=%" PRId16 " R=%" PRId16, filteredSpeedL, filteredSpeedR);
            stallCounter = 0;
        }
    } else {
        stallCounter = 0;
    }
    
    // PI-Regler berechnen
    int16_t u = 0;
    if (absSpeedR >= minSpeedTicks || absSpeedL >= minSpeedTicks) {
        int16_t speedDiff = filteredSpeedR - filteredSpeedL;
        int16_t absSpeedDiff = (speedDiff > 0) ? speedDiff : -speedDiff;
        int16_t absPositionDiff = (positionDiff > 0) ? positionDiff : -positionDiff;
        
        if (absSpeedDiff > deadZone || absPositionDiff > deadZone) {
            // P-Anteil
            int16_t pTerm = (speedDiff * Kp) / 100;
            
            // I-Anteil mit Anti-Windup
            integralError += speedDiff;
            if (integralError > maxIntegral) integralError = maxIntegral;
            if (integralError < -maxIntegral) integralError = -maxIntegral;
            int16_t iTerm = (integralError * Ki) / 100;
            
            // Positions-Korrektur
            int16_t posTerm = 0;
            if (absPositionDiff > positionCorrectionThreshold) {
                posTerm = (positionDiff * Kp) / 50;  // VERSTÄRKT: 4x stärkere Positions-Korrektur
            }
            
            u = pTerm + iTerm + posTerm;
            
            // Begrenzung
            if (u > maxAdjustment) u = maxAdjustment;
            if (u < -maxAdjustment) u = -maxAdjustment;
        } else {
            // Innerhalb Deadzone - Integral langsam abbauen (5% pro Zyklus)
            integralError = (integralError * 95) / 100;
        }
    }
    
    // Basis-PWM bestimmen (mit Ramp-Down in Phase 2)
    int16_t basePWMLeft = targetPWMLeft;
    int16_t basePWMRight = targetPWMRight;
    
    if (phase == 2) {
        // Ramp-Down: PWM proportional zur verbleibenden Distanz reduzieren
        int16_t rampFactor = (int16_t)((remainingTicks * 100) / rampDownThreshold);
        if (rampFactor < 30) rampFactor = 30;   // Min. 30% PWM
        if (rampFactor > 100) rampFactor = 100;
        basePWMLeft = (targetPWMLeft * rampFactor) / 100;
        basePWMRight = (targetPWMRight * rampFactor) / 100;
    }
    
    // Symmetrische PWM-Anpassung
    // Wenn speedR > speedL (speedDiff > 0, u > 0): rechts bremsen, links beschleunigen
    int16_t pwmLeft = basePWMLeft + u;
    int16_t pwmRight_new = basePWMRight - u;
    
    // Slew-Rate Limit
    if (lastPWMLeft != 0) {
        int16_t dL = pwmLeft - lastPWMLeft;
        int16_t dR = pwmRight_new - lastPWMRight;
        if (dL > maxSlewRate) pwmLeft = lastPWMLeft + maxSlewRate;
        if (dL < -maxSlewRate) pwmLeft = lastPWMLeft - maxSlewRate;
        if (dR > maxSlewRate) pwmRight_new = lastPWMRight + maxSlewRate;
        if (dR < -maxSlewRate) pwmRight_new = lastPWMRight - maxSlewRate;
    }
    lastPWMLeft = pwmLeft;
    lastPWMRight = pwmRight_new;
    
    // Absolute Grenzen
    if (pwmLeft < 1000) pwmLeft = 1000;
    if (pwmLeft > 8191) pwmLeft = 8191;
    if (pwmRight_new < 1000) pwmRight_new = 1000;
    if (pwmRight_new > 8191) pwmRight_new = 8191;
    
    /*
    //TODO: Fall, falls keine/nur eine Wand vorhanden

    //Offset 
    if(CalibrateIRSensors(3) > (CalibrateIRSensors(0)+3)){ // left front bigger than right front
        pwmLeft = (int16_t) ((float)pwmLeft * 1.07f);
    } 
    else if(CalibrateIRSensors(0) > (CalibrateIRSensors(3)+3)){ // right front bigger than left front
        pwmLeft = (int16_t) ((float)pwmLeft * 0.97f);
    }
    */

    pwmLeft = (int16_t) ((float)pwmLeft * 1.03f);

    Motor_setPWM(pwmLeft, pwmRight_new);
    
    // Logging (10 Hz) - TEMPORÄR DEAKTIVIERT
    /*
    if (timeTask_getDuration(&lastLogTime, &now) >= logInterval) {
        int16_t speedDiff = filteredSpeedR - filteredSpeedL;
        communication_log(LEVEL_INFO, "P%" PRIu8 ": sL=%" PRId16 " sR=%" PRId16 " rem=%" PRId32 " u=%" PRId16 " I=%" PRId32 " pL=%" PRId16 " pR=%" PRId16,
                         phase, filteredSpeedL, filteredSpeedR, remainingTicks, u, integralError, pwmLeft, pwmRight_new);
        // Debug: Encoder-Werte und Deltas loggen (nur alle 1 Sekunde, um Spam zu vermeiden)
        static uint8_t debugLogCounter = 0;
        if (++debugLogCounter >= 10) {  // Alle 1 Sekunde (10 * 100ms)
            communication_log(LEVEL_INFO, "DEBUG: Enc R=%" PRId16 " L=%" PRId16 " startR=%" PRId16 " startL=%" PRId16 " deltaR=%" PRId32 " deltaL=%" PRId32 " avgDelta=%" PRId32,
                           currentEncoderR, currentEncoderL, startEncoderR, startEncoderL, deltaR, deltaL, avgDelta);
            debugLogCounter = 0;
        }
        timeTask_getTimestamp(&lastLogTime);
    }
    */
    
    // Werte für nächsten Zyklus speichern
    lastEncoderR = currentEncoderR;
    lastEncoderL = currentEncoderL;
    timeTask_getTimestamp(&controlLoopTime);

    #undef communication_log
}

void setState(state newState) {
    currentState = newState;
}

// Setze Ziel-Distanz für Drive_Forward_Distance
void statemachine_setTargetDistance(uint16_t distance_mm) {
    targetDistance_mm = distance_mm;
}

// Setze Ziel-Ticks für Drive_Forward_Ticks
void statemachine_setTargetTicks(int16_t ticks) {
    targetTicks = ticks;
}

// Setze PWM-Wert für rechten Motor
void statemachine_setTargetPWM(int16_t pwm) {
    targetPWM = pwm;
}

// Fahre eine bestimmte Distanz in mm vorwärts (Wrapper-Funktion)
// Rechnet mm in Ticks um und ruft drive_Forward_ticks auf
void drive_Forward_distance_mm(uint16_t distance_mm, int16_t pwmRight) {
    // 1 Encoder-Tick = 0.0688 mm (2048 Ticks = 1 Rad-Umdrehung)
    // Umrechnung: distance_mm / 0.0688 mm = targetTicks
    // 1/0.0688 ≈ 14.5349, verwende 14535/1000 für bessere Genauigkeit
    int16_t targetTicks = (int16_t)((int32_t)distance_mm * 14535 / 1000);  // distance_mm / 0.0688 mm ≈ targetTicks
    
    // Rufe die tick-basierte Funktion mit dem übergebenen PWM-Wert für rechten Motor auf
    drive_Forward_ticks(targetTicks, pwmRight);
    if (currentState == CorrectRotationMovement_Wait) {
        updateLabyrinthPosition();
    }
}

// Setze Ziel-Winkel für Drehung auf der Stelle
void statemachine_setTargetAngle(int16_t angle_degrees) {
    targetAngle_degrees = angle_degrees;
}

/*Drehe den Roboter um einen bestimmten Winkel auf der Stelle
  angle_degrees: Winkel in Grad (positiv = rechts drehen, negativ = links drehen)
  pwm: PWM-Wert für beide Motoren (absolut)*/
void turn_On_Spot_degrees(int16_t angle_degrees, int16_t pwm) {
    /* logs for this function suppressed */
    #define communication_log(level, ...) ((void)0)
    static uint8_t initialized = 0;
    static int16_t startEncoderR = 0;
    static int16_t startEncoderL = 0;
    static int16_t targetTicksValue = 0;
    static int16_t pwmLeft = 0;
    static int16_t pwmRight = 0;
    
    static uint8_t phase = 0;  // 0 = Soft-Start, 1 = Drehen
    static int16_t currentSoftStartPWM = 0;
    static timeTask_time_t softStartTime;
    static timeTask_time_t waitStart;
    
    // Konstanten
    const int16_t softStartMinPWM = 1000;  // Start-PWM für Soft-Start
    const int16_t softStartStep = 200;     // PWM-Schrittweite für Soft-Start
    const uint32_t softStartInterval = 50000UL; // 50ms zwischen Soft-Start-Schritten
    const uint32_t baseSpeedWaitTime = 200000UL; // 200ms Basis-Wartezeit nach Soft-Start
    const uint32_t timePerPWM = 50UL; // 0.05ms pro PWM-Einheit (50µs)
    const int16_t minPWMForCalc = 3000; // Referenz-PWM für Berechnung
    
    // Physikalische Konstanten
    const float wheelbase_mm = 166.5f;  // Radabstand in mm
    const float mmPerTick = 0.0688f;    // mm pro Tick
    const float pi = 3.14159265359f;
    
    if (!initialized) {
        // Setze Start-Encoder-Werte BEVOR Motoren starten
        startEncoderR = encoder_getCountR();
        startEncoderL = encoder_getCountL();
        
        // Berechne benötigte Ticks für den Winkel
        // Umfang der Drehbewegung = π × Radabstand
        // Distanz pro Rad = (π × Radabstand × Winkel) / 360°
        int16_t absAngle= (angle_degrees > 0) ? angle_degrees : -angle_degrees;
        float distancePerWheel_mm = (pi * wheelbase_mm * (float)absAngle ) / 360.0f;
        targetTicksValue = (int16_t)(distancePerWheel_mm / mmPerTick);
        
        // Bestimme Drehrichtung: positiv = links drehen (L vorwärts, R rückwärts)
        int16_t absPWM = (pwm > 0) ? pwm : -pwm;
        if (angle_degrees > 0) {
            // Links drehen: L vorwärts (+), R rückwärts (-)
            pwmLeft = absPWM;
            pwmRight = -absPWM;
        } else {
            // Rechts drehen: L rückwärts (-), R vorwärts (+)
            pwmLeft = -absPWM;
            pwmRight = absPWM;
        }
        
        // Sicherheitsprüfung
        if (absPWM <= 0) {
            communication_log(LEVEL_WARNING, "PWM ist 0 oder negativ (%" PRId16 "), verwende 3000", pwm);
            absPWM = 3000;
            if (angle_degrees > 0) {
                pwmLeft = 3000;
                pwmRight = -3000;
            } else {
                pwmLeft = -3000;
                pwmRight = 3000;
            }
        }
        
        // Starte mit Soft-Start
        currentSoftStartPWM = softStartMinPWM;
        Motor_setPWM(pwmLeft, pwmRight);
        timeTask_getTimestamp(&softStartTime);
        timeTask_getTimestamp(&waitStart);
        phase = 0;  // Starte mit Soft-Start
        initialized = 1;
        communication_log(LEVEL_INFO, "Drehe um %" PRId16 "° auf der Stelle, benötigt %" PRId16 " Ticks pro Rad, PWM=%" PRId16 "...", 
                         angle_degrees, targetTicksValue, absPWM);
    }
    
    if (phase == 0) {
        // Phase 0: Soft-Start - PWM schrittweise hochfahren
        timeTask_time_t now;
        timeTask_getTimestamp(&now);
        
        // Prüfe ob Ziel bereits erreicht (auch während Soft-Start)
        int16_t currentEncoderR = encoder_getCountR();
        int16_t currentEncoderL = encoder_getCountL();
        int16_t deltaR = currentEncoderR - startEncoderR;
        int16_t deltaL = currentEncoderL - startEncoderL;
        // Für Drehung: Beide Deltas sollten gleich groß sein (aber unterschiedliche Vorzeichen)
        int16_t absDeltaR = (deltaR > 0) ? deltaR : -deltaR;
        int16_t absDeltaL = (deltaL > 0) ? deltaL : -deltaL;
        int16_t maxDelta = (absDeltaR > absDeltaL) ? absDeltaR : absDeltaL;
        
        if (maxDelta >= targetTicksValue) {
            // Ziel bereits erreicht - stoppe sofort
            Motor_stopAll();
            
            communication_log(LEVEL_INFO, "=== Drehung abgeschlossen (während Soft-Start) ===");
            communication_log(LEVEL_INFO, "Ziel: %" PRId16 "° (%" PRId16 " Ticks pro Rad)", targetAngle_degrees, targetTicksValue);
            communication_log(LEVEL_INFO, "Encoder Links:  %" PRId16 " (Delta: %" PRId16 " Ticks)", currentEncoderL, deltaL);
            communication_log(LEVEL_INFO, "Encoder Rechts: %" PRId16 " (Delta: %" PRId16 " Ticks)", currentEncoderR, deltaR);
            communication_log(LEVEL_INFO, "===========================");
            
            #undef communication_log
            communication_log(LEVEL_INFO, "CorrectionRotationMovement_Wait state set.");
            #define communication_log(level, ...) ((void)0)

            setState(CorrectRotationMovement_Wait);
            initialized = 0;
            return;
        }
        
        if (timeTask_getDuration(&softStartTime, &now) >= softStartInterval) {
            currentSoftStartPWM += softStartStep;
            
            int16_t absPWM = (pwmLeft > 0) ? pwmLeft : -pwmLeft;
            if (currentSoftStartPWM >= absPWM) {
                // Ziel-PWM erreicht - wechsle zu Phase 1 (Drehen)
                Motor_setPWM(pwmLeft, pwmRight);
                timeTask_getTimestamp(&waitStart);
                phase = 1;  // Wechsle zu Drehen
                communication_log(LEVEL_INFO, "Soft-Start abgeschlossen, drehe jetzt mit PWM L=%" PRId16 " R=%" PRId16 "...", 
                                 pwmLeft, pwmRight);
            } else {
                // Erhöhe PWM weiter (beide Motoren in entgegengesetzte Richtung)
                if (pwmLeft > 0) {
                    Motor_setPWM(currentSoftStartPWM, -currentSoftStartPWM);
                } else {
                    Motor_setPWM(-currentSoftStartPWM, currentSoftStartPWM);
                }
                timeTask_getTimestamp(&softStartTime);
            }
        }
    } else if (phase == 1) {
        // Phase 1: Drehen bis Ziel erreicht
        timeTask_time_t now;
        timeTask_getTimestamp(&now);
        
        // Warte auf Betriebsdrehzahl
        int16_t absPWM = (pwmLeft > 0) ? pwmLeft : -pwmLeft;
        int16_t pwmDiff = (absPWM > minPWMForCalc) ? (absPWM - minPWMForCalc) : 0;
        uint32_t requiredWaitTime = baseSpeedWaitTime + (pwmDiff * timePerPWM);
        
        if (timeTask_getDuration(&waitStart, &now) >= requiredWaitTime) {
            // Betriebsdrehzahl erreicht - prüfe ob Ziel erreicht
            int16_t currentEncoderR = encoder_getCountR();
            int16_t currentEncoderL = encoder_getCountL();
            int16_t deltaR = currentEncoderR - startEncoderR;
            int16_t deltaL = currentEncoderL - startEncoderL;
            int16_t absDeltaR = (deltaR > 0) ? deltaR : -deltaR;
            int16_t absDeltaL = (deltaL > 0) ? deltaL : -deltaL;
            int16_t maxDelta = (absDeltaR > absDeltaL) ? absDeltaR : absDeltaL;
            
            // Prüfe ob Ziel erreicht
            if (maxDelta >= targetTicksValue) {
                Motor_stopAll();
                
                communication_log(LEVEL_INFO, "=== Drehung abgeschlossen ===");
                communication_log(LEVEL_INFO, "Ziel: %" PRId16 "° (%" PRId16 " Ticks pro Rad)", targetAngle_degrees, targetTicksValue);
                communication_log(LEVEL_INFO, "Encoder Links:  %" PRId16 " (Delta: %" PRId16 " Ticks)", currentEncoderL, deltaL);
                communication_log(LEVEL_INFO, "Encoder Rechts: %" PRId16 " (Delta: %" PRId16 " Ticks)", currentEncoderR, deltaR);
                communication_log(LEVEL_INFO, "Finale PWM-Werte: L=%" PRId16 " R=%" PRId16, pwmLeft, pwmRight);
                communication_log(LEVEL_INFO, "===========================");

                #undef communication_log
                communication_log(LEVEL_INFO, "CorrectionRotationMovement_Wait state set.");
                #define communication_log(level, ...) ((void)0)
               
                setState(CorrectRotationMovement_Wait);
                //setState(IDLE);
                initialized = 0;
            }
        }
    }
#undef communication_log
}

// ==================== SETTER-FUNKTIONEN FÜR WALL-FOLLOWING PARAMETER ====================


void statemachine_setWallDetectionThreshold(int16_t threshold_mm) {
    if (threshold_mm > 0 && threshold_mm < 200) {
        wall_detection_threshold_mm_config = threshold_mm;
        communication_log(LEVEL_INFO, "Wall detection threshold set to %" PRId16 " mm", threshold_mm);
    } else {
        communication_log(LEVEL_WARNING, "Invalid wall detection threshold: %" PRId16, threshold_mm);
    }
}

void statemachine_setWallCorrectionThreshold(int16_t threshold_mm) {
    if (threshold_mm > 0 && threshold_mm < 200) {
        wall_correction_threshold_mm_config = threshold_mm;
        communication_log(LEVEL_INFO, "Wall correction threshold set to %" PRId16 " mm", threshold_mm);
    } else {
        communication_log(LEVEL_WARNING, "Invalid wall correction threshold: %" PRId16, threshold_mm);
    }
}

void statemachine_setWallDistanceGain(int16_t gain) {
    if (gain > 0 && gain < 200) {
        wall_distance_gain_config = gain;
        communication_log(LEVEL_INFO, "Wall distance gain set to %" PRId16, gain);
    } else {
        communication_log(LEVEL_WARNING, "Invalid wall distance gain: %" PRId16, gain);
    }
}

void statemachine_setWallParallelGain(int16_t gain) {
    if (gain > 0 && gain < 200) {
        wall_parallel_gain_config = gain;
        communication_log(LEVEL_INFO, "Wall parallel gain set to %" PRId16, gain);
    } else {
        communication_log(LEVEL_WARNING, "Invalid wall parallel gain: %" PRId16, gain);
    }
}

void statemachine_setCenterTargetDistance(int16_t distance_mm) {
    if (distance_mm > 0 && distance_mm < 200) {
        center_target_distance_mm_config = distance_mm;
        communication_log(LEVEL_INFO, "Center target distance set to %" PRId16 " mm", distance_mm);
    } else {
        communication_log(LEVEL_WARNING, "Invalid center target distance: %" PRId16, distance_mm);
    }
}

void statemachine_setWallCorrectionEnabled(uint8_t enabled) {
    wall_correction_enabled = enabled ? 1 : 0;
    communication_log(LEVEL_INFO, "Wall correction %s", wall_correction_enabled ? "ENABLED" : "DISABLED");
}

uint8_t statemachine_isWallCorrectionEnabled(void) {
    return wall_correction_enabled;
}
