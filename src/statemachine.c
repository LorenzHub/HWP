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

static timeTask_time_t start;

// Statische Variable für Ziel-Distanz in mm
static uint16_t targetDistance_mm = 100;  // Default: 100mm

// Statische Variable für Ziel-Ticks
static int16_t targetTicks = 1000;  // Default: 1000 Ticks

// Statische Variable für PWM-Wert des rechten Motors
static int16_t targetPWM = 3000;  // Default: 3000 PWM

// Statische Variable für Ziel-Winkel in Grad
static int16_t targetAngle_degrees = 90;  // Default: 90°
static uint8_t initialized_drive = 0;

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
        case Drive_Forward: //Motor A drives left wheel, Motor B drives right wheel
            break;
        case Drive_Forward_5sec:
            drive_Forward_5sec();
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
        case FollowThePath:
            break;
        case CorrectRotationMovement:
            correctRotationMovement();
            break;
        case CorrectRotationMovement_Wait:
            correctRotationMovement_Wait();
            break;
    }
}

void mazeExplore(void) {
    // KEINE Odometrie mehr verwenden - Position wird nur durch manuelles Zählen aktualisiert
    exploreMaze();
}

void drive_Forward_distance_mm_then_explore(uint16_t distance_mm, int16_t pwmRight){
    initialized_drive = 1;
    
    // Starte Bewegung
    drive_Forward_distance_mm(distance_mm, pwmRight);
    //correctRotationMovement();
// Wenn Bewegung normal abgeschlossen ist (IDLE)
if(currentState == IDLE) {
    // Manuelle Positionsaktualisierung nach vollständiger Vorwärtsbewegung
    //TODO Schluss
    updateLabyrinthPosition();
    setState(ExploreMaze);
    
}
// Wenn State bereits auf ExploreMaze gesetzt wurde (Wand erkannt in drive_Forward_ticks),
// dann wurde die Position bereits dort aktualisiert

    
}

void turn_degrees_then_drive(int16_t angle_degrees, int16_t pwm){
    initialized_drive = 2;
    turn_On_Spot_degrees(angle_degrees, pwm);

    if(currentState == IDLE){
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
    
    // Warte 2 Sekunden (2000000 Mikrosekunden)
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
    // TODO: Implement correctRotationMovement

    //const int16_t correctionDistance_deg = 4;
    const int16_t correctionDistance_mm = 3;
    const int16_t correctionPWM = 4000;
    const float sensorDistance_mm = 90.0f; // Abstand zwischen den Sensoren in mm
    
    if(ADC_getFilteredValue(0) > 400) { //rigth front

        int16_t diff = CalibrateIRSensors(1) - CalibrateIRSensors(0);
        //Correct rotation movement with rigth sensors    
        if(diff < -correctionDistance_mm) { //right back - right front
        communication_log(LEVEL_INFO, "difference: %d - correcting right", diff);
        int16_t angle_deg = calculateCorrectionAngle(diff, sensorDistance_mm);
        statemachine_setTargetAngle(-angle_deg);
        statemachine_setTargetPWM(correctionPWM);
        setState(turn_On_Spot_degrees_then_drive);
        
    
        }
        else if(diff > correctionDistance_mm) { //right front - right back
            communication_log(LEVEL_INFO, "difference: %d - correcting left", diff);
            int16_t angle_deg = calculateCorrectionAngle(diff, sensorDistance_mm);
            statemachine_setTargetAngle(-angle_deg); // Negativ, da nach links korrigiert wird
            statemachine_setTargetPWM(correctionPWM);
            setState(turn_On_Spot_degrees_then_drive);
            
            
        }
        else{
            
            communication_log(LEVEL_INFO, "no correction needed - difference: %d", diff);
            if(initialized_drive == 1){
                updateLabyrinthPosition();
                setState(ExploreMaze);
                initialized_drive = 0;
            }
            else if(initialized_drive == 2){
                setState(drive_Forward_distance_then_explore);
                initialized_drive = 0;
            }
            else{
                initialized_drive = 0;
            }
        }
    }
    else if(ADC_getFilteredValue(3) > 400) { //left front
        int16_t diff = CalibrateIRSensors(4) - CalibrateIRSensors(3);
        
//Correct rotation movement with left sensors
        if(diff < -correctionDistance_mm) { //left back - left front
            communication_log(LEVEL_INFO, "difference: %d - correcting left", diff);
            int16_t angle_deg = calculateCorrectionAngle(diff, sensorDistance_mm);
            statemachine_setTargetAngle(-angle_deg); // Negativ, da nach links korrigiert wird
            statemachine_setTargetPWM(correctionPWM);
            setState(turn_On_Spot_degrees_then_drive);
           
           
        }
        else if(diff > correctionDistance_mm) { //left front - left back
            communication_log(LEVEL_INFO, "difference: %d - correcting right", diff);
            int16_t angle_deg = calculateCorrectionAngle(diff, sensorDistance_mm);
            statemachine_setTargetAngle(-angle_deg); // Positiv, da nach rechts korrigiert wird
            statemachine_setTargetPWM(correctionPWM);
            setState(turn_On_Spot_degrees_then_drive);
            
            
        }
        else{
            communication_log(LEVEL_INFO, "no correction needed - difference: %d", diff);
            if(initialized_drive == 1){
                updateLabyrinthPosition();
                setState(ExploreMaze);
                initialized_drive = 0;
            }
            else if(initialized_drive == 2){
                setState(drive_Forward_distance_then_explore);
                initialized_drive = 0;
            }
            else{
                initialized_drive = 0;
            }
            
           
        }
        
    }
    else{
        communication_log(LEVEL_INFO, "no wall found for correction");
        if(initialized_drive == 1){
            updateLabyrinthPosition();
            setState(ExploreMaze);
            initialized_drive = 0;
        }
        else if(initialized_drive == 2){
            setState(drive_Forward_distance_then_explore);
            initialized_drive = 0;
        }
        else{
            initialized_drive = 0;
        }
    }
}

// Fahre eine bestimmte Anzahl von Encoder-Ticks vorwärts mit kontinuierlicher Korrektur
void drive_Forward_ticks(int16_t targetTicksValue, int16_t pwmRight) {
    // ==================== STATISCHE VARIABLEN ====================
    // Initialisierung und Zustand
    static uint8_t initialized = 0;
    static uint8_t phase = 0;  // 0 = Soft-Start, 1 = Fahren, 2 = Abbremsen
    
    // Encoder-Werte
    static int16_t startEncoderR = 0;
    static int16_t startEncoderL = 0;
    static int16_t lastEncoderR = 0;
    static int16_t lastEncoderL = 0;
    
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
    
    // PI-Regler Parameter
    const int16_t Kp = 50;                      // Proportional-Verstärkung (skaliert mit /100)
    const int16_t Ki = 10;                      // Integral-Verstärkung (skaliert mit /100)
    const int32_t maxIntegral = 5000;           // Anti-Windup Grenze
    const int16_t deadZone = 2;                 // Totzone für Geschwindigkeitsdifferenz
    const int16_t maxAdjustment = 500;          // Maximale PWM-Korrektur
    const int16_t maxSlewRate = 100;            // Maximale PWM-Änderung pro Zyklus
    const int16_t positionCorrectionThreshold = 5; // Schwelle für Positions-Korrektur
    
    // Timing Parameter
    const uint32_t controlLoopInterval = 20000UL; // 20ms Regelzyklus
    const uint32_t logInterval = 100000UL;        // 100ms Log-Intervall
    const int16_t minSpeedTicks = 3;              // Mindest-Ticks für gültige Geschwindigkeitsmessung
    
    // Abbremsen Parameter
    const int16_t rampDownThreshold = 100;      // Ticks vor Ziel: Abbremsen beginnen
    
    // ==================== WAND-ERKENNUNG ====================
    if (currentState == drive_Forward_distance_then_explore) {
        if (ADC_getFilteredValue(2) > 750) {
            Motor_stopAll();
            communication_log(LEVEL_INFO, "Front wall detected - stopping movement");
            updateLabyrinthPosition();
            setState(ExploreMaze);
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
        communication_log(LEVEL_INFO, "Fahre %" PRId16 " Ticks, Ziel-PWM L=%" PRId16 " R=%" PRId16, 
                         targetTicksValue, targetPWMLeft, targetPWMRight);
    }
    
    // ==================== AKTUELLE WERTE LESEN ====================
    timeTask_time_t now;
    timeTask_getTimestamp(&now);
    
    int16_t currentEncoderR = encoder_getCountR();
    int16_t currentEncoderL = encoder_getCountL();
    int16_t deltaR = currentEncoderR - startEncoderR;
    int16_t deltaL = currentEncoderL - startEncoderL;
    int16_t avgDelta = (deltaL + deltaR) / 2;
    int16_t remainingTicks = targetTicksValue - avgDelta;
    int16_t positionDiff = deltaR - deltaL;  // Positions-Differenz zwischen Rädern
    
    // ==================== ZIEL ERREICHT? ====================
    if (avgDelta >= targetTicksValue) {
        Motor_stopAll();
        
        int16_t encoderDiff = deltaR - deltaL;
        int16_t absEncoderDiff = (encoderDiff > 0) ? encoderDiff : -encoderDiff;
        uint16_t actualDistance_mm = (uint16_t)((int32_t)avgDelta * 688 / 10000);
        uint32_t duration_ms = timeTask_getDuration(&startOfDrive, &now) / 1000;
        
        communication_log(LEVEL_INFO, "=== Fahrt abgeschlossen ===");
        communication_log(LEVEL_INFO, "Ziel: %" PRId16 " Ticks, Erreicht: %" PRId16 " Ticks", targetTicksValue, avgDelta);
        communication_log(LEVEL_INFO, "Encoder L=%" PRId16 " R=%" PRId16 " Diff=%" PRId16, deltaL, deltaR, encoderDiff);
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
        
        setState(IDLE);
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
    if (phase == 1 && remainingTicks <= rampDownThreshold) {
        phase = 2;
        communication_log(LEVEL_INFO, "Beginne Abbremsen, verbleibend: %" PRId16 " Ticks", remainingTicks);
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
                posTerm = (positionDiff * Kp) / 200;
            }
            
            u = pTerm + iTerm + posTerm;
            
            // Begrenzung
            if (u > maxAdjustment) u = maxAdjustment;
            if (u < -maxAdjustment) u = -maxAdjustment;
        } else {
            // Innerhalb Deadzone - Integral langsam abbauen
            integralError = (integralError * 9) / 10;
        }
    }
    
    // Basis-PWM bestimmen (mit Ramp-Down in Phase 2)
    int16_t basePWMLeft = targetPWMLeft;
    int16_t basePWMRight = targetPWMRight;
    
    if (phase == 2) {
        // Ramp-Down: PWM proportional zur verbleibenden Distanz reduzieren
        int16_t rampFactor = (remainingTicks * 100) / rampDownThreshold;
        if (rampFactor < 30) rampFactor = 30;   // Min. 30% PWM
        if (rampFactor > 100) rampFactor = 100;
        basePWMLeft = (targetPWMLeft * rampFactor) / 100;
        basePWMRight = (targetPWMRight * rampFactor) / 100;
    }
    
    // Symmetrische PWM-Anpassung
    int16_t pwmLeft = basePWMLeft - u;
    int16_t pwmRight_new = basePWMRight + u;
    
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
    
    Motor_setPWM(pwmLeft, pwmRight_new);
    
    // Logging (10 Hz)
    if (timeTask_getDuration(&lastLogTime, &now) >= logInterval) {
        int16_t speedDiff = filteredSpeedR - filteredSpeedL;
        communication_log(LEVEL_INFO, "P%" PRIu8 ": sL=%" PRId16 " sR=%" PRId16 " rem=%" PRId16 " u=%" PRId16 " pL=%" PRId16 " pR=%" PRId16, 
                         phase, filteredSpeedL, filteredSpeedR, remainingTicks, u, pwmLeft, pwmRight_new);
        timeTask_getTimestamp(&lastLogTime);
    }
    
    // Werte für nächsten Zyklus speichern
    lastEncoderR = currentEncoderR;
    lastEncoderL = currentEncoderL;
    timeTask_getTimestamp(&controlLoopTime);
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
    //#define communication_log(level, ...) ((void)0)
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
            
            setState(IDLE);
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

                
               setState(IDLE);//Temp
                //setState(CorrectRotationMovement_Wait);
                initialized = 0;
            }
        }
    }
#undef communication_log
}
