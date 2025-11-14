#include "position.h"
#include "encoder.h"
#include <communication/communication.h>
#include <tools/timeTask/timeTask.h>
#include <math.h>
#include <inttypes.h>

// Konstanten für Kamera-Pose und Korrektur
#define CAMERA_POSE_MAX_AGE_MS 2000          // 2 Sekunden
#define CORRECTION_POSITION_THRESHOLD_MM 50.0f  // 50mm
#define CORRECTION_ANGLE_THRESHOLD_RAD 0.2f     // ≈11°
#define ERROR_POSITION_THRESHOLD_MM 100.0f       // 100mm
#define ERROR_ANGLE_THRESHOLD_RAD 0.3f           // ≈17°

// Aktuelle erwartete Pose des Roboters (Odometrie)
static Pose_t expectedPose = { 200.0f, 0.0f, M_PI_2 };

// Kamera-Pose (AprilTag)
static Pose_t truePose = { 200.0f, 0.0f, M_PI_2 };
static timeTask_time_t lastCameraUpdate = { 0, 0 };
static uint8_t cameraPoseValid = 0;  // Flag ob Kamera-Pose jemals empfangen wurde

// Pose-Differenz zwischen Odometrie und Kamera
static Pose_t poseDifference = { 0.0f, 0.0f, 0.0f };

// Letzte Encoder-Werte für Delta-Berechnung
static int16_t lastEncoderR = 0;
static int16_t lastEncoderL = 0;
static uint8_t initialized = 0;

// Forward declaration
const RobotParameters_t* getRobotParams(void);

void position_init(const Pose_t* initialPose) {
    if (initialPose != NULL) {
        expectedPose = *initialPose;
    }
    
    // Initialisiere Encoder-Referenzwerte
    lastEncoderR = encoder_getCountR();
    lastEncoderL = encoder_getCountL();
    initialized = 1;
}

void position_updateExpectedPose(void) {
    if (!initialized) {
        // Erste Initialisierung
        lastEncoderR = encoder_getCountR();
        lastEncoderL = encoder_getCountL();
        initialized = 1;
        return;
    }
    
    // Lese aktuelle Encoder-Werte
    int16_t currentEncoderR = encoder_getCountR();
    int16_t currentEncoderL = encoder_getCountL();
    
    // Berechne Deltas seit letztem Update
    int16_t deltaR = currentEncoderR - lastEncoderR;
    int16_t deltaL = currentEncoderL - lastEncoderL;
    
    // Speichere aktuelle Werte für nächstes Update
    lastEncoderR = currentEncoderR;
    lastEncoderL = currentEncoderL;
    
    // Hole Robot-Parameter
    const RobotParameters_t* params = getRobotParams();
    
    // Berechne Differenz zwischen den Rädern
    int16_t diffLR = deltaR - deltaL;
    int16_t absDiffLR = (diffLR > 0) ? diffLR : -diffLR;
    
    // Prüfe ob Geradeausfahrt (nur bei sehr kleinen Differenzen)
    // Threshold: 2-3 Ticks für numerische Stabilität
    const int16_t threshold = 3;
    
    if (absDiffLR < threshold) {
        // Fall 1: Geradeausfahrt
        // Verwende Durchschnitt beider Räder für genauere Berechnung
        float avgDelta = ((float)(deltaR + deltaL) / 2.0f);
        float d = avgDelta * params->distPerTick;
        expectedPose.x += d * cosf(expectedPose.theta);
        expectedPose.y += d * sinf(expectedPose.theta);
        // Theta bleibt unverändert bei Geradeausfahrt
    } else {
        // Fall 2: Kurvenfahrt
        // Berechne Winkeländerung
        float dTheta = (float)diffLR * params->distPerTick / params->axleWidth;
        
        // Berechne Kurvenradius
        float R = ((float)(deltaR + deltaL) / (float)diffLR) * (params->axleWidth / 2.0f);
        
        // Berechne Positionsänderung
        expectedPose.x += R * (sinf(expectedPose.theta + dTheta) - sinf(expectedPose.theta));
        expectedPose.y += R * (cosf(expectedPose.theta) - cosf(expectedPose.theta + dTheta));
        
        // Aktualisiere Theta
        expectedPose.theta += dTheta;
    }
    
    // Normalisiere Theta auf [-π, +π]
    while (expectedPose.theta > M_PI) {
        expectedPose.theta -= 2.0f * M_PI;
    }
    while (expectedPose.theta < -M_PI) {
        expectedPose.theta += 2.0f * M_PI;
    }
}

const Pose_t* position_getCurrentPose(void) {
    return &expectedPose;
}

void position_setPose(const Pose_t* newPose) {
    if (newPose != NULL) {
        expectedPose = *newPose;
        
        // Normalisiere Theta auf [-π, +π]
        while (expectedPose.theta > M_PI) {
            expectedPose.theta -= 2.0f * M_PI;
        }
        while (expectedPose.theta < -M_PI) {
            expectedPose.theta += 2.0f * M_PI;
        }
    }
}

// Schritt 1: Kamera-Pose-Speicherung

void position_setAprilTagPose(const Pose_t* cameraPose) {
    if (cameraPose != NULL) {
        truePose = *cameraPose;
        
        // Normalisiere Theta auf [-π, +π]
        while (truePose.theta > M_PI) {
            truePose.theta -= 2.0f * M_PI;
        }
        while (truePose.theta < -M_PI) {
            truePose.theta += 2.0f * M_PI;
        }
        
        // Aktualisiere Zeitstempel
        timeTask_getTimestamp(&lastCameraUpdate);
        cameraPoseValid = 1;
    }
}

const Pose_t* position_getAprilTagPose(void) {
    return &truePose;
}

uint32_t position_getLastCameraUpdateTime(void) {
    if (!cameraPoseValid) {
        return UINT32_MAX;  // Noch nie empfangen
    }
    
    timeTask_time_t now;
    timeTask_getTimestamp(&now);
    return timeTask_getDuration(&lastCameraUpdate, &now);
}

// Schritt 2: Pose-Differenz-Berechnung

uint8_t position_hasValidCameraPose(void) {
    if (!cameraPoseValid) {
        return 0;  // Noch nie empfangen
    }
    
    // Prüfe ob Kamera-Pose nicht zu alt ist
    uint32_t age_us = position_getLastCameraUpdateTime();
    uint32_t maxAge_us = CAMERA_POSE_MAX_AGE_MS * 1000UL;
    
    return (age_us < maxAge_us) ? 1 : 0;
}

void position_calculatePoseDifference(void) {
    if (!position_hasValidCameraPose()) {
        // Keine gültige Kamera-Pose - Differenz auf 0 setzen
        poseDifference.x = 0.0f;
        poseDifference.y = 0.0f;
        poseDifference.theta = 0.0f;
        return;
    }
    
    // Berechne Differenz: truePose - expectedPose
    poseDifference.x = truePose.x - expectedPose.x;
    poseDifference.y = truePose.y - expectedPose.y;
    
    // Theta-Differenz normalisieren (kürzester Weg)
    poseDifference.theta = truePose.theta - expectedPose.theta;
    
    // Normalisiere auf [-π, +π]
    while (poseDifference.theta > M_PI) {
        poseDifference.theta -= 2.0f * M_PI;
    }
    while (poseDifference.theta < -M_PI) {
        poseDifference.theta += 2.0f * M_PI;
    }
}

const Pose_t* position_getPoseDifference(void) {
    return &poseDifference;
}

