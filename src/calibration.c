#include "calibration.h"
#include "statemachine.h"
#include <motor/motor.h>
#include <tools/timeTask/timeTask.h>
#include "encoder.h"
#include <communication/communication.h>
#include <inttypes.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>

// EEPROM-Struktur für gespeicherte PWM-Werte
#define PWM_CALIBRATION_MAGIC 0xCAFE  // Magic number zur Validierung
#define CALIBRATION_PWM_COUNT 10      // Anzahl der kalibrierten PWM-Werte

typedef struct {
    uint16_t magic;      // Magic number (0xCAFE = gültig)
    int16_t pwmLeft[10];  // Kalibrierte PWM-Werte für linkes Rad (für 3000, 3500, ..., 7500)
    int16_t pwmRight[10]; // Kalibrierte PWM-Werte für rechtes Rad (für 3000, 3500, ..., 7500)
} PWMCalibration_t;

// Statische Variablen für geladene PWM-Werte
static int16_t calibratedPWMLeft[10] = {3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000};   // Default-Werte: alle 3000
static int16_t calibratedPWMRight[10] = {3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000};  // Default-Werte: alle 3000
static uint8_t calibrationLoaded = 0;   // Flag ob Kalibrierung geladen wurde

// EEPROM-Instanz (wird zur Laufzeit nicht verwendet, nur für Adresse)
static PWMCalibration_t calibrationData EEMEM;

// PWM-Werte für die Kalibrierung
static const int16_t calibrationPWMValues[10] = {3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500};

// Struktur für Kalibrierungsergebnisse
typedef struct {
    int16_t pwmValue;
    int16_t pwmLeft;   // Tatsächlicher PWM-Wert für linken Motor
    int16_t pwmRight;  // Tatsächlicher PWM-Wert für rechten Motor
    int16_t speedDiff; // Geschwindigkeitsdifferenz beim Abschluss
    uint8_t iterations; // Anzahl Anpassungen
} CalibrationResult_t;

// Lade PWM-Werte aus EEPROM
static void loadPWMCalibration(void) {
    uint8_t sreg = SREG;
    cli(); // Interrupts deaktivieren während EEPROM-Zugriff
    eeprom_busy_wait();
    uint16_t magic = eeprom_read_word((uint16_t*)&calibrationData.magic);
    SREG = sreg; // Interrupts wieder aktivieren
    
    if (magic == PWM_CALIBRATION_MAGIC) {
        // Lade alle 10 PWM-Werte
        sreg = SREG;
        cli();
        uint8_t valid = 1;  // Prüfe ob alle Werte gültig sind
        for (uint8_t i = 0; i < CALIBRATION_PWM_COUNT; i++) {
            eeprom_busy_wait();
            uint16_t pwmLeft_u = eeprom_read_word((uint16_t*)&calibrationData.pwmLeft[i]);
            eeprom_busy_wait();
            uint16_t pwmRight_u = eeprom_read_word((uint16_t*)&calibrationData.pwmRight[i]);
            
            // Prüfe ob Werte gültig sind (nicht 0xFFFF = uninitialisiertes EEPROM)
            // PWM-Werte sollten zwischen -8191 und 8191 sein, aber wir erwarten positive Werte
            if (pwmLeft_u == 0xFFFF || pwmRight_u == 0xFFFF || 
                (int16_t)pwmLeft_u <= 0 || (int16_t)pwmRight_u <= 0 ||
                (int16_t)pwmLeft_u > 8191 || (int16_t)pwmRight_u > 8191) {
                valid = 0;
                break;
            }
            
            calibratedPWMLeft[i] = (int16_t)pwmLeft_u;
            calibratedPWMRight[i] = (int16_t)pwmRight_u;
        }
        SREG = sreg;
        
        if (valid) {
            calibrationLoaded = 1;
        } else {
            // Ungültige Werte gefunden - verwende Defaults
            for (uint8_t i = 0; i < CALIBRATION_PWM_COUNT; i++) {
                calibratedPWMLeft[i] = calibrationPWMValues[i];
                calibratedPWMRight[i] = calibrationPWMValues[i];
            }
            calibrationLoaded = 0;
        }
    } else {
        // Keine gespeicherten Werte - verwende Defaults
        for (uint8_t i = 0; i < CALIBRATION_PWM_COUNT; i++) {
            calibratedPWMLeft[i] = calibrationPWMValues[i];
            calibratedPWMRight[i] = calibrationPWMValues[i];
        }
        calibrationLoaded = 0;
    }
}

// Speichere alle PWM-Werte in EEPROM
static void savePWMCalibration(CalibrationResult_t* results) {
    uint8_t sreg = SREG;
    cli(); // Interrupts deaktivieren während EEPROM-Zugriff
    
    eeprom_busy_wait();
    eeprom_write_word((uint16_t*)&calibrationData.magic, PWM_CALIBRATION_MAGIC);
    
    // Speichere alle 10 PWM-Werte
    for (uint8_t i = 0; i < CALIBRATION_PWM_COUNT; i++) {
        eeprom_busy_wait();
        eeprom_write_word((uint16_t*)&calibrationData.pwmLeft[i], (uint16_t)results[i].pwmLeft);
        eeprom_busy_wait();
        eeprom_write_word((uint16_t*)&calibrationData.pwmRight[i], (uint16_t)results[i].pwmRight);
    }
    
    SREG = sreg; // Interrupts wieder aktivieren
    
    // Aktualisiere auch die statischen Arrays
    for (uint8_t i = 0; i < CALIBRATION_PWM_COUNT; i++) {
        calibratedPWMLeft[i] = results[i].pwmLeft;
        calibratedPWMRight[i] = results[i].pwmRight;
    }
    calibrationLoaded = 1;
}

// Initialisiere PWM-Kalibrierung (beim Start aufrufen)
void calibration_init(void) {
    loadPWMCalibration();
}

// Finde den besten passenden PWM-Wert für einen gewünschten PWM-Wert
// Gibt den Index zurück oder interpoliert zwischen zwei Werten
static void findBestPWMValues(int16_t desiredPWM, int16_t* pwmLeft, int16_t* pwmRight) {
    // Wenn keine Kalibrierung geladen, verwende Defaults
    if (!calibrationLoaded) {
        *pwmLeft = desiredPWM;
        *pwmRight = desiredPWM;
        return;
    }
    
    // Finde nächstgelegenen oder interpoliere
    if (desiredPWM <= calibrationPWMValues[0]) {
        // Unterhalb des Minimums - verwende ersten Wert
        *pwmLeft = calibratedPWMLeft[0];
        *pwmRight = calibratedPWMRight[0];
    } else if (desiredPWM >= calibrationPWMValues[CALIBRATION_PWM_COUNT - 1]) {
        // Oberhalb des Maximums - verwende letzten Wert
        *pwmLeft = calibratedPWMLeft[CALIBRATION_PWM_COUNT - 1];
        *pwmRight = calibratedPWMRight[CALIBRATION_PWM_COUNT - 1];
    } else {
        // Finde zwei benachbarte Werte für Interpolation
        uint8_t lowerIndex = 0;
        for (uint8_t i = 0; i < CALIBRATION_PWM_COUNT - 1; i++) {
            if (desiredPWM >= calibrationPWMValues[i] && desiredPWM <= calibrationPWMValues[i + 1]) {
                lowerIndex = i;
                break;
            }
        }
        
        uint8_t upperIndex = lowerIndex + 1;
        int16_t lowerPWM = calibrationPWMValues[lowerIndex];
        int16_t upperPWM = calibrationPWMValues[upperIndex];
        
        // Interpoliere zwischen den beiden Werten
        int16_t pwmDiff = desiredPWM - lowerPWM;
        int16_t pwmRange = upperPWM - lowerPWM;
        
        if (pwmRange > 0) {
            // Lineare Interpolation
            int32_t leftDiff = (int32_t)(calibratedPWMLeft[upperIndex] - calibratedPWMLeft[lowerIndex]) * pwmDiff / pwmRange;
            int32_t rightDiff = (int32_t)(calibratedPWMRight[upperIndex] - calibratedPWMRight[lowerIndex]) * pwmDiff / pwmRange;
            
            *pwmLeft = calibratedPWMLeft[lowerIndex] + (int16_t)leftDiff;
            *pwmRight = calibratedPWMRight[lowerIndex] + (int16_t)rightDiff;
        } else {
            // Fallback: verwende unteren Wert
            *pwmLeft = calibratedPWMLeft[lowerIndex];
            *pwmRight = calibratedPWMRight[lowerIndex];
        }
    }
}

// Hole kalibrierte PWM-Werte für einen gewünschten PWM-Wert
int16_t calibration_getPWMLeft(int16_t desiredPWM) {
    int16_t pwmLeft, pwmRight;
    findBestPWMValues(desiredPWM, &pwmLeft, &pwmRight);
    return pwmLeft;
}

int16_t calibration_getPWMRight(int16_t desiredPWM) {
    int16_t pwmLeft, pwmRight;
    findBestPWMValues(desiredPWM, &pwmLeft, &pwmRight);
    return pwmRight;
}

void calibration_run(void) {
    static uint8_t initialized = 0;
    static uint8_t phase = 0; // 0 = Soft-Start, 1 = Geschwindigkeitssynchronisation, 2 = Endgeschwindigkeit halten (5s), 3 = Übersicht
    static int16_t basePWM = 0;      // Basis-PWM-Wert für diesen Durchgang
    static int16_t pwmLeft = 0;      // Aktueller PWM für links (wird angepasst)
    static int16_t pwmRight = 0;     // Aktueller PWM für rechts (wird angepasst)
    static int16_t currentSoftStartPWM = 0; // Aktueller PWM während Soft-Start
    static uint8_t pwmIndex = 0;     // Index für aktuellen PWM-Wert
    static uint8_t iteration = 0;    // Iteration für aktuellen PWM-Wert
    static timeTask_time_t waitStart;
    static timeTask_time_t speedMeasureStart; // Startzeit für Geschwindigkeitsmessung
    static int16_t speedMeasureEncoderR = 0; // Encoder-Werte bei Start der Geschwindigkeitsmessung
    static int16_t speedMeasureEncoderL = 0;
    static uint8_t speedMeasureInitialized = 0; // Flag ob Geschwindigkeitsmessung initialisiert wurde
    static CalibrationResult_t results[10]; // 10 PWM-Werte: 3000, 3500, ..., 7500
    
    const int16_t pwmValues[10] = {3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500};
    const int16_t softStartMinPWM = 1000; // Start-PWM für Soft-Start
    const int16_t softStartStep = 200;    // PWM-Schrittweite für Soft-Start
    const uint32_t softStartInterval = 50000UL; // 50ms zwischen Soft-Start-Schritten
    const uint32_t baseSpeedWaitTime = 200000UL; // 200ms Basis-Wartezeit nach Soft-Start
    const uint32_t timePerPWM = 50UL; // 0.05ms pro PWM-Einheit (50µs)
    const int16_t minPWMForCalc = 3000; // Referenz-PWM für Berechnung
    const uint32_t speedMeasureInterval = 200000UL; // 200ms für Geschwindigkeitsmessung
    const int16_t targetSpeedDiff = 3; // Ziel: Geschwindigkeitsdifferenz < 3 Ticks pro 200ms (etwas lockerer)
    const uint8_t maxIterations = 25;  // Max. Iterationen pro PWM-Wert (erhöht)
    const int16_t minSpeedTicks = 10;  // Mindest-Ticks für gültige Geschwindigkeitsmessung
    const uint32_t endSpeedHoldTime = 5000000UL; // 5 Sekunden Endgeschwindigkeit halten
    static timeTask_time_t endSpeedStart; // Startzeit für Endgeschwindigkeits-Überprüfung (nur wenn Diff < 3)
    static int16_t endSpeedEncoderR = 0; // Encoder-Werte bei Start der Endgeschwindigkeits-Überprüfung
    static int16_t endSpeedEncoderL = 0;
    static timeTask_time_t endSpeedMeasureStart; // Startzeit für kontinuierliche Messung während 5s
    static int16_t endSpeedMeasureEncoderR = 0; // Encoder-Werte für kontinuierliche Messung
    static int16_t endSpeedMeasureEncoderL = 0;
    static uint8_t endSpeedMeasureInitialized = 0; // Flag ob kontinuierliche Messung initialisiert wurde
    static uint8_t stableDiffStarted = 0; // Flag ob 5s-Timer für stabile Differenz gestartet wurde
    static timeTask_time_t lastLogTime; // Zeitpunkt der letzten Log-Nachricht (jede Sekunde)
    static timeTask_time_t phase2StartTime; // Startzeit von Phase 2 für Zeitberechnung
    
    if (!initialized) {
        phase = 0; // Starte mit Soft-Start
        pwmIndex = 0;
        iteration = 0;
        
        // Starte immer bei PWM 3000 (Index 0)
        basePWM = pwmValues[0];  // 3000
        pwmIndex = 0;
        
        // Rechter Motor: Immer genau basePWM
        pwmRight = basePWM;
        
        // Linker Motor: Verwende gespeicherten Wert als Startwert (falls vorhanden)
        if (calibrationLoaded) {
            // Gespeicherte Werte vorhanden - verwende Wert für PWM 3000 (Index 0)
            pwmLeft = calibratedPWMLeft[0];
            communication_log(LEVEL_INFO, "Starte mit gespeichertem Wert: L=%" PRId16 " (R=%" PRId16 " fix)", 
                             pwmLeft, basePWM);
        } else {
            // Erste Kalibrierung - starte mit basePWM für links
            pwmLeft = basePWM;
            communication_log(LEVEL_INFO, "Erste Kalibrierung - starte mit L=%" PRId16 " R=%" PRId16, 
                             basePWM, basePWM);
        }
        
        currentSoftStartPWM = softStartMinPWM;
        Motor_setPWM(currentSoftStartPWM, currentSoftStartPWM);
        timeTask_getTimestamp(&waitStart);
        communication_log(LEVEL_INFO, "=== Kalibrierung gestartet ===");
        communication_log(LEVEL_INFO, "PWM-Wert %d/%d: %" PRId16 " (Soft-Start von %" PRId16 ", Start: L=%" PRId16 " R=%" PRId16 ")", 
                         pwmIndex + 1, 10, basePWM, softStartMinPWM, pwmLeft, pwmRight);
        initialized = 1;
    }
    
    timeTask_time_t now;
    timeTask_getTimestamp(&now);
    
    if (phase == 0) {
        // Phase 0: Soft-Start - PWM schrittweise hochfahren
        if (timeTask_getDuration(&waitStart, &now) >= softStartInterval) {
            currentSoftStartPWM += softStartStep;
            
            if (currentSoftStartPWM >= basePWM) {
                // Ziel-PWM erreicht - wechsle zu Phase 1 (Geschwindigkeitssynchronisation)
                currentSoftStartPWM = basePWM;
                // Rechter Motor: Immer genau basePWM
                pwmRight = basePWM;
                Motor_setPWM(pwmLeft, pwmRight);
                phase = 1;
                timeTask_getTimestamp(&waitStart);
                // WICHTIG: Setze Encoder-Startwerte für Geschwindigkeitsmessung erst NACH Betriebsdrehzahl
                // (werden in Phase 1 gesetzt, wenn Betriebsdrehzahl erreicht ist)
                communication_log(LEVEL_INFO, "Soft-Start abgeschlossen, warte auf Betriebsdrehzahl...");
            } else {
                // Erhöhe PWM weiter
                Motor_setPWM(currentSoftStartPWM, currentSoftStartPWM);
                timeTask_getTimestamp(&waitStart);
            }
        }
    } else if (phase == 1) {
        // Phase 1: Geschwindigkeitssynchronisation
        // Warte erst auf Betriebsdrehzahl
        int16_t pwmDiff = (basePWM > minPWMForCalc) ? (basePWM - minPWMForCalc) : 0;
        uint32_t requiredWaitTime = baseSpeedWaitTime + (pwmDiff * timePerPWM);
        
        if (timeTask_getDuration(&waitStart, &now) >= requiredWaitTime) {
            // Betriebsdrehzahl erreicht - setze Encoder-Startwerte für erste Messung
            // (nur beim ersten Mal nach Erreichen der Betriebsdrehzahl)
            if (!speedMeasureInitialized) {
                speedMeasureEncoderR = encoder_getCountR();
                speedMeasureEncoderL = encoder_getCountL();
                timeTask_getTimestamp(&speedMeasureStart);
                speedMeasureInitialized = 1;
                communication_log(LEVEL_INFO, "Betriebsdrehzahl erreicht, starte Geschwindigkeitsmessung...");
            }
            
            // Starte Geschwindigkeitsmessung
            if (timeTask_getDuration(&speedMeasureStart, &now) >= speedMeasureInterval) {
                // Messperiode abgelaufen - messe Geschwindigkeit
                int16_t currentEncoderR = encoder_getCountR();
                int16_t currentEncoderL = encoder_getCountL();
                int16_t speedR = currentEncoderR - speedMeasureEncoderR;
                int16_t speedL = currentEncoderL - speedMeasureEncoderL;
                
                // Prüfe ob genug Bewegung für gültige Messung
                int16_t absSpeedR = (speedR > 0) ? speedR : -speedR;
                int16_t absSpeedL = (speedL > 0) ? speedL : -speedL;
                
                if (absSpeedR >= minSpeedTicks && absSpeedL >= minSpeedTicks) {
                    // Gültige Messung - berechne Differenz
                    int16_t speedDiff = speedR - speedL;
                    int16_t absSpeedDiff = (speedDiff > 0) ? speedDiff : -speedDiff;
                    
                    // Prüfe ob synchronisiert
                    if (absSpeedDiff <= targetSpeedDiff) {
                        // Erfolgreich synchronisiert! - Wechsle zu Phase 2 (Endgeschwindigkeit halten)
                        communication_log(LEVEL_INFO, "PWM %" PRId16 ": Synchronisiert! (Speed L=%" PRId16 " R=%" PRId16 " Diff=%" PRId16 ", PWM L=%" PRId16 " R=%" PRId16 ") - %d Iterationen", 
                                         basePWM, speedL, speedR, speedDiff, pwmLeft, pwmRight, iteration);
                        communication_log(LEVEL_INFO, "Halte Endgeschwindigkeit für 5 Sekunden mit kontinuierlicher Korrektur...");
                        
                        // Wechsle zu Phase 2 (Endgeschwindigkeit halten mit kontinuierlicher Korrektur)
                        phase = 2; // Wechsle zu Endgeschwindigkeit halten
                        speedMeasureInitialized = 0; // Reset für nächste Phase
                        endSpeedMeasureInitialized = 0; // Reset für 2s-Überprüfung
                    } else {
                        // Noch nicht synchronisiert - passe PWM-Werte an
                        iteration++;
                        if (iteration >= maxIterations) {
                            // Max. Iterationen erreicht - wechsle trotzdem zu Phase 2 (Endgeschwindigkeit halten)
                            communication_log(LEVEL_INFO, "PWM %" PRId16 ": Max Iterationen erreicht (Speed L=%" PRId16 " R=%" PRId16 " Diff=%" PRId16 ", PWM L=%" PRId16 " R=%" PRId16 ")", 
                                             basePWM, speedL, speedR, speedDiff, pwmLeft, pwmRight);
                            communication_log(LEVEL_INFO, "Halte Endgeschwindigkeit für 5 Sekunden zur Überprüfung...");
                            
                            // Wechsle zu Phase 2 (Endgeschwindigkeit halten mit kontinuierlicher Korrektur)
                            phase = 2; // Wechsle zu Endgeschwindigkeit halten
                            speedMeasureInitialized = 0; // Reset für nächste Phase
                            endSpeedMeasureInitialized = 0; // Reset für 5s-Überprüfung
                        } else {
                            // Passe nur linken Motor an, rechter bleibt auf basePWM
                            // speedDiff = speedR - speedL
                            // Wenn speedDiff > 0: R ist schneller -> erhöhe pwmLeft
                            // Wenn speedDiff < 0: L ist schneller -> reduziere pwmLeft
                            
                            // Rechter Motor: Immer genau basePWM
                            pwmRight = basePWM;
                            
                            // Adaptive Anpassung basierend auf Geschwindigkeitsdifferenz
                            // Größere Schritte für schnellere Konvergenz
                            int16_t adjustment;
                            if (absSpeedDiff > 30) {
                                adjustment = absSpeedDiff / 2; // Sehr große Differenz - große Schritte
                            } else if (absSpeedDiff > 15) {
                                adjustment = absSpeedDiff / 3; // Große Differenz
                            } else if (absSpeedDiff > 5) {
                                adjustment = absSpeedDiff / 2; // Mittlere Differenz
                            } else {
                                adjustment = absSpeedDiff; // Kleine Differenz - direkte Anpassung
                            }
                            
                            // Mindestanpassung: mindestens 1
                            if (adjustment < 1) adjustment = 1;
                            
                            // Maximum-Anpassung begrenzen (nicht zu große Sprünge)
                            if (adjustment > 50) adjustment = 50;
                            
                            // Flüssige Anpassung nur des linken Motors
                            if (speedDiff > 0) {
                                // Rechtes Rad ist schneller -> erhöhe pwmLeft
                                pwmLeft += adjustment;
                            } else {
                                // Linkes Rad ist schneller -> reduziere pwmLeft
                                pwmLeft -= adjustment;
                            }
                            
                            // Begrenze linken PWM-Wert (nicht zu weit vom Basis-Wert entfernen)
                            // Erhöhte Toleranz für größere Anpassungen
                            int16_t maxDeviation = basePWM / 2; // Max. 50% Abweichung (erhöht von 25%)
                            if (pwmLeft < basePWM - maxDeviation) pwmLeft = basePWM - maxDeviation;
                            if (pwmLeft > basePWM + maxDeviation) pwmLeft = basePWM + maxDeviation;
                            
                            // Absolute Grenzen: PWM muss zwischen 1000 und 8191 sein
                            if (pwmLeft < 1000) pwmLeft = 1000;
                            if (pwmLeft > 8191) pwmLeft = 8191;
                            
                            // Setze neue PWM-Werte direkt (Motoren laufen weiter)
                            Motor_setPWM(pwmLeft, pwmRight);
                            
                            communication_log(LEVEL_INFO, "PWM %" PRId16 " Iter %d/%d: Speed L=%" PRId16 " R=%" PRId16 " Diff=%" PRId16 " -> PWM L=%" PRId16 " R=%" PRId16, 
                                             basePWM, iteration, maxIterations, speedL, speedR, speedDiff, pwmLeft, pwmRight);
                            
                            // Starte neue Geschwindigkeitsmessung
                            speedMeasureEncoderR = encoder_getCountR();
                            speedMeasureEncoderL = encoder_getCountL();
                            timeTask_getTimestamp(&speedMeasureStart);
                        }
                    }
                } else {
                    // Noch nicht genug Bewegung - warte weiter
                    speedMeasureEncoderR = encoder_getCountR();
                    speedMeasureEncoderL = encoder_getCountL();
                    timeTask_getTimestamp(&speedMeasureStart);
                }
            }
        }
    } else if (phase == 2) {
        // Phase 2: Endgeschwindigkeit für 5 Sekunden mit Diff < 3 halten und kontinuierlich nachjustieren
        timeTask_time_t now;
        timeTask_getTimestamp(&now);
        
        // Initialisiere kontinuierliche Messung beim Start von Phase 2
        if (!endSpeedMeasureInitialized) {
            endSpeedEncoderR = encoder_getCountR();
            endSpeedEncoderL = encoder_getCountL();
            endSpeedMeasureEncoderR = encoder_getCountR();
            endSpeedMeasureEncoderL = encoder_getCountL();
            timeTask_getTimestamp(&endSpeedMeasureStart);
            timeTask_getTimestamp(&lastLogTime);
            timeTask_getTimestamp(&phase2StartTime);
            endSpeedMeasureInitialized = 1;
            stableDiffStarted = 0; // Timer für stabile Differenz noch nicht gestartet
        }
        
        // Kontinuierliche Korrektur während der 5 Sekunden (alle 200ms)
        if (timeTask_getDuration(&endSpeedMeasureStart, &now) >= speedMeasureInterval) {
            // Messperiode abgelaufen - messe aktuelle Geschwindigkeit
            int16_t currentEncoderR = encoder_getCountR();
            int16_t currentEncoderL = encoder_getCountL();
            int16_t speedR = currentEncoderR - endSpeedMeasureEncoderR;
            int16_t speedL = currentEncoderL - endSpeedMeasureEncoderL;
            
            // Prüfe ob genug Bewegung für gültige Messung
            int16_t absSpeedR = (speedR > 0) ? speedR : -speedR;
            int16_t absSpeedL = (speedL > 0) ? speedL : -speedL;
            
            if (absSpeedR >= minSpeedTicks && absSpeedL >= minSpeedTicks) {
                // Gültige Messung - berechne Differenz
                int16_t speedDiff = speedR - speedL;
                int16_t absSpeedDiff = (speedDiff > 0) ? speedDiff : -speedDiff;
                
                // Prüfe ob Differenz < 3 Ticks ist (stabil)
                if (absSpeedDiff <= 3) {
                    // Differenz ist stabil - starte oder setze Timer fort
                    if (!stableDiffStarted) {
                        // Timer starten (nur wenn Differenz < 3)
                        endSpeedEncoderR = encoder_getCountR();
                        endSpeedEncoderL = encoder_getCountL();
                        timeTask_getTimestamp(&endSpeedStart);
                        stableDiffStarted = 1;
                        communication_log(LEVEL_INFO, "PWM %" PRId16 ": Differenz stabil (%" PRId16 "), Timer gestartet", basePWM, absSpeedDiff);
                    }
                } else {
                    // Differenz ist nicht stabil (> 3 Ticks) - Timer zurücksetzen
                    if (stableDiffStarted) {
                        stableDiffStarted = 0;
                        communication_log(LEVEL_INFO, "PWM %" PRId16 ": Differenz überschritten (%" PRId16 " > 3), Timer zurückgesetzt - 5s müssen neu gestartet werden", basePWM, absSpeedDiff);
                    }
                }
                
                // Kontinuierliche Korrektur während der 5 Sekunden (konservativer als Phase 1)
                if (absSpeedDiff > 3) {  // Nur korrigieren wenn Differenz > 3 Ticks
                    // Passe linken Motor an (kleinere Anpassung für Langzeit-Feinabstimmung)
                    int16_t adjustment;
                    if (absSpeedDiff > 15) {
                        adjustment = absSpeedDiff / 3; // Große Differenz - konservativer
                        if (adjustment > 8) adjustment = 8; // Max. 8 PWM pro Anpassung
                    } else if (absSpeedDiff > 8) {
                        adjustment = absSpeedDiff / 3; // Mittlere Differenz
                        if (adjustment > 5) adjustment = 5; // Max. 5 PWM pro Anpassung
                    } else {
                        adjustment = absSpeedDiff / 2; // Kleine Differenz
                        if (adjustment > 3) adjustment = 3; // Max. 3 PWM pro Anpassung
                    }
                    
                    // Mindestanpassung: mindestens 1
                    if (adjustment < 1 && absSpeedDiff > 0) adjustment = 1;
                    
                    // Flüssige Anpassung nur des linken Motors
                    if (speedDiff > 0) {
                        // Rechtes Rad ist schneller -> erhöhe pwmLeft
                        pwmLeft += adjustment;
                    } else if (speedDiff < 0) {
                        // Linkes Rad ist schneller -> reduziere pwmLeft
                        pwmLeft -= adjustment;
                    }
                    
                    // Begrenze PWM-Wert
                    int16_t maxDeviation = basePWM / 2;
                    if (pwmLeft < basePWM - maxDeviation) pwmLeft = basePWM - maxDeviation;
                    if (pwmLeft > basePWM + maxDeviation) pwmLeft = basePWM + maxDeviation;
                    if (pwmLeft < 1000) pwmLeft = 1000;
                    if (pwmLeft > 8191) pwmLeft = 8191;
                    
                    // Setze neue PWM-Werte direkt (Motoren laufen weiter)
                    Motor_setPWM(pwmLeft, pwmRight);
                    
                    // Optional: Logge größere Anpassungen
                    if (adjustment > 3) {
                        communication_log(LEVEL_INFO, "PWM %" PRId16 ": 5s-Korrektur: Speed L=%" PRId16 " R=%" PRId16 " Diff=%" PRId16 " -> PWM L=%" PRId16 " R=%" PRId16, 
                                         basePWM, speedL, speedR, speedDiff, pwmLeft, pwmRight);
                    }
                }
                
                // Starte neue Geschwindigkeitsmessung
                endSpeedMeasureEncoderR = encoder_getCountR();
                endSpeedMeasureEncoderL = encoder_getCountL();
                timeTask_getTimestamp(&endSpeedMeasureStart);
                
                // Log-Nachricht jede Sekunde (1000ms)
                if (timeTask_getDuration(&lastLogTime, &now) >= 1000000UL) {
                    int16_t currentEncoderR = encoder_getCountR();
                    int16_t currentEncoderL = encoder_getCountL();
                    int16_t totalDeltaR = currentEncoderR - endSpeedEncoderR;
                    int16_t totalDeltaL = currentEncoderL - endSpeedEncoderL;
                    int16_t totalDiff = totalDeltaR - totalDeltaL;
                    int16_t absTotalDiff = (totalDiff > 0) ? totalDiff : -totalDiff;
                    
                    // Berechne verstrichene Zeit seit Start von Phase 2
                    uint32_t elapsedTime_us = timeTask_getDuration(&phase2StartTime, &now);
                    uint32_t elapsedTime_s = elapsedTime_us / 1000000UL;
                    
                    communication_log(LEVEL_INFO, "PWM %" PRId16 ": [%lu s] Speed L=%" PRId16 " R=%" PRId16 " Ticks, Diff=%" PRId16 " (PWM L=%" PRId16 " R=%" PRId16 ")", 
                                     basePWM, elapsedTime_s, totalDeltaL, totalDeltaR, absTotalDiff, pwmLeft, pwmRight);
                    timeTask_getTimestamp(&lastLogTime);
                }
            } else {
                // Noch nicht genug Bewegung - warte weiter
                endSpeedMeasureEncoderR = encoder_getCountR();
                endSpeedMeasureEncoderL = encoder_getCountL();
                timeTask_getTimestamp(&endSpeedMeasureStart);
            }
        }
        
        // Prüfe ob 5 Sekunden mit stabiler Differenz (< 3 Ticks) abgelaufen sind
        if (stableDiffStarted && timeTask_getDuration(&endSpeedStart, &now) >= endSpeedHoldTime) {
            // 5 Sekunden abgelaufen - messe finale Geschwindigkeit
            int16_t currentEncoderR = encoder_getCountR();
            int16_t currentEncoderL = encoder_getCountL();
            int16_t totalDeltaR = currentEncoderR - endSpeedEncoderR;
            int16_t totalDeltaL = currentEncoderL - endSpeedEncoderL;
            int16_t totalDiff = totalDeltaR - totalDeltaL;
            int16_t absTotalDiff = (totalDiff > 0) ? totalDiff : -totalDiff;
            
            // Durchschnittliche Geschwindigkeit über 5 Sekunden (5s = 25 * 200ms)
            int16_t avgSpeedR = totalDeltaR / 25;
            int16_t avgSpeedL = totalDeltaL / 25;
            int16_t avgSpeedDiff = avgSpeedR - avgSpeedL;
            
            // Speichere Ergebnis (unabhängig von der Differenz, da bereits kontinuierlich korrigiert wurde)
            results[pwmIndex].pwmValue = basePWM;
            results[pwmIndex].pwmLeft = pwmLeft;
            results[pwmIndex].pwmRight = pwmRight;
            results[pwmIndex].speedDiff = avgSpeedDiff; // Durchschnittliche Differenz über 5s
            results[pwmIndex].iterations = iteration;
            
            communication_log(LEVEL_INFO, "PWM %" PRId16 ": Endgeschwindigkeit geprüft (5s): L=%" PRId16 " R=%" PRId16 " Ticks, Diff=%" PRId16 " (PWM L=%" PRId16 " R=%" PRId16 ")", 
                             basePWM, totalDeltaL, totalDeltaR, totalDiff, pwmLeft, pwmRight);
            
            // Nächster PWM-Wert (ohne zu stoppen - smooth erhöhen)
            pwmIndex++;
            iteration = 0;
            speedMeasureInitialized = 0; // Reset für nächsten PWM-Wert
            endSpeedMeasureInitialized = 0; // Reset für nächste 5s-Überprüfung
            stableDiffStarted = 0; // Reset Timer
            
            if (pwmIndex >= 10) {
                // Alle PWM-Werte kalibriert - zeige Übersicht
                Motor_stopAll();
                phase = 3;
                timeTask_getTimestamp(&waitStart);
            } else {
                // Smooth PWM-Erhöhung zum nächsten Wert (Motoren laufen weiter, kein Soft-Start)
                int16_t oldBasePWM = basePWM;
                basePWM = pwmValues[pwmIndex];
                // Rechter Motor: Immer genau basePWM (smooth erhöhen)
                pwmRight = basePWM;
                // Skaliere linken PWM-Wert proportional zum neuen Basis-Wert
                pwmLeft = (int32_t)pwmLeft * basePWM / oldBasePWM;
                // Setze neue PWM-Werte direkt (Motoren laufen weiter, keine Unterbrechung)
                Motor_setPWM(pwmLeft, pwmRight);
                // Warte kurz auf Betriebsdrehzahl (PWM-abhängig)
                int16_t pwmDiff = (basePWM > minPWMForCalc) ? (basePWM - minPWMForCalc) : 0;
                uint32_t requiredWaitTime = baseSpeedWaitTime + (pwmDiff * timePerPWM);
                timeTask_getTimestamp(&waitStart);
                // Gehe direkt zu Phase 1 (Geschwindigkeitssynchronisation), kein Soft-Start
                phase = 1; // Direkt zu Geschwindigkeitssynchronisation
                communication_log(LEVEL_INFO, "PWM-Wert %d/%d: %" PRId16 " (Smooth erhöht, Start: L=%" PRId16 " R=%" PRId16 ")", 
                                 pwmIndex + 1, 10, basePWM, pwmLeft, pwmRight);
            }
        }
    } else if (phase == 3) {
        // Phase 3: Zeige Übersicht
        timeTask_time_t now;
        timeTask_getTimestamp(&now);
        if (timeTask_getDuration(&waitStart, &now) >= 1000000UL) { // 1 Sekunde warten
            communication_log(LEVEL_INFO, "");
            communication_log(LEVEL_INFO, "=== Kalibrierungs-Übersicht ===");
            communication_log(LEVEL_INFO, "PWM    | PWM L   | PWM R   | Speed Diff | Iter | L/R %% | PWM Diff");
            communication_log(LEVEL_INFO, "-------+---------+---------+------------+------+-------+----------");
            
            int16_t bestSpeedDiff = 32767;
            uint8_t bestIndex = 0;
            
            for (uint8_t i = 0; i < 10; i++) {
                int16_t absSpeedDiff = (results[i].speedDiff > 0) ? results[i].speedDiff : -results[i].speedDiff;
                
                // Berechne prozentualen Unterschied: (R - L) / R * 100
                // Verwende R als Referenz (da R = basePWM)
                int16_t percentDiff = 0;
                if (results[i].pwmRight != 0) {
                    // Berechne in Hundertsteln: ((pwmLeft - pwmRight) * 10000) / pwmRight
                    int16_t pwmDiff = results[i].pwmLeft - results[i].pwmRight;
                    percentDiff = ((int32_t)pwmDiff * 10000) / results[i].pwmRight;
                }
                
                // Formatierung: Zeige Vorzeichen, ganze Zahl und 2 Dezimalstellen
                int16_t percentWhole = percentDiff / 100;
                int16_t percentFrac = (percentDiff > 0) ? (percentDiff % 100) : (-percentDiff % 100);
                
                // Berechne PWM-Differenz: PWM L - PWM R
                int16_t pwmDiff = results[i].pwmLeft - results[i].pwmRight;
                
                communication_log(LEVEL_INFO, "%5" PRId16 " | %7" PRId16 " | %7" PRId16 " | %10" PRId16 " | %4d | %+3d.%02d%% | %9" PRId16, 
                                 results[i].pwmValue, results[i].pwmLeft, results[i].pwmRight, 
                                 results[i].speedDiff, results[i].iterations, percentWhole, percentFrac, pwmDiff);
                
                // Finde beste (kleinste) Geschwindigkeitsdifferenz
                if (absSpeedDiff < bestSpeedDiff) {
                    bestSpeedDiff = absSpeedDiff;
                    bestIndex = i;
                }
            }
            
            communication_log(LEVEL_INFO, "-------+---------+---------+------------+------+-------+----------");
            communication_log(LEVEL_INFO, "Bester PWM-Wert: %" PRId16 " (Speed Diff: %" PRId16 ", %d Iterationen)", 
                             results[bestIndex].pwmValue, results[bestIndex].speedDiff, results[bestIndex].iterations);
            
            // Speichere alle Werte in EEPROM
            savePWMCalibration(results);
            communication_log(LEVEL_INFO, "Alle PWM-Werte in EEPROM gespeichert!");
            communication_log(LEVEL_INFO, "Bester PWM-Wert: %" PRId16 " (L=%" PRId16 " R=%" PRId16 ")", 
                             results[bestIndex].pwmValue, results[bestIndex].pwmLeft, results[bestIndex].pwmRight);
            communication_log(LEVEL_INFO, "=== Kalibrierung abgeschlossen ===");
            
            setState(IDLE);
            initialized = 0;
        }
    }
}
