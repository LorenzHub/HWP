#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#include <stdint.h>

/**
 * @brief Initialisiert die PWM-Kalibrierung und lädt gespeicherte Werte aus EEPROM
 * 
 * Diese Funktion muss beim Systemstart aufgerufen werden, um die gespeicherten
 * PWM-Kalibrierungswerte aus dem EEPROM zu laden.
 */
void calibration_init(void);

/**
 * @brief Führt einen Schritt der Kalibrierung aus
 * 
 * Diese Funktion implementiert eine State-Machine für die automatische
 * PWM-Kalibrierung. Sie muss regelmäßig aufgerufen werden (z.B. im Hauptloop).
 * Die Kalibrierung läuft über mehrere Phasen:
 * - Phase 0: Soft-Start der Motoren
 * - Phase 1: Warten auf Betriebsdrehzahl
 * - Phase 2: Messung der Encoder-Werte
 * - Phase 3: Auswertung und Anpassung der PWM-Werte
 * - Phase 4: Zusammenfassung und Speicherung
 * 
 * @note Die Funktion setzt den State automatisch auf IDLE zurück, wenn die
 *       Kalibrierung abgeschlossen ist.
 */
void calibration_run(void);

/**
 * @brief Gibt den kalibrierten PWM-Wert für den linken Motor zurück
 * 
 * @param desiredPWM Gewünschter PWM-Wert (wird verwendet um den besten passenden Wert zu finden)
 * @return Kalibrierter PWM-Wert für linken Motor (interpoliert basierend auf desiredPWM)
 */
int16_t calibration_getPWMLeft(int16_t desiredPWM);

/**
 * @brief Gibt den kalibrierten PWM-Wert für den rechten Motor zurück
 * 
 * @param desiredPWM Gewünschter PWM-Wert (wird verwendet um den besten passenden Wert zu finden)
 * @return Kalibrierter PWM-Wert für rechten Motor (interpoliert basierend auf desiredPWM)
 */
int16_t calibration_getPWMRight(int16_t desiredPWM);

#endif /* CALIBRATION_H_ */

