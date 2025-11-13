#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

// Physikalische Kalibrierungswerte
// Gemessen: 2048 Motor-Ticks = 1 Rad-Umdrehung
// 1 Encoder-Tick = 0.0688 mm (berechnet: 141 mm / 2048 Ticks)
// 1 Rad-Umdrehung = 2048 * 0.0688 mm = 140.9 mm (Rad-Umfang)
// Rad-Durchmesser = 140.9 mm / π ≈ 44.9 mm (Radius ≈ 22.5 mm)
// 
// WICHTIG: Die Encoder messen direkt am Motor (Motor-Ticks)
// Bei genau 1 Rad-Umdrehung zählt der Encoder um 2048 Ticks hoch
//
// Drehung auf der Stelle:
// Radabstand = 170 mm (von Mitte zu Mitte)
// Umfang der Drehbewegung = π × 170 mm ≈ 534.07 mm
// Für 90°: Jedes Rad muss π × 170 / 4 ≈ 133.52 mm fahren ≈ 1941 Ticks
// Für 180°: Jedes Rad muss π × 170 / 2 ≈ 267.04 mm fahren ≈ 3882 Ticks

void encoder_init();
int16_t encoder_getCountR();
int16_t encoder_getCountL();

#endif /* ENCODER_H */