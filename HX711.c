/*
 * File:   HX711.c
 * Author: Honti
 *
 * Created on 2021. március 3., 13:54
 */


#include "xc.h"
#include "umogi2.h"  
#include "mechanics.h"
#include "HX711.h"

signed long offsetL;
float scaleL;
signed long offsetR;
float scaleR;
float valueL;
float valueR;
float Fx;
float Fy;

signed long readCount;
BYTE i;
BYTE loadTick;
int32_t hx711_min;
int32_t hx711_max;
float output_min;
float output_max;
float slope;

float cosTheta2, cosTheta3R, sinTheta3R, cosTheta3L, sinTheta3L, cosTheta4;

float v4y, v3x, v3y, v2x, v2y;
float F3, F2;

void init_LoadCells(void) {
    ADDT1_TRIS = 1; /* Data line is Input to PIC */
    ADSCK1_TRIS = 0; /* Clock line to Output from PIC */

    ADDT2_TRIS = 1; /* Data line is Input to PIC */
    ADSCK2_TRIS = 0; /* Clock line to Output from PIC */

    set_scale_left(-1); //jobbra legyen a pozitív elõjel
    set_scale_right(-1); //jobbra legyen a pozitív elõjel
    tare_left(10);
    tare_right(10);

    hx711_min = 0xFF800000; //hx711 min
    hx711_max = 0x7FFFFF; //hx711 max
    output_min = -10000; //tartomány alja
    output_max = 10000; //tartomány teteje
    slope = (float) (output_max - output_min) / (hx711_max - hx711_min); //meredekség
    valueL = 0.0f;
    valueR = 0.0f;
    Fx = 0.0f;
    Fy = 0.0f;
    loadTick = 0;

    cosTheta2 = 0.0f;
    cosTheta3R = 0.0f;
    sinTheta3R = 0.0f;
    cosTheta3L = 0.0f;
    sinTheta3L = 0.0f;
    cosTheta4 = 0.0f;
}

void get_values_LoadCell(void) {
    if (loadTick % 12 == 1) {
        valueL = get_value_leftF();
        valueR = get_value_rightF();

        v4y = -a5 - x4; //és könnyebb legyen vele számolni majd
        v3x = x3 - x4;
        v3y = y3 - y4;
        cosTheta4 = (y4 * v3x + v4y * v3y) / (sqrt(y4 * y4 + v4y * v4y) * sqrt(v3x * v3x + v3y * v3y));

        cosTheta3L = v3y / (sqrt(v3x * v3x + v3y * v3y));
        sinTheta3L = sqrt(1 - cosTheta3L * cosTheta3L);

        v2x = x3 - x2;
        v2y = y3 - y2;
        cosTheta2 = (-y2 * v2x + x2 * v2y) / (sqrt(y2 * y2 + x2 * x2) * sqrt(v2x * v2x + v2y * v2y));

        cosTheta3R = v2y / (sqrt(v2x * v2x + v2y * v2y));
        sinTheta3R = sqrt(1 - cosTheta3R * cosTheta3R);

        F3 = valueL / cosTheta4;
        F2 = -valueR / cosTheta2;

        Fx = F3 * sinTheta3L - F2*sinTheta3R;
        Fy = F3 * cosTheta3L + F2*cosTheta3R;
    }
}

signed long read_left(void) {
    ADSCK1_LAT = 0; /* Konverzió indítása */
    readCount = 0;
    while (ADDT1_GetValue()); /* Ha még fut konverzió, várjuk meg míg vége */
    for (i = 0; i < 24; i++) {
        ADSCK1_SetHigh();
        readCount = readCount << 1; /* Balra tolás */
        ADSCK1_SetLow();
        readCount = readCount | ADDT1_GetValue(); /* Beolvassuk a következõ bit-et */
    }
    for (i = 0; i < 1; i++) {/* Erõsítés 1:128, 2:32, 3:64 */
        ADSCK1_SetHigh();
        Nop();
        ADSCK1_SetLow();
        Nop();
    }
    return readCount;
}

signed long read_right(void) {
    ADSCK2_LAT = 0; /* Konverzió indítása */
    readCount = 0;
    while (ADDT2_GetValue()); /* Ha még fut konverzió, várjuk meg míg vége */
    for (i = 0; i < 24; i++) {
        ADSCK2_SetHigh();
        readCount = readCount << 1; /* Balra tolás */
        ADSCK2_SetLow();
        readCount = readCount | ADDT2_GetValue(); /* Beolvassuk a következõ bit-et */
    }
    for (i = 0; i < 1; i++) { /* Erõsítés 1:128, 2:32, 3:64 */
        ADSCK2_SetHigh();
        Nop();
        ADSCK2_SetLow();
        Nop();
    }

    return readCount;
}

signed long read_average_left(BYTE times) {
    signed long sum = 0;
    BYTE i;
    for (i = 0; i < times; i++) {
        sum += read_left();
    }
    return sum / times;
}

signed long read_average_right(BYTE times) {
    signed long sum = 0;
    BYTE i;
    for (i = 0; i < times; i++) {
        sum += read_right();
    }
    return sum / times;
}

void set_scale_left(float scale) {
    scaleL = scale;
}

void set_scale_right(float scale) {
    scaleR = scale;
}

void set_offset_left(signed long offset) {
    offsetL = offset;
}

void set_offset_right(signed long offset) {
    offsetR = offset;
}

void tare_left(BYTE times) {
    signed long sum = read_average_left(times);
    set_offset_left(sum);
}

void tare_right(BYTE times) {
    signed long sum = read_average_right(times);
    set_offset_right(sum);
}

signed long get_value_left(void) {
    return read_left() - offsetL;
}

float get_value_leftF(void) {
    return (output_min + slope * (get_value_left() - hx711_min)) * scaleL;
}

signed long get_value_right(void) {
    return read_right() - offsetR;
}

float get_value_rightF(void) {
    return (output_min + slope * (get_value_right() - hx711_min)) * scaleR;
}




