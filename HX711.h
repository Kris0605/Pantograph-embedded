#include <xc.h>
#include <math.h>

#define    ADSCK1_LAT          LATBbits.LATB0    /* If ADC1 Clock signal is connected to RB0 */
#define    ADSCK1_TRIS         TRISBbits.TRISB0
#define    ADSCK1_SetHigh()    LATBbits.LATB0 = 1
#define    ADSCK1_SetLow()     LATBbits.LATB0 = 0

#define    ADDT1_PORT          PORTBbits.RB1    /* If ADC1 Data Input is connected to RB1 */
#define    ADDT1_SetHigh()     LATBbits.LATB1 = 1
#define    ADDT1_SetLow()      LATBbits.LATB1 = 0
#define    ADDT1_GetValue()    PORTBbits.RB1
#define    ADDT1_TRIS          TRISBbits.TRISB1

#define    ADSCK2_LAT          LATBbits.LATB2    /* If ADC2 Clock signal is connected to RB2 */
#define    ADSCK2_TRIS         TRISBbits.TRISB2
#define    ADSCK2_SetHigh()    LATBbits.LATB2 = 1
#define    ADSCK2_SetLow()     LATBbits.LATB2 = 0

#define    ADDT2_PORT          PORTBbits.RB3    /* If ADC2 Data Input is connected to RB3 */
#define    ADDT2_SetHigh()     LATBbits.LATB3 = 1
#define    ADDT2_SetLow()      LATBbits.LATB3 = 0
#define    ADDT2_GetValue()    PORTBbits.RB3
#define    ADDT2_TRIS          TRISBbits.TRISB3

extern signed long offsetL;
extern float scaleL;
extern signed long offsetR;
extern float scaleR;
extern float valueL;
extern float valueR;
extern float Fx;
extern float Fy;
extern BYTE loadTick;

extern float cosTheta2;
extern float cosTheta3R;
extern float sinTheta3R;
extern float cosTheta3L;
extern float sinTheta3L;
extern float cosTheta4;

void init_LoadCells(void);

signed long read_left(void);
void set_scale_left(float scale);
void tare_left(BYTE times);
void set_offset_left(signed long offset);
signed long get_value_left(void);
float get_value_leftF(void);

signed long read_right(void);
void set_scale_right(float scale);
void tare_right(BYTE times);
void set_offset_right(signed long offset);
signed long get_value_right(void);
float get_value_rightF(void);

void get_values_LoadCell(void);
