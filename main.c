/*
 * K�sz�tette: Riskutia Bal�zs, 2020
 * BME MOGI Tansz�k
 */

#include <xc.h> 
#include <math.h>

#include "umogi2.h"
#include "system.h"

#include "app_device_cdc_basic.h"

#include "usb.h"
#include "usb_device.h"
#include "usb_device_cdc.h"

#include "mechanics.h" // a virtu�lis t�r mechanikai szimul�ci�ja
#include "HX711.h"     //er�m�r�s

// v�ltoz�k a k�t enk�derhez
char enc1_current_state, enc1_prev_state;
char enc2_current_state, enc2_prev_state;
float enc1_radian, enc2_radian;

// v�ltoz�k a k�t motorhoz
float motor1_duty, motor5_duty; // kit�lt�si t�nyez?k


// villog� led a norm�l �zem jelz�s�hez
void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt(void) {
    // timer1: 1 hz.
    LEDB = !LEDB;
    IFS0bits.T1IF = 0;
}

// az enk�derek �llapotainak meghat�roz�sa
void read_encoders_state() {
    if (!PORTDbits.RD8 && !PORTDbits.RD9) enc1_current_state = 0b00;
    if (!PORTDbits.RD8 && PORTDbits.RD9) enc1_current_state = 0b01;
    if (PORTDbits.RD8 && !PORTDbits.RD9) enc1_current_state = 0b10;
    if (PORTDbits.RD8 && PORTDbits.RD9) enc1_current_state = 0b11;

    if (!PORTDbits.RD10 && !PORTDbits.RD11) enc2_current_state = 0b00;
    if (!PORTDbits.RD10 && PORTDbits.RD11) enc2_current_state = 0b01;
    if (PORTDbits.RD10 && !PORTDbits.RD11) enc2_current_state = 0b10;
    if (PORTDbits.RD10 && PORTDbits.RD11) enc2_current_state = 0b11;
}

/*
 * az enk�derek �llapotainak alapj�n a sz�ml�l�k �rt�keinek n�vel�se)
 * - a sz�ml�l�k �rt�keib?l kaphatjuk meg a k�s?bbiekben a relat�v
 *   sz�gelfordul�sokat
 */
void update_encoder_counters() {
    // encoder 1
    if (enc1_prev_state != enc1_current_state) {
        switch (enc1_current_state) {
            case 0b00:
                if (enc1_prev_state == 0b10) enc1_counter++;
                if (enc1_prev_state == 0b01) enc1_counter--;
                break;
            case 0b01:
                if (enc1_prev_state == 0b00) enc1_counter++;
                if (enc1_prev_state == 0b11) enc1_counter--;
                break;
            case 0b11:
                if (enc1_prev_state == 0b01) enc1_counter++;
                if (enc1_prev_state == 0b10) enc1_counter--;
                break;
            case 0b10:
                if (enc1_prev_state == 0b11) enc1_counter++;
                if (enc1_prev_state == 0b00) enc1_counter--;
                break;
        }
    }

    // encoder 2
    if (enc2_prev_state != enc2_current_state) {
        switch (enc2_current_state) {
            case 0b00:
                if (enc2_prev_state == 0b10) enc2_counter++;
                if (enc2_prev_state == 0b01) enc2_counter--;
                break;
            case 0b01:
                if (enc2_prev_state == 0b00) enc2_counter++;
                if (enc2_prev_state == 0b11) enc2_counter--;
                break;
            case 0b11:
                if (enc2_prev_state == 0b01) enc2_counter++;
                if (enc2_prev_state == 0b10) enc2_counter--;
                break;
            case 0b10:
                if (enc2_prev_state == 0b11) enc2_counter++;
                if (enc2_prev_state == 0b00) enc2_counter--;
                break;
        }
    }
}

/*
 * CN interrupt
 * - az enk�derek jelv�ltoz�sakor fut le
 */
void __attribute__((__interrupt__, __auto_psv__)) _CNInterrupt(void) {
    read_encoders_state();
    update_encoder_counters();
    enc1_prev_state = enc1_current_state;
    enc2_prev_state = enc2_current_state;
    IFS1bits.CNIF = 0;
}

// motorok aktiviz�l�sa
void command_motors() {
    /*
     * Motor 1
     */
    // ir�ny meghat�roz�sa
    if (torque_left > 0) {
        LATBbits.LATB12 = 1;
    } else {
        LATBbits.LATB12 = 0;
    }
    // kit�lt�si t�nyez� sz�m�t�sa
    motor5_duty = sqrt(fabsf(torque_left) / 0.561);
    
    // biztos�tjuk, hogy a kit�lt�si t�nyez? 0% �s 100% k�z�tt van
    if (motor5_duty > 1) {
        motor5_duty = 1;
    } else if (motor5_duty < 0) {
        motor5_duty = 0;
    }

    // PWM jel kimenetre kapcsol�sa (Output Compare)
    OC1R = (int) (motor5_duty * 1024);

    /*
     * Motor 2(5)
     */
    // ir�ny meghat�roz�sa
    if (torque_right > 0) {
        LATBbits.LATB13 = 1;
    } else {
        LATBbits.LATB13 = 0;
    }
    // biztos�tjuk, hogy a kit�lt�si t�nyez? 0% �s 100% k�z�tt van
    motor1_duty = sqrt(fabsf(torque_right) / 0.561);

    // biztos�tjuk, hogy a kit�lt�si t�nyez? 0% �s 100% k�z�tt van
    if (motor1_duty > 1) {
        motor1_duty = 1;
    } else if (motor1_duty < 0) {
        motor1_duty = 0;
    }

    // PWM jel kimenetre kapcsol�sa (Output Compare)
    OC2R = (int) (motor1_duty * 1024);
}

MAIN_RETURN main(void) {
    CLKDIVbits.CPDIV = 0; // 32 mhz
    while (!OSCCONbits.LOCK) Nop();
    __builtin_write_OSCCONL(OSCCON | (1 << 1));
    // pps

    // Motor 1 PWM (OC1) -> PWM1 pin
    RPOR4bits.RP8R = 18;

    // Motor 2 PWM (OC2) -> PWM2 pin
    RPOR4bits.RP9R = 19;

    __builtin_write_OSCCONL(OSCCON | (1 << 6)); // iolock=1

    // 1 sec: timer1 
    PR1 = 0x8000; // 32768, 1 sec: 32.768 khz
    IPC0bits.T1IP = 4; // 4-es priorit�s
    IFS0bits.T1IF = 0; // flag t�rl�se
    IEC0bits.T1IE = 1; // interrupt enged�lyez�se
    T1CON = 0x8003; // start, k�ls�

    //timer2
    /*PR2 = 0x4000; // 32768
    IPC1bits.T2IP = 5; // 5-es priorit�s
    IFS0bits.T2IF = 0; // flag t�rl�se
    IEC0bits.T2IE = 1; // interrupt enged�lyez�se
    T2CONbits.TCS = 0; //bels� �rajel
    T2CONbits.TCKPS = 3; //1:256 el�oszt�
    T2CONbits.TON = 0; //start
    TMR2 = 0;*/

    // Motor 1 PWM
    OC1CON1 = 0; // OC modul be�ll�t�sainak t�rl�se
    OC1CON2 = 0;
    OC1CON2bits.SYNCSEL = 0x1F; // szinkroniz�l�s �nmag�val
    OC1CON1bits.OCTSEL = 7; // rendszer �rajel�vel m?k�dik
    OC1R = 0; // kit�lt�s
    OC1RS = 1024; // peri�dus
    OC1CON1bits.OCM = 6; // Edge Aligned PWM mode

    // Motor 2 PWM
    OC2CON1 = 0; // OC modul be�ll�t�sainak t�rl�se
    OC2CON2 = 0;
    OC2CON2bits.SYNCSEL = 0x1F; // szinkroniz�l�s �nmag�val
    OC2CON1bits.OCTSEL = 7; // rendszer �rajel�vel m?k�dik
    OC2R = 0; // kit�lt�s
    OC2RS = 1024; // peri�dus
    OC2CON1bits.OCM = 6; // Edge Aligned PWM mode

    // 100ms: timer3
    PR3 = 6250;
    T3CONbits.TCS = 0;
    T3CONbits.TCKPS = 3; // 256-tal oszt
    TMR3 = 0;

    // led tris
    TRISGbits.TRISG6 = 0;
    TRISGbits.TRISG7 = 0;
    TRISGbits.TRISG8 = 0;
    TRISGbits.TRISG9 = 0;
    TRISDbits.TRISD15 = 0;
    TRISFbits.TRISF4 = 0;
    TRISFbits.TRISF5 = 0;


    // pwm �s dir pinek tris
    TRISBbits.TRISB8 = 0; // IN1: PWM1 pin
    TRISBbits.TRISB9 = 0; // IN2: PWM2 pin
    TRISBbits.TRISB12 = 0; // IN3: DIR1 pin
    TRISBbits.TRISB13 = 0; // IN4: DIR2 pin

    // input change notifications
    TRISDbits.TRISD8 = 1;
    TRISDbits.TRISD9 = 1;
    TRISDbits.TRISD10 = 1;
    TRISDbits.TRISD11 = 1;

    //Switchek
    TRISCbits.TRISC1 = 1;
    TRISCbits.TRISC3 = 1;
    TRISEbits.TRISE8 = 1;
    TRISEbits.TRISE9 = 1;

    // enable
    CNEN4bits.CN53IE = 1;
    CNEN4bits.CN54IE = 1;
    CNEN4bits.CN55IE = 1;
    CNEN4bits.CN56IE = 1;

    enc1_counter = 0;
    enc2_counter = 0;

    // enable interrupts
    IEC1bits.CNIE = 1;
    IFS1bits.CNIF = 1;



    AD1PCFGH = 0b11; // 16,17: disabled
    AD1PCFGL = 0xffef; // 1: analog. 1111 1111 1111 1101
    AD1CON2bits.VCFG = 0; // ref+=VDD, ref-=VSS
    AD1CON3bits.ADRC = 0; // CPU orajel
    AD1CON3bits.ADCS = 1; //Tad= 2 x Tcy= 130ns >76ns
    AD1CON3bits.SAMC = 31; //31Tad
    AD1CON1bits.SSRC = 7; // automatikus konverzio
    AD1CON1bits.ADON = 1; // bekapcs
    AD1CHS0bits.CH0SA = 4; //AN4

    IPC3bits.AD1IP = 4; // 4-es priorit�s
    IFS0bits.AD1IF = 0; // flag t�rl�se

    // LoadCell inicializ�l�sa
    init_LoadCells();

    SYSTEM_Initialize(SYSTEM_STATE_USB_START);

    USBDeviceInit();
    USBDeviceAttach();

    // pwm pinek HIGH-ra
    LATBbits.LATB8 = 1;
    LATBbits.LATB9 = 1;

    // mechanika inicializ�l�sa
    init_mechanics();



    // kijelz�
    lcd_init();
    lcd_cgram();
    sprintf(lcd, "HAPTIC          ");
    sprintf(lcd + lcd_cpl, "PANTOGRAPH      ");
    lcd_update();

    SimSelect = 0;

    while (1) {
        SYSTEM_Tasks();
        /*if (SW1) {
            reset_simulation();
        }
        if (SW2) {
            setTargetPoint(0, 0);
        }
        if (SW3) {
            enc1_counter = 0;
            enc2_counter = 0;
        }*/
        //Application specific tasks

        enc1_radian = 0.001534f * enc1_counter;
        enc2_radian = 0.001534f * enc2_counter;
        fwd_kinematics(enc1_radian, enc2_radian); /*Effector position*/
        if (isEffectorMoving) {
            calculateForces();
            apply_forces_on_effector();
            command_motors();
        } else {
            if (SimSelect != 0) { /*Wait for start the simulation*/
                collision_detection();
                apply_forces();
                command_motors();
                simulate_motion();
            }
        }
        get_values_LoadCell();
        APP_DeviceCDCBasicDemoTasks();

    }//end while
}//end main
