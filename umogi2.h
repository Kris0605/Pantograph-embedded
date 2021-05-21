// umogi2.h

#ifndef UMOGI_H
#define UMOGI_H

#include <xc.h>   
#define SYS_FREQ 32000000L
#define FCY SYS_FREQ/2 

#include <stdlib.h> // malloc
#include <stdio.h> // sprintf
#include <string.h> // char *
#include <libpic30.h>

#define BYTE unsigned char
#define BOOL unsigned char
#define FALSE 0
#define TRUE 1

#define LED1 LATGbits.LATG6
#define LED2 LATGbits.LATG7
#define LED3 LATGbits.LATG8
#define LED4 LATGbits.LATG9
#define LEDR LATDbits.LATD15
#define LEDG LATFbits.LATF4
#define LEDB LATFbits.LATF5

#define CAP LATBbits.LATB0

#define SW1 !PORTCbits.RC1
#define SW2 !PORTCbits.RC3
#define SW3 !PORTEbits.RE8
#define SW4 !PORTEbits.RE9

#define L293_ENA LATBbits.LATB14
#define L293_ENB LATBbits.LATB15
#define L293_1 LATBbits.LATB8
#define L293_2 LATBbits.LATB9
#define L293_3 LATBbits.LATB12
#define L293_4 LATBbits.LATB13

 // LCD data: RE0..RE7
    // LCD RS: RG0
    // LCD RW: RG1
    // LCD E: RF1
#define lcd_cpl 16
#define LCD_DATA LATE
#define LCD_RS LATGbits.LATG0
#define LCD_RW LATGbits.LATG1
#define LCD_E LATFbits.LATF1
#define LCD_BF PORTEbits.RE7

// GPIO 
#define GPIO_1 LATDbits.LATD0
#define GPIO_2 LATDbits.LATD11
#define GPIO_3 LATDbits.LATD10
#define GPIO_4 LATDbits.LATD9
#define GPIO_5 LATDbits.LATD8
#define GPIO_6 LATDbits.LATD14
// an3=rb3
#define GPIO_7 LATBbits.LATB11
// AN11=rb11
#define GPIO_8 LATBbits.LATB3

//I2C 
#define I2C_ACK 0 
#define I2C_NAK 1
#define I2C_WADDR(x) (x & 0xFE) //törli az R/W bitet az I2C címbájtban
#define I2C_RADDR(x) (x | 0x01) //'1'-be állítja az R/W bitet az I2C címbájtban
#define lm75_addr 0x90 // 0b1001 0000 LM75 címe 8biten

typedef struct {
    float x;
    float y;
} position;

extern position effector_position;
extern position ball_position;

extern char lcd[lcd_cpl*2+1];

extern BYTE SimSelect;
extern int enc1_counter, enc2_counter;
extern int PointH, PointV;;

//I2C configurálása 
//Fscl 100 vagy 400 [kHz]] 
void configI2C2(int Fscl);
//I2C start müvelet 
void startI2C2(void);
//I2C restart müvelet 
void rstartI2C2(void);
//I2C stop müvelet 
void stopI2C2(void);
//1 bájt küldése 
BYTE putI2C2(int val);
//1 bájt olvasása 
char getI2C2(char ack);
//1 bájt küldése addr címü eszköznek 
void write1I2C2(char addr,char d1);
//2 bájt küldése addr címü eszköznek 
void write2I2C2(char addr,char d1, char d2);
//1 bájt olvasása addr címü eszközröl 
void read1I2C2(char addr,char* pd1);
//2 bájt olvasása addr címü eszközröl 
void read2I2C2(char addr,char* d1, char* d2);
//LM75A hömérséklet lekérdezése 
float ReadTempLM75(void);
// lcd
void lcd_init();
void lcd_update();
void lcd_cls();
void lcd_cgram(void);

// spi
#define CSEE LATDbits.LATD7
//  25LC256 SPI EEPROM parancsai
#define SEE_WRSR    1 // státusz regiszter írása
#define SEE_WRITE   2 // írás parancs
#define SEE_READ    3 // olvasás parancs
#define SEE_WDIS     4 // írás tiltása parancs
#define SEE_RDSR    5 // státusz regiszter olvasása 
#define SEE_WENA     6 // írás engedélyezése parancs

//1 bájt küldése és fogadása spi-n
int WriteSPI1(int i);
// státusz regiszter olvasása 
BYTE ReadSR_25xx256(void);
//Írás engedélyezése 
void WriteEnable_25xx256(void);
// disable write, protect eeprom
void WriteDisable_25xx256(void) ;
// 16 bites cím tartalmának olvasása 
BYTE ReadEE_25xx256(int address);
// block read
void ReadEEBlock_25xx256(int address,BYTE *t,int darab);
//Adat írása a 16 bites címre 
void WriteEE_25xx256(int address, BYTE data) ;

#endif