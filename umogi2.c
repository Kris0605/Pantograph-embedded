// umogi2.c

// PIC24FJ256GB108 Configuration Bit Settings

// CONFIG3
#pragma config WPFP = WPFP511           // Write Protection Flash Page Segment Boundary (Highest Page (same as page 170))
#pragma config WPDIS = WPDIS            // Segment Write Protection Disable bit (Segmented code protection disabled)
#pragma config WPCFG = WPCFGDIS         // Configuration Word Code Page Protection Select bit (Last page(at the top of program memory) and Flash configuration words are not protected)
#pragma config WPEND = WPENDMEM         // Segment Write Protection End Page Select bit (Write Protect from WPFP to the last page of memory)

// CONFIG2
#pragma config POSCMOD = HS             // Primary Oscillator Select (HS oscillator mode selected)
#pragma config DISUVREG = OFF            // Internal USB 3.3V Regulator Disable bit (Regulator is disabled)
#pragma config IOL1WAY = ON             // IOLOCK One-Way Set Enable bit (Write RP Registers Once)
#pragma config OSCIOFNC = OFF           // Primary Oscillator Output Function (OSCO functions as CLKO (FOSC/2))
#pragma config FCKSM = CSDCMD           // Clock Switching, Clock Monitor are disabled
#pragma config FNOSC = PRIPLL           // Oscillator Select (Primary oscillator +pll )
#pragma config PLL_96MHZ = ON           // 96MHz PLL Disable (Enabled)
#pragma config PLLDIV = DIV3           // USB 96 MHz PLL Prescaler Select bits (Oscillator input divided by 3 (4MHz input))
#pragma config IESO = OFF               // Internal External Switch Over Mode (IESO mode (Two-speed start-up)disabled)

// CONFIG1
#pragma config WDTPS = PS32768          // Watchdog Timer Postscaler (1:32,768)
#pragma config FWPSA = PR128            // WDT Prescaler (Prescaler ratio of 1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Standard Watchdog Timer enabled,(Windowed-mode is disabled))
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog Timer is disabled)
#pragma config ICS = PGx2               // Comm Channel Select (Emulator functions are shared with PGEC2/PGED2)
#pragma config GWRP = OFF               // General Code Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF                // General Code Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG port is disabled)

#include "umogi2.h"

char lcd[lcd_cpl*2+1];

BYTE SimSelect;
int enc1_counter, enc2_counter;
int PointH, PointV;

void configI2C2(int Fscl) 
{ 
    int BRGRV; 
    BRGRV = ((FCY/1000L)/(Fscl) - FCY/10000000L) - 2; 
    I2C2BRG = BRGRV;          //I2C Baudrate reload ??rt??ke 
    I2C2CONbits.I2CEN = 1;    //I2C ended??lyez??se 
}

void startI2C2(void) 
{ 
  I2C2CONbits.SEN = 1;      //start felt??tel k??ld??se 
  while (I2C2CONbits.SEN);  //start befejez??d??tt 
}

void rstartI2C2(void) 
{ 
  I2C2CONbits.RSEN = 1;     //restart felt??tel k??ld??se 
  while (I2C2CONbits.RSEN); //restart befejez??d??tt 
}

void stopI2C2(void) 
{ 
    I2C2CONbits.PEN=1;        //stop felt??tel k??ld??se 
    while (I2C2CONbits.PEN);  //stop befejez??d??tt 
}

BYTE putI2C2(int val) // out: ACK?
{ 
    I2C2TRN = val;                // b??jt k??ld??se 
    while (I2C2STATbits.TRSTAT);  // (8bit+ack) befejez??d??tt 
    return I2C2STATbits.ACKSTAT;
}
char getI2C2(char ack) 
{ 
    char inByte; 
    while (I2C2CON & 0x1F);         //v??rakoz??s idle ??llapotra 
    I2C2CONbits.RCEN = 1;           //fogad??s enged??lyez??se 
    while (!I2C2STATbits.RBF);      //v??rakoz??s az adat ??rkez??s??re 
    inByte = I2C2RCV;               //1 b??jt beolvas??sa 
    //v??rakoz??s idle ??llapotra, nyugt??z??s k??ld??se el??tt 
    while (I2C2CON & 0x1F);         //v??rakoz??s idle ??llapotra 
    I2C2CONbits.ACKDT = ack;        //ACK bit be??ll??t??sa 
    I2C2CONbits.ACKEN = 1;          //ACK bit visszak??ld??se 
    while (I2C2CONbits.ACKEN);      //v??rakoz??s a befejez??sig 
    return(inByte);                 //visszat??r??s az adattal 
}

void write1I2C2(char addr,char d1) 
{
    startI2C2();
    putI2C2(I2C_WADDR(addr)); 
    putI2C2(d1); 
    stopI2C2(); 
}

void write2I2C2(char addr,char d1, char d2) 
{
    startI2C2(); 
    putI2C2(I2C_WADDR(addr)); 
    putI2C2(d1); 
    putI2C2(d2); 
    stopI2C2(); 
}

void read1I2C2(char addr,char* pd1) 
{ 
    startI2C2(); 
    putI2C2(I2C_RADDR(addr)); 
    *pd1 = getI2C2(I2C_NAK); //az utols?? nyugt??z??s mindig NAK 
    stopI2C2();
}

void read2I2C2(char addr,char* d1, char* d2) 
{ 
    startI2C2(); 
    putI2C2(I2C_RADDR(addr)); 
    *d1 = getI2C2(I2C_ACK); 
    *d2 = getI2C2(I2C_NAK); 
    stopI2C2(); 
}

float ReadTempLM75(void) 
{ 
    char lo,hi; 
    int temp; 
    write1I2C2(lm75_addr, 0x00);  //Temp regiszter 
    read2I2C2(lm75_addr, &hi, &lo);     //2 byte kiolvas??sa 
    temp = hi << 8; 
    temp = (temp + lo) >> 5;            //11bit 
    if (temp > 1024) return (float)(~temp + 1)*-0.125; 
    else return (float)temp*0.125; 
}

int WriteSPI1(int i) 
{   
    SPI1BUF = i; // buffer ??r??sa k??ld??sre 
    while(!SPI1STATbits.SPIRBF) Nop(); // v??rakoz??s az ??tvitel befejez??s??ig 
    return SPI1BUF;    // be??rkez?? adat kiolvas??sa 
}

BYTE ReadSR_25xx256(void) 
{
   BYTE i; 
   CSEE = 0; // EEPROM kiv??laszt??sa 
   WriteSPI1(SEE_RDSR);  // st??tusz regiszter olvas??sa
   i = WriteSPI1(0); // k??ld??s/fogad??s 
   CSEE = 1; // EEPROM elenged??se 
   return i; 
}

void WriteEnable_25xx256(void) 
{ 
   CSEE = 0; // EEPROM kiv??laszt??sa 
   WriteSPI1(SEE_WENA); // ??r??s enged??lyez??se parancs
   CSEE=1; // EEPROM elenged??se
   Nop();Nop();
   CSEE=0;
   WriteSPI1(SEE_WRSR); // write stat reg
   WriteSPI1(0x00); // wpen 0, wp0, wp1 0
   CSEE = 1; // ett??l resetel??dik az ??r??si enged??ly
   while (ReadSR_25xx256()  & 0x01) Nop();
   CSEE = 0;  WriteSPI1(SEE_WENA);  CSEE=1;
}

void WriteDisable_25xx256(void) 
{ 
   CSEE = 0; // EEPROM kiv??laszt??sa 
   WriteSPI1(SEE_WENA); // ??r??s enged??lyez??se parancs
   CSEE=1; // EEPROM elenged??se
   Nop();Nop();
   CSEE=0;
   WriteSPI1(SEE_WRSR); // write stat reg
   WriteSPI1(0xff); // wpen 0, wp0, wp1 0
   CSEE = 1; // ett??l resetel??dik az ??r??si enged??ly
   while (ReadSR_25xx256()  & 0x01) Nop();
}

BYTE ReadEE_25xx256(int address) 
{ 
    BYTE val; 
    CSEE = 0; // EEPROM kiv??laszt??sa 
    WriteSPI1(SEE_READ); // olvas??s parancs 
    WriteSPI1(address>>8); // a c??m fels?? r??sze (MSB) 
    WriteSPI1(address & 0x00ff); // a c??m als?? r??sze (LSB) 
    val = WriteSPI1(0);     // dummy ??rt??k k??ld??se/??rt??k beolvas??sa 
    CSEE = 1;               // EEPROM elenged??se 
    return val;
}

void ReadEEBlock_25xx256(int address,BYTE *t,int darab)
{
    int i;
    CSEE=0;
    WriteSPI1(SEE_READ); // olvas??s parancs 
    WriteSPI1(address>>8); // a c??m fels?? r??sze (MSB) 
    WriteSPI1(address & 0x00ff); // a c??m als?? r??sze (LSB) 
    for (i=0;i<darab;i++) t[i]=WriteSPI1(0);
    CSEE=1;
}

void WriteEE_25xx256(int address, BYTE data) 
{ 
    WriteEnable_25xx256(); //??r??s enged??lyez??se
    CSEE = 0; // EEPROM kiv??laszt??sa 
    WriteSPI1(SEE_WRITE); // ??r??s parancs 
    WriteSPI1(address>>8); // a c??m fels?? r??sze (MSB) 
    WriteSPI1(address & 0x00ff); // a c??m als?? r??sze (LSB) 
    WriteSPI1(data); // az adat k??ld??se 
    CSEE = 1; // EEPROM elenged??se
    while (ReadSR_25xx256()  & 0x01) Nop(); // ??r??s folyamat??nak v??ge (WIP) 
    WriteDisable_25xx256();
}

void lcd_data(BYTE b,BYTE rs)
{ // rs: 0:inst, 1:data 
   BYTE bf;
   int trise_ment=TRISE;
   LCD_RS=rs;
   LCD_DATA=(LCD_DATA & 0xff00) | b;
   Nop(); Nop(); Nop(); // >230 ns. 62.5 ns/utasit??s
   LCD_E=1;// +1 utasit??s
   Nop(); Nop(); Nop();
   LCD_E=0;
   // bf check
   Nop(); Nop(); Nop();
   LCD_DATA &= 0xff00;
   TRISE |= 0x00ff; // lcd input
   LCD_RW=1; // r/w is 1
   LCD_RS=0;
   Nop(); Nop(); Nop();
   do
   {
       LCD_E=1;
       Nop(); Nop(); Nop();
       bf=LCD_BF;
       LCD_E=0;
       Nop(); Nop(); Nop();
   } while (bf);
   TRISE=trise_ment; 
   LCD_RW=0;
}

void lcd_init()
{
  // hitachi HD44780
  // tris
  TRISE &= 0xff00; // upper: don't change, lower 8: data output
  TRISGbits.TRISG0=0; // rs
  TRISGbits.TRISG1=0; // rw
  TRISFbits.TRISF1=0; // e
  LCD_RW=0;
  __delay_ms(50); // more than 40 msec
  // 8 bit: bf can be checked
  lcd_data(0b00111000,0); // 0x38 N=1 (2 lines), 5x8 font, charset 0 - ft1:0=00 
  lcd_data(0x0e,0); // 0b00001110: disp on, curs on, blink off 
  lcd_data(0x01,0); // disp clear
  lcd_data(0x02,0); // return home
  lcd_data(0x06,0); // entry mode set 
  lcd_cls();
}

void lcd_update()
{
    int i;
    lcd_data(0x80,0); // line 0
    for (i=0; i<lcd_cpl; i++)
        lcd_data(lcd[i],1); // lcd write
    lcd_data(0xc0,0); // line 1
    for (i=0; i<lcd_cpl; i++)
        lcd_data(lcd[lcd_cpl+i],1);
}

void lcd_cls()
{
    memset(lcd,' ',2*lcd_cpl);   
}

void lcd_cgram(void) 
{
  lcd_data(0x40,0); // set cgram, and define char 0
  lcd_data(0,1); // 0 8x5
  lcd_data(0x0a,1); // 1
  lcd_data(0x15,1); // 2 
  lcd_data(0x11,1); // 3 
  lcd_data(0x0a,1); // 4 
  lcd_data(0x04,1); // 5 
  lcd_data(0,1); // 6 
  lcd_data(0,1); // 7 
  // char 1 ...
}
