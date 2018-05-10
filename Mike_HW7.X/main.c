#include <xc.h>           // processor SFR definitions
#include <sys/attribs.h>  // __ISR macro
#include <stdio.h>        // standard i/o functions
#include "i2c_master_noint.h" // I2C2 init and communication functions
#include "ST7735.h" // TFTLCD functions and character table

// DEVCFG0
#pragma config DEBUG = 0b00 // no debugging
#pragma config JTAGEN = 0b0 // no jtag
#pragma config ICESEL = 0b11 // use PGED1 and PGEC1
#pragma config PWP = 0b11111111 // no write protect
#pragma config BWP = 0b1 // no boot write protect
#pragma config CP = 0b1 // no code protect

// DEVCFG1
#pragma config FNOSC = 0b011 // use primary oscillator with pll
#pragma config FSOSCEN = 0b0 // turn off secondary oscillator
#pragma config IESO = 0b0 // no switching clocks
#pragma config POSCMOD = 0b10 // high speed crystal mode
#pragma config OSCIOFNC = 0b1 // disable secondary osc
#pragma config FPBDIV = 0b00 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = 0b10 // do not enable clock switch
#pragma config WDTPS = 0b00000 // use slowest wdt
#pragma config WINDIS = 0b1 // wdt no window mode
#pragma config FWDTEN = 0b0 // wdt disabled
#pragma config FWDTWINSZ = 0b11 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = 0b001 //(2x: 8->4) divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = 0b111 // (24x: 4->96) multiply clock after FPLLIDIV
#pragma config FPLLODIV = 0b001 // (2: 96->48) divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = 0b001 // (2: 8->4, 4*12->48) divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = 0b0 // USB clock on

// DEVCFG3
#pragma config USERID = 0xFFFF // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = 0b0 // allow multiple reconfigurations
#pragma config IOL1WAY = 0b0 // allow multiple reconfigurations
#pragma config FUSBIDIO = 0b1 // USB pins controlled by USB module
#pragma config FVBUSONIO = 0b1 // USB BUSON controlled by USB module

#define SLAVE_ADDR 0b1101011

void drawCross(unsigned short, unsigned short, unsigned short, signed short, signed short, unsigned short, unsigned short, unsigned short);
void I2C_read_multiple(unsigned char, unsigned char, unsigned char *, int);
void initExpander(void);
unsigned char getExpander(char);
void clearLCD(void);
void drawChar(unsigned short,unsigned short,char,unsigned short,unsigned short);
void drawString(unsigned short,unsigned short, char *,unsigned short,unsigned short);
void progBar(unsigned short, unsigned short, unsigned short, unsigned short,unsigned short,unsigned short,unsigned short);


int main() {

    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here
    TRISAbits.TRISA4 = 0;
    TRISBbits.TRISB4 = 1;
    TRISBbits.TRISB8 = 0;
    
    // init I2C2, used as master
    i2c_master_setup();
    
    // init TFTLCD
    LCD_init();
    
    // init IMU
    initExpander();
    
    __builtin_enable_interrupts();
     
    LATAbits.LATA4 = 1;     // Turn on ack check LED
    LATBbits.LATB8 = 0;     // Turn off whoami blink
    char whoami_check = 0;
    char message[30];
    unsigned char data_store[14];
    int i = 0, j = 0, fillX = 0, fillY = 0;
    signed short data_conv[7];
   
    clearLCD();     // Color every pixel on LCD black

    while(1){
        _CP0_SET_COUNT(0);      // set clock to 0 for loop timing
        I2C_read_multiple(SLAVE_ADDR, 0x20, data_store, 14);    // sequentially read data from the IMU
       
        // convert read data to 16 bit signed shorts
        for(i = 0;i<7;i++){
            data_conv[i] = (data_store[i*2+1] << 8) | data_store[i*2];
        }
            
        fillX = (int) (-(data_conv[4]*0.00006104)*50);  // calculate x bar fill
        fillY = (int) ((data_conv[6]*0.00006104)*50);   // calculate y bar fill
        
        drawCross(64,60,2,fillX,fillY,50,RED,CYAN);     // draw the accelerometer bars
        
        whoami_check = getExpander(0x0F);               // check whoami regester to ensure communcation is working
        
        // blinks LED as long as the IMU is still communicating
        if(whoami_check == 0x69){
            LATBINV = 0x100;
        }
        while(_CP0_GET_COUNT() < 24000000/20){;}        // wait a 20th of a second
    }
    
    
    

        
    
    
}

void drawCross(unsigned short x, unsigned short y, unsigned short h, signed short lenx1, signed short leny1, unsigned short len, unsigned short c1, unsigned short c2){
    int realH = (2*h+1), fillH = realH - 2;
    int i = 0, j = 0, k = 0, oS = (realH-1)/2, oS_Fill = oS-1;
    
    // Bar Borders:
    for(i = 0; i<=len; i++){
    LCD_drawPixel(x+oS,y+i,c1);
    LCD_drawPixel(x-oS,y+i,c1);
    LCD_drawPixel(x+i,y-oS,c1);
    LCD_drawPixel(x+i,y+oS,c1);
    LCD_drawPixel(x+oS,y-i,c1);
    LCD_drawPixel(x-oS,y-i,c1);
    LCD_drawPixel(x-i,y-oS,c1);
    LCD_drawPixel(x-i,y+oS,c1);
    if(i==0 | i == len){
        for(j = 0; j < realH; j++){
            LCD_drawPixel(x-oS+j,y+i,c1);
            LCD_drawPixel(x-oS+j,y-i,c1);
            LCD_drawPixel(x+i,y-oS+j,c1);
            LCD_drawPixel(x-i,y-oS+j,c1);
            }
        }
    }
    
    // X Bar Fill:
    if(lenx1>=0){
        for(i = 0; i<len; i++){
            for(j=0;j<fillH;j++){
                if(i<lenx1){
                    LCD_drawPixel(x+i, y-oS_Fill+j,c1); 
                }else{
                    LCD_drawPixel(x+i, y-oS_Fill+j,c2); 
                }
                LCD_drawPixel(x-i,y-oS_Fill+j,c2);
            }
        }
    }else if(lenx1<0) {
        for(i = 0; i>(-len); i--){
            for(j=0;j<fillH;j++){
                if(i>lenx1){
                    LCD_drawPixel(x+i, y-oS_Fill+j,c1);
                }else{
                    LCD_drawPixel(x+i, y-oS_Fill+j,c2);
                }
                LCD_drawPixel(x-i,y-oS_Fill+j,c2);    
            }
        }
    }
    
    // Y Bar Fill:
    if(leny1>=0){
        for(i = 0; i<len; i++){
            for(j=0;j<fillH;j++){
                if(i<leny1){
                    LCD_drawPixel(x-oS_Fill+j, y+i,c1);
                }else{
                    LCD_drawPixel(x-oS_Fill+j, y+i,c2);
                }
                LCD_drawPixel(x-oS_Fill+j,y-i,c2);
            }
        }
    }else{
        for(i = 0; i>(-len); i--){
            for(j=0;j<fillH;j++){
                if(i>leny1){
                    LCD_drawPixel(x-oS_Fill+j, y+i,c1);
                }else{
                    LCD_drawPixel(x-oS_Fill+j, y+i,c2);
                }
                LCD_drawPixel(x-oS_Fill+j,y-i,c2);
            }
        }
    }
}


void I2C_read_multiple(unsigned char address, unsigned char reg_add, unsigned char * data, int length){
    int i = 0;
    i2c_master_start();
    i2c_master_send(address << 1 | 0);
    i2c_master_send(reg_add);
    i2c_master_restart();
    i2c_master_send(address << 1 | 1);
    for(i = 0; i < length; i++){
        data[i] = i2c_master_recv();
        if(i == length-1){
            break;
        }
        i2c_master_ack(0);
    }
    i2c_master_ack(1);
    i2c_master_stop();
    return;
}
    


void initExpander(void) {
    
    
    // Set CTRL1_XL register (accelerometer)
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR << 1 | 0);
    i2c_master_send(0x10);
    // 0b10000010 ([7:4] 1.66kHz samp rate, [3:2] +-2g sens, [1:0] 100 Hz filter)
    i2c_master_send(0x82);
    i2c_master_stop();
    
    // Set CTRL2_G register (gyroscope)
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR << 1 | 0);
    i2c_master_send(0x11);
    // 0b10001000 ([7:4] 1.66kHz samp rate, [3:2] 1000 dps scale, [1:0] 00)
    i2c_master_send(0b10001000);
    i2c_master_stop();
    
    // Set CTRL3_C register (IF_INC = 1, Register Increment)
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR << 1 | 0);
    i2c_master_send(0x12);
    // 0b00000100
    i2c_master_send(0b00000100);
    i2c_master_stop();
}


unsigned char getExpander(char reg_add) {
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR << 1 | 0);
    i2c_master_send(reg_add);
    i2c_master_restart();
    i2c_master_send(SLAVE_ADDR << 1 | 1);
    unsigned char received = i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
    
    return received;
}

// clearLCD: colors entire screen black
void clearLCD(void){
    int i = 0, j = 0;
    for(i = 0; i<128;i++){
        for (j = 0; j<160;j++){
            LCD_drawPixel(i,j,BLACK);
        }   
    }
}

// drawChar: draws a single character at (x,y) w/ foreground color c1 and
//           background color c2
void drawChar(unsigned short x, unsigned short y, char mes, unsigned short c1, unsigned short c2){
    char row = mes - 0x20;         // subtract for missing ascii command characters
    int col = 0;
    for (col = 0; col < 5; col++){
        char pixels = ASCII[row][col];
        int j = 0;
        for(j = 0; j < 8; j++){
            if((pixels>>j)&1==1){
                LCD_drawPixel(x+col,y+j,c1);
            }else{
                LCD_drawPixel(x+col,y+j,c2);
            }
        }
    }
    
}

// drawString: writes a sprintf character array at (x,y) w/ foreground color c1
//             and background color c2
void drawString(unsigned short x, unsigned short y, char *message, unsigned short c1, unsigned short c2){
   int i = 0;
   while (message[i]){
       drawChar(x+6*i,y,message[i],c1,c2);
       i++;
   }
}

// progBar: draws a progress bar at (x,y) w/ height h,
//          fill length len1 (0 to 100), total length len2, 
//          foreground color c1, and background color c2
void progBar(unsigned short x, unsigned short y, unsigned short h, unsigned short len1, unsigned short c1, unsigned short len2, unsigned short c2){
    int i = 0, j = 0, k = 0;
    if(len1 > 100){
        len1 = 100;
    }else if(len1 < 0){
        len1 = 0;
    }
    int fill = (int) (((((float) len1)/100)*len2)-1);
    for(j = 0; j < h; j++){
        LCD_drawPixel(x,y+j,c1);
        LCD_drawPixel(x+len2,y+j,c1);
    }
    for(i = 0; i <= len2; i++){
        LCD_drawPixel(x+i,y,c1);
        LCD_drawPixel(x+i,y+h,c1);
    }
    for(k = 0; k < fill; k++){
        for(j = 0; j < h; j++){
            LCD_drawPixel(x+1+k, y+1+j, c1);
        }
    }
}

    