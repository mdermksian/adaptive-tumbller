/*******************************************************************
 * Program:     EncoderInterface
 * Device:      PIC16F15313
 * Created By:  Sam Bednarski
 * Created On:  11/13/2020
 * Revised By:  N/A
 * Revised On:  N/A
 * 
 * This program interfaces quadrature encoder counting over I2C.
 * Master reads in 4-byte signed integer, LS byte first.
 ********************************************************************/

/**  Header Files **************************************************/
#include <xc.h>
#include <pic16f15313.h>

/** Configuration Bits *********************************************/
#pragma config FEXTOSC = OFF    // no external oscillator
#pragma config RSTOSC = HFINT1  // high frequency internal oscillator
#pragma config WDTE = OFF       // watchdog timer disabled
#pragma config LVP = OFF        // low voltage programming disabled
#pragma config BOREN = OFF      // brown-out reset disabled

/** Local Function Prototypes **************************************/
void __interrupt() isr(void);

/** Constants ******************************************/
#define WHEEL 1 // Left 0, Right 1

#if WHEEL
#define DIR 0   // wheel direction (fwd)
#else
#define DIR 1   // wheel direction (rev)
#endif

#if WHEEL
#define PIC_I2C_ADDR 0x09   // PIC I2C address (right wheel)
#else
#define PIC_I2C_ADDR 0x08   // PIC I2C address (left wheel)
#endif

/** Global Variables ***********************************************/
volatile long count = 0L;   // absolute position counter (reset 0)
volatile long count_copy;   // counter copy for transmission
volatile char old_enc;      // previous encoder state

// Increment selection matrix [old][new] (definition depends on DIR)
#if DIR
const signed char inc_mat[4][4] = {{ 0,  1, -1,  0},
                                   {-1,  0,  0,  1},
                                   { 1,  0,  0, -1},
                                   { 0, -1,  1,  0}};   // flipped / reversed
#else
const signed char inc_mat[4][4] = {{ 0, -1,  1,  0},
                                   { 1,  0,  0, -1},
                                   {-1,  0,  0,  1},
                                   { 0,  1, -1,  0}};   // normal / forwards
#endif

/*******************************************************************
 * Function:        void main(void)
 ********************************************************************/
int main(void) {
    // Oscillator Setup
    OSCCON1 = 0b1100000;    // HFINTOSC, no divisor
    OSCFRQ = 0b011;         // 8 MHz
    
    // Pin IO Setup
    TRISA = 0b110111;   // set all input
    ANSELA = 0b000001;  // set all pins to digital (except RA0 - unused)
    SLRCONA = 0b000000; // set all slews to maximum rate
    ODCONA = 0b000110;  // configure RA1/SCL & RA2/SDA as open-drain
    
    // Peripheral Pin Select (PPS) Setup
    SSP1CLKPPS = 0b00001;   // Map SCL to RA1
    SSP1DATPPS = 0b00010;   // Map SDA to RA2
    RA1PPS = 0x15;          // Map RA1 to SCL
    RA2PPS = 0x16;          // Map RA2 to SDA
    
    // Interrupt-On-Change (IOC) Setup
    IOCAP = 0b110000;   // RA4 & RA5 positive edge enabled
    IOCAN = 0b110000;   // RA4 & RA5 negative edge enabled
    
    // I2C Setup
    SSP1STAT = 0b11000000;          // slew rate control off, SMBus disabled
    SSP1CON1 = 0b00110110;          // enable MSSP, enable clock, I2C slave, 7-bit address
    SSP1CON3 = 0b00000010;          // address hold enabled
    SSP1ADD = PIC_I2C_ADDR << 1;    // set 7-bit address
    
    // Initialize encoder state
    old_enc = PORTA >> 4;
    
    // Interrupt Setup
    PIE0bits.IOCIE = 1;     // enable IOC general interrupt
    PIE3bits.SSP1IE = 1;    // enable MSSP interrupt
    INTCONbits.PEIE = 1;    // enable peripheral interrupts
    INTCONbits.GIE = 1;     // enable global interrupts
    
    // Infinite loop
    while(1);   // do nothing
}

/*****************************************************************
 * Function:        void isr(void)
 ******************************************************************/
void __interrupt() isr(void) {
    char byte;  // temporary byte, multiple uses
    
    // Encoder update
    if(PIR0bits.IOCIF) {
        byte = old_enc ^ (IOCAF >> 4);  // determine new encoder reading
        
        IOCAF = 0;          // clear individual flags
        PIR0bits.IOCIF = 0; // clear global flag
        
        count += (long) inc_mat[old_enc][byte]; // increment counter
        old_enc = byte;                         // update encoder state
    }
    
    // I2C request
    else if(PIR3bits.SSP1IF) {
        PIR3bits.SSP1IF = 0;    // clear flag
        
        // Address byte received
        if(!SSP1STATbits.DA) {
            // Holding address, master awaiting ACK
            if(SSP1CON3bits.ACKTIM) {
                byte = SSP1BUF; // flush buffer, clear flag
                
                SSP1CON2bits.ACKDT = !SSP1STATbits.RW;  // clear ACK if request is for read
                SSP1CON1bits.CKP = 1;                   // release clock
            }
            
            // ACK sent, master awaiting data
            else {
                count_copy = count;             // copy counter for transmission
                SSP1BUF = count_copy & 0xff;    // load buffer with counter LS byte
                SSP1CON1bits.CKP = 1;           // release clock
                
                count_copy >>= 8;   // shift counter copy
            }
        }
        
        // Continuing transmission, ACK received, master awaiting data
        // NOTE: if more than 4 bytes requested, will send all 0's
        else if(!SSP1CON2bits.ACKSTAT) {
            SSP1BUF = count_copy & 0xff;    // load buffer with counter LS byte
            SSP1CON1bits.CKP = 1;           // release clock

            count_copy >>= 8;   // shift counter copy
        }
    }
}
