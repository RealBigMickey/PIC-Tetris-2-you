#include <xc.h>
#include <stdint.h>
#include "helpers.h"
#define CS_PIN LATDbits.LATD0   
#define CS_TRIS TRISDbits.TRISD0 

#define DIN_PIN LATCbits.LATC5   
#define DIN_TRIS TRISCbits.TRISC5 

#define CLK_PIN LATCbits.LATC3   
#define CLK_TRIS TRISCbits.TRISC3 

// MAX7219 Registers
#define REG_NO_OP 0x00
#define REG_DECODE 0x09
#define REG_INTENSITY 0x0A
#define REG_SCANLIMIT 0x0B
#define REG_SHUTDOWN 0x0C
#define REG_DISPLAYTEST 0x0F

// Number of matrices daisy-chained
#define NUM_DEVICES 2

void MCU_Init(void) {
    OSCCON = 0x72;  // 8MHz Internal Oscillator
    ADCON1 = 0x0F;  // Digital I/O (All pins)
    
    CS_PIN = 1;     // CS Idle High
    DIN_PIN = 0;
    CLK_PIN = 0;    // CLK Idle Low
    
    CS_TRIS = 0;
    DIN_TRIS = 0;
    CLK_TRIS = 0;
}


void SPI_Write_Byte(uint8_t data) {
    for(int i = 7; i >= 0; i--) {
        CLK_PIN = 0; 
        
        // Set Data
        DIN_PIN = (data >> i) & 1;
        
        // Pulse Clock (Rising Edge latches data)
        CLK_PIN = 1;
        // Small delay to ensure pulse width
        asm("NOP");
        asm("NOP");
        CLK_PIN = 0; 
    }
}

// Write to all daisy-chained dot matrices
void MAX7219_Write_All(uint8_t reg, uint8_t data) {
    CS_PIN = 0; // Start Load

    for(int i = 0; i < NUM_DEVICES; i++) {
        SPI_Write_Byte(reg);
        SPI_Write_Byte(data);
    }

    CS_PIN = 1; // Latch Data
    asm("NOP");
    asm("NOP");
}

void MAX7219_Init(void) {
    MAX7219_Write_All(REG_DISPLAYTEST, 0x00);   // Normal Operation
    MAX7219_Write_All(REG_SCANLIMIT, 0x07);     // Scan all 8 digits
    MAX7219_Write_All(REG_DECODE, 0x00);        // Matrix Mode (No Decode)
    MAX7219_Write_All(REG_INTENSITY, 0x02);     // Low Brightness
    MAX7219_Write_All(REG_SHUTDOWN, 0x01);      // Wake Up
    
    // clear them beforehand
    for (int i = 1; i <= 8; i++) {
        MAX7219_Write_All(i, 0x00);
    }
}


/* Kinda messed up the wiring, with both screens being swapped on accident.
 * Thus we write rows 8~15 first then 0~7 */
void update_display(void) {
    for (int i = 0; i < 8; i++) {
        uint8_t pattern_top = 0;
        for (int col = 0; col < 8; col++) {
            if (display_matrix[i + 8][col]) {
                pattern_top |= (1u << (7 - col));
            }
        }

        uint8_t pattern_bot = 0;
        for (int col = 0; col < 8; col++) {
            if (display_matrix[i][col]) {
                pattern_bot |= (1u << (7 - col));
            }
        }

        // SPI Transmission
        CS_PIN = 0;

        SPI_Write_Byte(i + 1);       
        SPI_Write_Byte(pattern_bot); 

        SPI_Write_Byte(i + 1);       
        SPI_Write_Byte(pattern_top); 

        CS_PIN = 1; // Latch both
    }
}


