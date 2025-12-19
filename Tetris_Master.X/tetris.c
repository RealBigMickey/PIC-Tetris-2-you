#define _XTAL_FREQ 8000000      // Required for __delay_ms definitions
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#pragma config OSC = INTIO67
#pragma config WDT = OFF        // Watchdog Timer Disabled
#pragma config PWRT = OFF       // Power-up Timer Disabled
#pragma config BOREN = ON       // Brown-out Reset Enabled
#pragma config MCLRE = ON       // MCLR Pin Enabled
#pragma config PBADEN = OFF     // PORTB<4:0> digital on Reset
#pragma config LVP = OFF        // Low-Voltage Programming Disabled
#pragma config CPD = OFF        // Data EEPROM Code Protection Disabled

#include "lib.h"
#include "display.h"
#include "helpers.h"
#include "marquee.h"
#include "alcohol.h"


static bool prev_rb0 = false;
static bool prev_rb1 = false;
static bool prev_rb2 = false;
static bool prev_rb3 = false;
static bool prev_rb4 = false;


static void check_inputs(void) {
    // Rotate right
    if (PORTBbits.RB0 == 1 && prev_rb0 == 0) { // Rising Edge
        __delay_ms(20); // Debounce
        if (PORTBbits.RB0 == 1) {
            rotate_current_block(dir_right);
            if(current_block_is_in_illegal_position()){
                rotate_current_block(dir_left); // Undo
            }
        }
    }
    prev_rb0 = PORTBbits.RB0;

    // Rotate left
    if (PORTBbits.RB1 == 1 && prev_rb1 == 0) {
        if (PORTBbits.RB1 == 1) {
            rotate_current_block(dir_left);
            if(current_block_is_in_illegal_position()){
                rotate_current_block(dir_right); // Undo
            }
        }
    }
    prev_rb1 = PORTBbits.RB1;

    // Move left
    if (PORTBbits.RB2 == 1 && prev_rb2 == 0) {
        __delay_ms(20);
        if (PORTBbits.RB2 == 1) {
            block_location_x++;
            if(current_block_is_in_illegal_position()){
                block_location_x--; // Undo
            }
        }
    }
    prev_rb2 = PORTBbits.RB2;

    // Move right
    if (PORTBbits.RB3 == 1 && prev_rb3 == 0) {
        __delay_ms(20);
        if (PORTBbits.RB3 == 1) {
            block_location_x--;
            if(current_block_is_in_illegal_position()){
                block_location_x++; // Undo
            }
        }
    }
    prev_rb3 = PORTBbits.RB3;
    
    // Hold
    if (PORTBbits.RB4 == 1 && prev_rb4 == 0) {
        __delay_ms(20);
        if (PORTBbits.RB4 == 1) {
            attempt_hold_block();
        }
    }
    prev_rb4 = PORTBbits.RB4;
}



void __interrupt(low_priority) Lo_ISR(void) {
    if (processSerialReceive()) {
        return;
    }
}

void onReadLine(char *line, byte len) {
    // not used :P
}

void onReadChar(char c) {
    if (game_mode == 1) {
        switch (c) {
        case 'A':
            game_mode = 2;
            reset_game();
            compose_frame();
            break;
        case '1':
            generate_garbage(1);
            break;
        case '2':
            generate_garbage(2);
            break;
        case '3':
            generate_garbage(3);
            break;
        case '4':
            generate_garbage(4);
            break;
        }
    }
    
}



void main(void) {
    setIntrnalClock();

    // Setup Pins
    ADCON1 = 0x0F; 
    TRISBbits.TRISB0 = 1; 
    TRISBbits.TRISB1 = 1;
    TRISBbits.TRISB2 = 1;
    TRISBbits.TRISB3 = 1;
    TRISBbits.TRISB4 = 1; // Start / Hold / Reset
    
    MCU_Init();
    MAX7219_Init();
    ADC_Init();
    
    enableInterruptPriorityMode(1);
    enableGlobalInterrupt(1);
    enablePeripheralInterrupt(1);
    
    serialBegin(9600, 0b0);
    serialOnReadLine = onReadLine;
    serialOnReadChar = onReadChar;

    srand(1);   // random seed for blocks

    __delay_ms(1000);

    int master_pos = 0; // Tracks the scroll position (0 to 79)
    game_mode = 0;
    serialPrint("Z");

    while (1) {
        // Mode 0: Marquee
        if (game_mode == 0) {
            for (int t = 0; t < 3; t++) {
                display_marquee_slice(master_pos);
                
                if (!PORTBbits.RB4) {
                    serialPrint("S");
                    game_mode = 1;
                    reset_game();
                    break; 
                }
            }

            // move and sync if still in mode 0
            if (game_mode == 0) {
                master_pos++;
                if (master_pos >= 80) {
                    master_pos = 0;
                    serialPrint("R");
                } else
                    serialPrint("M");
            }
        }
        
        // Mode 1: TETRIS!
        else if (game_mode == 1) {
            for (uint8_t t = 0; t < game_speed_ticks; t++) {
                check_inputs();
                
                compose_frame();
                update_display();
                __delay_ms(BASE_DELAY_MS);
                
                if (game_mode != 1)
                    break;
            }

            // Only step gravity if we are still alive
            if (game_mode == 1)
                game_step();
        }
        
        // Mode 2: Alcohol test mode >:) (Slave plays music as always)
        else if (game_mode == 2) {
            
            ShowAlcoholLevelOnLED(CalculateAlcoholLevel());
            __delay_ms(BASE_DELAY_MS);
           
            if (!PORTBbits.RB4) {
                // Force release
                while(PORTBbits.RB4 == 1);

                reset_game();
                game_mode = 0; // Go back to start screen
                serialPrint("Z");
            }
            __delay_ms(500);
        }
        prev_rb4 = PORTBbits.RB4;
    }
}
