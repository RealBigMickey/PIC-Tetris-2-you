#define _XTAL_FREQ 8000000      // Required for __delay_ms definitions
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#pragma config OSC = INTIO67    // Internal Oscillator
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
#include "music.h"


static bool prev_rb0 = false;
static bool prev_rb1 = false;
static bool prev_rb2 = false;
static bool prev_rb3 = false;
static bool prev_rb4 = false;


static void check_inputs(void) {
    // Rotate right
    if (PORTBbits.RB1 == 1 && prev_rb1 == 0) { // Rising Edge
        __delay_ms(20); // Debounce
        if (PORTBbits.RB1 == 1) {
            rotate_current_block(dir_left);
            if(current_block_is_in_illegal_position()){
                rotate_current_block(dir_right); // Undo
            }
        }
    }
    prev_rb1 = PORTBbits.RB1;

    // Rotate left
    if (PORTBbits.RB0 == 1 && prev_rb0 == 0) {
        if (PORTBbits.RB0 == 1) {
            rotate_current_block(dir_right);
            if(current_block_is_in_illegal_position()){
                rotate_current_block(dir_left); // Undo
            }
        }
    }
    prev_rb0 = PORTBbits.RB0;

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
    if (INTCONbits.TMR0IF) {
        INTCONbits.TMR0IF = 0;
        int current_note_val;
        if (song == 0)
            current_note_val = note[note_index];
        else 
            current_note_val = padoru_note[note_index];

        // -1 is flag for the last note
        if (current_note_val == -1 && note_index > 0) {
            note_index = 0; // Loop back to start
        }

        if (song == 0)
            tone(note[note_index]);
        else
            tone(padoru_note[note_index]);

        int _tmp_interval;
        if (song == 0)
            _tmp_interval = interval[note_index];
        else
            _tmp_interval = padoru_interval[note_index];

        uint32_t counts = (uint32_t)(_tmp_interval) * 1000UL / 128UL;
        uint16_t preload = 65536 - counts;
        TMR0H = preload >> 8;
        TMR0L = preload & 0xFF;

        // Advance Index for NEXT time
        note_index++; 
    }
}


void onReadLine(char *line, byte len) {
//    if (strncmp("something", line, len)) {
//        // do stuff...
//    }
//    if(line[0]=='T'){
//        generate_garbage(1);        
//    }
}


volatile int marquee_offset = 0;
void onReadChar(char c) {
    if (c == 'Z') {
        game_mode = 0;
        marquee_offset = 0;
        reset_game();
        return;
    }
    if (game_mode == 0) {
        switch (c) {
            case 'M':
                marquee_offset++;
                if (marquee_offset >= 80)
                    marquee_offset = 0;
                break;
            case 'R':
                marquee_offset = 0;
                break;
            case 'S':
                game_mode = 1;
                reset_game();
                drop_block_from_sky();
                music_restart();
                break;
        }
    } else if (game_mode == 1) {
        switch (c) {
        case 'A':
            game_mode = 2;
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
    TRISBbits.TRISB4 = 1;
    
    MCU_Init();
    MAX7219_Init();
    
    enableInterruptPriorityMode(1);
    enableGlobalInterrupt(1);
    enablePeripheralInterrupt(1);
    
    serialBegin(9600, 0b0);
    serialOnReadLine = onReadLine;
    serialOnReadChar = onReadChar;

    // Music Init
    Timer2_PWM_Init();
    Timer0_Init();
    music_stop();

    
    srand(2); // Different seed from Master
    
    game_mode = 0;
    
    __delay_ms(1000);
    while (1) {
        // Mode 0: Marquee (follow master)
        if (game_mode == 0) {
            music_stop();
            reset_game();
            while (game_mode == 0) {
                // Slave is physically offset by 12 pixels, looks great on our screen XD
                int my_slice = (marquee_offset + 12) % 80;
                
                display_marquee_slice(my_slice);
            }
        }
        
        // Mode 1: TETRIS!
        else if (game_mode == 1) {
            for (uint8_t t = 0; t < game_speed_ticks; t++) {
                check_inputs();
                
                compose_frame();
                update_display();
                __delay_ms(BASE_DELAY_MS);
                
                if (game_mode != 1) {
                    compose_frame();
                    update_display();
                    break;
                }
            }

            if (game_mode == 1) {
                game_step();
            }
        }
        
        // Mode 2: Master is on alcohol mode, slave just plays song 2 - Padoru
        else if (game_mode == 2) {
            music_stop();
            song = 1;
            music_restart();
            update_display();
            
            // wait for ISR to set game_mode = 0 when 'M' is received
            while(game_mode == 2);
            song = 0;
        }
    }
}