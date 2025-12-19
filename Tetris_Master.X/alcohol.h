#pragma once
#include<pic18f4520.h>
#include<stdint.h>
#include<xc.h>
#define SAMPLE_COUNT_THRESHOLD 2

volatile long long alcohol_baseline_fp = 280*1000; // fixed point scaled 1000
volatile long alcohol_sample_count = 0;
volatile char is_get_first_sample = 0;

long ADC_Read() {
    ADCON0bits.GO = 1;
    while(ADCON0bits.GO); // wait  until it's done
    
    long ADC_result = ADRESH << 8 | ADRESL;
    return ADC_result;
}

void ADC_Init() {
    TRISAbits.RA0 = 1;
    
    ADCON1bits.VCFG0 = 0;
    ADCON1bits.VCFG1 = 0;
    ADCON1bits.PCFG = 0b1110; // AN0 is analog input, others are digital
    ADCON0bits.CHS = 0b0000;  // AN0 as analog input
    ADCON2bits.ADCS = 0b001;  // set 000 for 1Mhz, which is less than 2.86Mhz
    ADCON2bits.ACQT = 0b100;  // Tad = 2 us acquisition time, 2*Tad = 4 > 2.4
    
    ADCON2bits.ADFM = 1;    // right justified
    ADCON0bits.ADON = 1;
    
    PIE1bits.ADIE = 0; // disable 
    PIR1bits.ADIF = 0;
    IPR1bits.ADIP = 1; // high priority
}


int CalculateAlcoholLevel() {
    long ADC_result = ADC_Read();
    // estimate baseline
    long baseline = alcohol_baseline_fp / 1000;
    if (!is_get_first_sample) {
        alcohol_baseline_fp = ADC_result * 1000;
        if (alcohol_baseline_fp >= 300L*1000L) alcohol_baseline_fp = 300L*1000L;
        is_get_first_sample = 1;
        baseline = alcohol_baseline_fp / 1000;
    } else {
        ++alcohol_sample_count;
        if (alcohol_sample_count >= SAMPLE_COUNT_THRESHOLD) {
            alcohol_sample_count = 0;
            if (ADC_result < baseline) // if current < baseline, then drop fast
                alcohol_baseline_fp = alcohol_baseline_fp * 750 / 1000 + ADC_result * 250;
        }
    }
    
    baseline = alcohol_baseline_fp / 1000;
    long delta = ADC_result - baseline;
    if (delta < 0) delta = 0;
    int level = 1;
    
    if (delta >= 10) level = 2;
    if (delta >= 20) level = 3;
    if (delta >= 40) level = 4;
    if (delta >= 60) level = 5;
    if (delta >= 80) level = 6;
    if (delta >= 120) level = 7;
    if (delta >= 150) level = 8;
    
//    sprintf(buffer, "bas: %ld, cur: %ld, del: %ld", baseline, ADC_result, delta);
//    UART_Write_Text(buffer);
//    UART_Write('\n');
//    UART_Write('\r');
    //[0, 1023] to [0, 7]
    //int level = (delta * 8) / 1024;
    return level;
}

// 0. receive level from CalculateAlcoholLevel()
// 1. init display_matrix to 0
// 2. show LV :, LO/MID/HI, counting and update_display()
void ShowAlcoholLevelOnLED(int level) {
    for(int i = 0; i < 16; i++) {
        for(int j = 0; j < 8; j++) {
            display_matrix[i][j] = 0;
        }
    }
    
    // LV :
    display_matrix[0][0] = display_matrix[0][3] = display_matrix[0][5] = display_matrix[0][7] = 1;
    display_matrix[1][0] = display_matrix[1][3] = display_matrix[1][5] = 1;
    display_matrix[2][0] = display_matrix[2][1] = display_matrix[2][2] = display_matrix[2][4] = display_matrix[2][7] = 1;
    // LO / MID / HI
    if (level <= 3) {
        display_matrix[5][0] = display_matrix[5][5] = display_matrix[5][6] = display_matrix[5][7] = 1;
        display_matrix[6][0] = display_matrix[6][5] = display_matrix[6][7] = 1;
        display_matrix[7][0] = display_matrix[7][1] = display_matrix[7][2] = display_matrix[7][5] = display_matrix[7][6] = display_matrix[7][7] = 1;
    } else if (level >= 4 && level <= 6) {
        display_matrix[5][0] = display_matrix[5][2] = display_matrix[5][4] = display_matrix[5][5] = display_matrix[5][6] = 1;
        display_matrix[6][0] = display_matrix[6][1] = display_matrix[6][2] = display_matrix[6][4] = display_matrix[6][5] = display_matrix[6][7] = 1;
        display_matrix[7][0] = display_matrix[7][2] = display_matrix[7][4] = display_matrix[7][5] = display_matrix[7][6] = 1;
    } else {
        display_matrix[5][0] = display_matrix[5][2] = display_matrix[5][5] = display_matrix[5][6] = display_matrix[5][7] = 1;
        display_matrix[6][0] = display_matrix[6][1] = display_matrix[6][2] = display_matrix[6][6] = 1;
        display_matrix[7][0] = display_matrix[7][2] = display_matrix[7][5] = display_matrix[7][6] = display_matrix[7][7] = 1;
    }
    
    // counting
    
    for(int _cnt = 1; _cnt <= level; _cnt++) {
        if (_cnt <= 5) { // [1, 5]
            if (_cnt != 5) {
                display_matrix[8][2*_cnt - 1] = display_matrix[9][2*_cnt - 1] = display_matrix[10][2*_cnt - 1] = 1;
            } else {
                for(int t=0; t<8; t++) 
                    display_matrix[9][t] = 1;
            }
        } else { // [6, 10]
            if (_cnt != 10) {
                display_matrix[13][2*(_cnt-5) - 1] = display_matrix[14][2*(_cnt-5) - 1] = display_matrix[15][2*(_cnt-5) - 1] = 1;
            } else {
                for(int t=0; t<8; t++) 
                    display_matrix[14][t] = 1;
            }
        }
    }
    update_display();
}


//void main(void)
//{
//    // In theory, these have been set.
//    INTCONbits.PEIE = 1;
//    INTCONbits.GIE = 1;
//    
//    OSCCONbits.IRCF = 0b111;
//    RCONbits.IPEN = 1;
//    
//    // call this function to init ADC
//    ADC_Init();
//    
//    //TODO: when entering alcohol mode, call CalculateAlcoholLevel() first, get the return value "level"
//    // then call ShowAlcoholLevelOnLED(int level), to show LED
//    // maybe this will be repeated with __delay_ms(1000);
//    
//    while(1);
//}