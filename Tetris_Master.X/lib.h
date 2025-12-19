#pragma once

/**
 * PIC18F4520 Microcontroller Library
 * PIC18F4520 Microcontroller Library
 *
 * This header file provides a complete hardware abstraction layer for the PIC18F4520 microcontroller
 * Including: clock setting, timers, PWM, ADC, UART, interrupt control, and other functions
 */

/* ========== Compiler Compatibility ========== */
#ifdef __XC8
// XC8 Compiler Environment - Used for actual flashing to the PIC microcontroller
#include <xc.h>
#include <pic18f4520.h>
#define bit __bit      // XC8's bit type
#define bool _Bool     // XC8's boolean type
#else
// Non-XC8 environment (Used for IDE syntax checking/simulation)
#define __interrupt(priority)  // Empty interrupt macro definition
#include "C:/Program Files/Microchip/xc8/v2.50/pic/include/proc/pic18f4520.h"
#define bit unsigned char      // Simulated bit type
#define bool unsigned char     // Simulated boolean type
#endif

#include <stdio.h>

/* ========== Basic Type Definitions ========== */
//#define false 0b0              // Boolean false value
//#define true 0b1               // Boolean true v

#define uint16_t unsigned short // 16-bit unsigned integer
#define byte unsigned char     // 8-bit unsigned integer (byte)

/* ========== Macro Helpers ========== */
#define STR(x) #x                          // Stringify macro
#define XSTR(s) STR(s)                     // Expand then stringify
#define MACRO_CODE_CONCAT(A, B) A##B       // Concatenate two tokens
#define MACRO_CODE_CONCAT3(A, B, C) A##B##C // Concatenate three tokens
#define _pinGetPortBits(reg, port, pin) MACRO_CODE_CONCAT3(reg, port, bits) // Get port bit structure
#define _pinGetPinBit(reg, port, pin) MACRO_CODE_CONCAT(reg, pin)         // Get pin bit

/* ========== ADC Clock Source Settings ========== */
/**
 * ADC Conversion Clock Source Selection
 * TAD = Time required for one ADC conversion bit
 * Selection must consider device main frequency to ensure TAD >= 0.7µs
 */
#define AD_CLOCK_SOURCE_2TOSC 0b000    // FOSC/2  - Max device frequency: 2.86 MHz
#define AD_CLOCK_SOURCE_4TOSC 0b100    // FOSC/4  - Max device frequency: 5.71 MHz
#define AD_CLOCK_SOURCE_8TOSC 0b001    // FOSC/8  - Max device frequency: 11.43 MHz
#define AD_CLOCK_SOURCE_16TOSC 0b101   // FOSC/16 - Max device frequency: 22.86 MHz
#define AD_CLOCK_SOURCE_32TOSC 0b010   // FOSC/32 - Max device frequency: 40.0 MHz
#define AD_CLOCK_SOURCE_64TOSC 0b110   // FOSC/64 - Max device frequency: 40.0 MHz
#define AD_CLOCK_SOURCE_RC 0b011       // Internal RC Oscillator - Max device frequency: 1.00 MHz

/* ========== Internal Oscillator Frequency ========== */
/**
 * IRCF<2:0>: Internal Oscillator Frequency Select bits
 * Used to set the IRCF bits of the OSCCON register
 */
#define INTERNAL_CLOCK_8MHz 0b111    // 8 MHz (INTOSC direct clock drive)
#define INTERNAL_CLOCK_4MHz 0b110    // 4 MHz
#define INTERNAL_CLOCK_2MHz 0b101    // 2 MHz
#define INTERNAL_CLOCK_1MHz 0b100    // 1 MHz
#define INTERNAL_CLOCK_500kHz 0b011  // 500 kHz
#define INTERNAL_CLOCK_250kHz 0b010  // 250 kHz
#define INTERNAL_CLOCK_125kHz 0b001  // 125 kHz
#define INTERNAL_CLOCK_31kHz 0b000   // 31 kHz (from INTOSC/256 or direct INTRC)
/**
 * Automatically select the corresponding clock setting based on _XTAL_FREQ
 * _XTAL_FREQ: System main frequency (Hz), must be defined in the project settings
 *
 * 32MHz and 16MHz require enabling PLL (Phase-Locked Loop)
 * PLL will multiply the 8MHz/4MHz internal oscillator to the target frequency
 */
#if (_XTAL_FREQ == 32000000)
// 32 MHz: Use 8MHz internal oscillator + 4x PLL
#define INTERNAL_CLOCK_IRCF INTERNAL_CLOCK_8MHz
#define AD_CLOCK_SOURCE AD_CLOCK_SOURCE_32TOSC
#define PLL_ENABLE
#elif (_XTAL_FREQ == 16000000)
// 16 MHz: Use 4MHz internal oscillator + 4x PLL
#define INTERNAL_CLOCK_IRCF INTERNAL_CLOCK_4MHz
#define AD_CLOCK_SOURCE AD_CLOCK_SOURCE_16TOSC
#define PLL_ENABLE
#elif (_XTAL_FREQ == 8000000)
// 8 MHz: Direct use of 8MHz internal oscillator
#define INTERNAL_CLOCK_IRCF INTERNAL_CLOCK_8MHz
#define AD_CLOCK_SOURCE AD_CLOCK_SOURCE_8TOSC
#elif (_XTAL_FREQ == 4000000)
// 4 MHz: Direct use of 4MHz internal oscillator
#define INTERNAL_CLOCK_IRCF INTERNAL_CLOCK_4MHz
#define AD_CLOCK_SOURCE AD_CLOCK_SOURCE_4TOSC
#elif (_XTAL_FREQ == 2000000)
// 2 MHz: Direct use of 2MHz internal oscillator
#define INTERNAL_CLOCK_IRCF INTERNAL_CLOCK_2MHz
#define AD_CLOCK_SOURCE AD_CLOCK_SOURCE_2TOSC
#elif (_XTAL_FREQ == 1000000)
// 1 MHz: Direct use of 1MHz internal oscillator
#define INTERNAL_CLOCK_IRCF INTERNAL_CLOCK_1MHz
#define AD_CLOCK_SOURCE AD_CLOCK_SOURCE_2TOSC
#elif (_XTAL_FREQ == 500000)
// 500 kHz: Direct use of 500kHz internal oscillator
#define INTERNAL_CLOCK_IRCF INTERNAL_CLOCK_500kHz
#define AD_CLOCK_SOURCE AD_CLOCK_SOURCE_2TOSC
#elif (_XTAL_FREQ == 250000)
// 250 kHz: Direct use of 250kHz internal oscillator
#define INTERNAL_CLOCK_IRCF INTERNAL_CLOCK_250kHz
#define AD_CLOCK_SOURCE AD_CLOCK_SOURCE_2TOSC
#elif (_XTAL_FREQ == 125000)
// 125 kHz: Direct use of 125kHz internal oscillator
#define INTERNAL_CLOCK_IRCF INTERNAL_CLOCK_125kHz
#define AD_CLOCK_SOURCE AD_CLOCK_SOURCE_2TOSC
#elif (_XTAL_FREQ == 31000)
// 31 kHz: Use the lowest frequency internal oscillator
#define INTERNAL_CLOCK_IRCF INTERNAL_CLOCK_31kHz
#define AD_CLOCK_SOURCE AD_CLOCK_SOURCE_2TOSC
#else
#error Illegal internal clock speed setting in "_XTAL_FREQ", please modify.
#endif

/* ========== Internal Clock Setup Macro ========== */
#ifdef PLL_ENABLE
/**
 * setIntrnalClock() - Set internal oscillator clock (with PLL)
 *
 * OSCCON: Oscillator Control Register
 * Reference: https://ww1.microchip.com/downloads/en/devicedoc/39631e.pdf#page=30
 *
 * Steps:
 * 1. Set IRCF bits to select internal oscillator frequency
 * 2. Execute Nop() to wait for stabilization
 * 3. Enable PLL to multiply the frequency by 4
 */
#define setIntrnalClock()                     \
    OSCCONbits.IRCF = INTERNAL_CLOCK_IRCF; \
    Nop();                                \
    OSCTUNEbits.PLLEN = 0b1
#else
/**
 * setIntrnalClock() - Set internal oscillator clock (without PLL)
 *
 * OSCCON: Oscillator Control Register
 * Reference: https://ww1.microchip.com/downloads/en/devicedoc/39631e.pdf#page=30
 *
 * Directly set IRCF bits to select internal oscillator frequency
 */
#define setIntrnalClock() OSCCONbits.IRCF = INTERNAL_CLOCK_IRCF
#endif

/* ========== ADC Acquisition Time ========== */
#pragma region AD_AcquisitionTime
/**
 * ADC Acquisition Time Calculation
 * Reference: https://ww1.microchip.com/downloads/en/devicedoc/39631e.pdf#page=230
 *
 * The ADC requires an acquisition time before starting conversion for the sampling capacitor to charge
 * Minimum acquisition time = 2.4µs (2400 nanoseconds)
 */
#define MINIMUM_ACQUISITION_TIME 2400  // Minimum acquisition time (nanoseconds)

/**
 * Calculate TAD (ADC Clock Period)
 * TAD = Time required for one ADC conversion bit
 * Formula: TAD = (Prescaler Ratio) / FOSC * 10^9 (convert to nanoseconds)
 */
#if (AD_CLOCK_SOURCE == AD_CLOCK_SOURCE_2TOSC)
#define _AD_CONVETER_TAD 2 * 1000000000 / _XTAL_FREQ    // TAD = 2/FOSC
#elif (AD_CLOCK_SOURCE == AD_CLOCK_SOURCE_4TOSC)
#define _AD_CONVETER_TAD 4 * 1000000000 / _XTAL_FREQ    // TAD = 4/FOSC
#elif (AD_CLOCK_SOURCE == AD_CLOCK_SOURCE_8TOSC)
#define _AD_CONVETER_TAD 8 * 1000000000 / _XTAL_FREQ    // TAD = 8/FOSC
#elif (AD_CLOCK_SOURCE == AD_CLOCK_SOURCE_16TOSC)
#define _AD_CONVETER_TAD 12 * 1000000000 / _XTAL_FREQ  // TAD = 12/FOSC
#elif (AD_CLOCK_SOURCE == AD_CLOCK_SOURCE_32TOSC)
#define _AD_CONVETER_TAD 32 * 1000000000 / _XTAL_FREQ  // TAD = 32/FOSC
#elif (AD_CLOCK_SOURCE == AD_CLOCK_SOURCE_64TOSC)
#define _AD_CONVETER_TAD 64 * 1000000000 / _XTAL_FREQ  // TAD = 64/FOSC
#else
#error TAD calculation not supported for RC clock source
#endif

/**
 * ACQT<2:0>: A/D Acquisition Time Select bits
 * Used in the ADCON2 register to set the number of acquisition periods before conversion
 */
#define AD_ACQUISITION_TIME_0TAD 0b000    // 0 TAD (Manual control)
#define AD_ACQUISITION_TIME_2TAD 0b001    // 2 TAD
#define AD_ACQUISITION_TIME_4TAD 0b010    // 4 TAD
#define AD_ACQUISITION_TIME_6TAD 0b011    // 6 TAD
#define AD_ACQUISITION_TIME_8TAD 0b100    // 8 TAD
#define AD_ACQUISITION_TIME_12TAD 0b101  // 12 TAD
#define AD_ACQUISITION_TIME_16TAD 0b110  // 16 TAD
#define AD_ACQUISITION_TIME_20TAD 0b111  // 20 TAD

/**
 * Automatically select the minimum required acquisition time
 * Ensure acquisition time >= MINIMUM_ACQUISITION_TIME (2.4µs)
 */
#if (2 * _AD_CONVETER_TAD > MINIMUM_ACQUISITION_TIME)
#define AD_ACQUISITION_TIME AD_ACQUISITION_TIME_2TAD
#elif (4 * _AD_CONVETER_TAD > MINIMUM_ACQUISITION_TIME)
#define AD_ACQUISITION_TIME AD_ACQUISITION_TIME_4TAD
#elif (6 * _AD_CONVETER_TAD > MINIMUM_ACQUISITION_TIME)
#define AD_ACQUISITION_TIME AD_ACQUISITION_TIME_6TAD
#elif (8 * _AD_CONVETER_TAD > MINIMUM_ACQUISITION_TIME)
#define AD_ACQUISITION_TIME AD_ACQUISITION_TIME_8TAD
#elif (12 * _AD_CONVETER_TAD > MINIMUM_ACQUISITION_TIME)
#define AD_ACQUISITION_TIME AD_ACQUISITION_TIME_12TAD
#elif (16 * _AD_CONVETER_TAD > MINIMUM_ACQUISITION_TIME)
#define AD_ACQUISITION_TIME AD_ACQUISITION_TIME_16TAD
#elif (20 * _AD_CONVETER_TAD > MINIMUM_ACQUISITION_TIME)
#define AD_ACQUISITION_TIME AD_ACQUISITION_TIME_20TAD
#endif
#pragma endregion AD_AcquisitionTime

/* ========== Timer0 ========== */
#pragma region Timer0
/**
 * Timer0 is a selectable 8/16 bit timer/counter
 * Features:
 * - Selectable 8-bit or 16-bit mode
 * - Programmable prescaler (1:2 to 1:256)
 * - Selectable internal or external clock source
 * - Can generate interrupt on overflow
 *
 * Reference: https://ww1.microchip.com/downloads/en/devicedoc/39631e.pdf#page=125
 */

/* Timer0 Prescaler Selection (T0PS<2:0>) */
#define TIMER0_PRESCALE_2 0b000    // 1:2 Prescale
#define TIMER0_PRESCALE_4 0b001    // 1:4 Prescale
#define TIMER0_PRESCALE_8 0b010    // 1:8 Prescale
#define TIMER0_PRESCALE_16 0b011   // 1:16 Prescale
#define TIMER0_PRESCALE_32 0b100   // 1:32 Prescale
#define TIMER0_PRESCALE_64 0b101   // 1:64 Prescale
#define TIMER0_PRESCALE_128 0b110  // 1:128 Prescale
#define TIMER0_PRESCALE_256 0b111  // 1:256 Prescale

/* Timer0 Prescaler Enable/Disable (PSA bit) */
#define TIMER0_PRESCALE_ENABLE 0b0   // Enable prescaler
#define TIMER0_PRESCALE_DISABLE 0b1  // Disable prescaler (1:1)

/* Timer0 Clock Source Selection (T0CS bit) */
#define TIMER0_CLOCK_SOURCE_T0CKI_PIN 0b1  // Use T0CKI pin external clock
#define TIMER0_CLOCK_SOURCE_INTERNAL 0b0   // Use internal instruction cycle (FOSC/4)

/* Timer0 Mode Selection (T08BIT bit) */
#define TIMER0_MODE_8BIT 0b1    // 8-bit mode (TMR0L)
#define TIMER0_MODE_16BIT 0b0  // 16-bit mode (TMR0H:TMR0L)

/**
 * enableTimer0() - Enable and configure Timer0
 * @param prescale       Prescale ratio (TIMER0_PRESCALE_x)
 * @param prescaleEnable Prescaler enable (TIMER0_PRESCALE_ENABLE/DISABLE)
 * @param clockSource    Clock source (TIMER0_CLOCK_SOURCE_x)
 * @param mode           Bit mode (TIMER0_MODE_8BIT/16BIT)
 *
 * T0CON Register Configuration
 */
#define enableTimer0(prescale, prescaleEnable, clockSource, mode) \
    T0CONbits.TMR0ON = 0b1;                                     \
    T0CONbits.T08BIT = mode;                                    \
    T0CONbits.T0CS = clockSource;                               \
    T0CONbits.PSA = prescaleEnable;                             \
    T0CONbits.T0PS = prescale;

#define disableTimer0() T0CONbits.TMR0ON = 0b0  // Disable Timer0
#define clearInterrupt_Timer0Overflow() INTCONbits.TMR0IF = 0b0  // Clear overflow interrupt flag

/**
 * enableInterrupt_Timer0Overflow() - Enable Timer0 overflow interrupt
 * @param priority  Interrupt priority (1=High, 0=Low)
 */
#define enableInterrupt_Timer0Overflow(priority) \
    INTCONbits.TMR0IE = 0b1;                  /* Enable TMR0 Overflow Interrupt */ \
    INTCON2bits.TMR0IP = priority;            /* Set interrupt priority */       \
    clearInterrupt_Timer0Overflow()

#define interruptByTimer0Overflow() INTCONbits.TMR0IF  // Check if it is Timer0 overflow interrupt

/**
 * setTimer0InterruptPeriod8() - Set interrupt period for 8-bit mode
 * @param period    Period time (microseconds µs)
 * @param prescale  Prescale ratio value (2, 4, 8, 16, 32, 64, 128, 256)
 *
 * Calculation formula: TMR0 = 255 - (period × FOSC) / (4 × prescale × 10^6) + 1
 */
#define setTimer0InterruptPeriod8(period, prescale) TMR0 = (byte)(255 - (period) / (1000000.0 / _XTAL_FREQ) / 4 / prescale + 1)

/**
 * setTimer0InterruptPeriod16() - Set interrupt period for 16-bit mode
 * @param period    Period time (microseconds µs)
 * @param prescale  Prescale ratio value (2, 4, 8, 16, 32, 64, 128, 256)
 *
 * Calculation formula: TMR0 = 65535 - (period × FOSC) / (4 × prescale × 10^6) + 1
 */
#define setTimer0InterruptPeriod16(period, prescale) TMR0 = (unsigned short)(65535 - (period) / (1000000.0 / _XTAL_FREQ) / 4 / prescale + 1)

#pragma endregion Timer0

/* ========== Timer1 ========== */
#pragma region Timer1
/**
 * Timer1 is a 16-bit timer/counter
 * Features:
 * - 16-bit timer/counter (TMR1H:TMR1L)
 * - Programmable prescaler (1:1, 1:2, 1:4, 1:8)
 * - Selectable internal or external clock source
 * - Supports Capture/Compare functions of the CCP module
 * - Can generate interrupt on overflow
 *
 * Reference: https://ww1.microchip.com/downloads/en/devicedoc/39631e.pdf#page=129
 */

/* Timer1 Prescaler Selection (T1CKPS<1:0>) */
#define TIMER1_PRESCALE_1 0b00  // 1:1 Prescale (No prescale)
#define TIMER1_PRESCALE_2 0b01  // 1:2 Prescale
#define TIMER1_PRESCALE_4 0b10  // 1:4 Prescale
#define TIMER1_PRESCALE_8 0b11  // 1:8 Prescale

/**
 * enableTimer1() - Enable Timer1
 * @param prescale  Prescale ratio (TIMER1_PRESCALE_x)
 *
 * Set to 16-bit read/write mode and enable the timer
 */
#define enableTimer1(prescale)     \
    T1CONbits.RD16 = 1;          /* 16-bit read/write mode */ \
    T1CONbits.T1CKPS = prescale; /* Set prescale ratio */   \
    T1CONbits.TMR1ON = 0b1       /* Enable Timer1 */

/**
 * configTimer1() - Configure Timer1 but do not enable
 * @param prescale  Prescale ratio (TIMER1_PRESCALE_x)
 */
#define configTimer1(prescale) \
    T1CONbits.RD16 = 1;      \
    T1CONbits.T1CKPS = prescale

#define enableTimer1bit() T1CONbits.TMR1ON = 0b1    // Enable Timer1
#define disableTimer1() T1CONbits.TMR1ON = 0b0      // Disable Timer1
#define clearInterrupt_Timer1Overflow() PIR1bits.TMR1IF = 0b0  // Clear overflow interrupt flag

/**
 * enableInterrupt_Timer1Overflow() - Enable Timer1 overflow interrupt
 * @param priority  Interrupt priority (1=High, 0=Low)
 */
#define enableInterrupt_Timer1Overflow(priority) \
    PIE1bits.TMR1IE = 0b1;                    /* Enable TMR1 Overflow Interrupt */ \
    IPR1bits.TMR1IP = priority;               /* Set interrupt priority */       \
    clearInterrupt_Timer1Overflow()

#define interruptByTimer1Overflow() PIR1bits.TMR1IF  // Check if it is Timer1 overflow interrupt

/**
 * setTimer1InterruptPeriod() - Set Timer1 interrupt period
 * @param period    Period time (microseconds µs)
 * @param prescale  Prescale ratio value (1, 2, 4, 8)
 *
 * Calculation formula: TMR1 = 65535 - (period × FOSC) / (4 × prescale × 10^6) + 1
 */
#define setTimer1InterruptPeriod(period, prescale) TMR1 = (unsigned short)(65535 - (period) / (1000000.0 / _XTAL_FREQ) / 4 / prescale + 1)
#pragma endregion Timer1

/* ========== Timer2 ========== */
#pragma region Timer2
/**
 * Timer2 is an 8-bit timer with prescaler and postscaler
 * Features:
 * - 8-bit timer (TMR2)
 * - 8-bit period register (PR2)
 * - Programmable prescaler (1:1, 1:4, 1:16)
 * - Programmable postscaler (1:1 to 1:16)
 * - Generates interrupt when TMR2 matches PR2
 * - Used as time base for PWM mode
 *
 * Reference: https://ww1.microchip.com/downloads/en/devicedoc/39631e.pdf#page=135
 */

/* Timer2 Prescaler Selection (T2CKPS<1:0>) */
#define TIMER2_PRESCALE_1 0b00    // 1:1 Prescale (No prescale)
#define TIMER2_PRESCALE_4 0b01    // 1:4 Prescale
#define TIMER2_PRESCALE_16 0b10  // 1:16 Prescale

/**
 * enableTimer2() - Enable Timer2
 * @param prescale      Prescale ratio (TIMER2_PRESCALE_x)
 * @param poscaleBits   Postscale ratio (0-15, corresponds to 1:1 to 1:16)
 *
 * Postscaler is used to reduce interrupt frequency: Actual interrupt frequency = Match frequency / (postscale + 1)
 */
#define enableTimer2(prescale, poscaleBits) \
    T2CONbits.T2CKPS = prescale;         /* Set prescale ratio */ \
    T2CONbits.T2OUTPS = poscaleBits;     /* Set postscale ratio */ \
    T2CONbits.TMR2ON = 0b1               /* Enable Timer2 */

/**
 * configTimer2() - Configure Timer2 but do not enable
 * @param prescale      Prescale ratio (TIMER2_PRESCALE_x)
 * @param poscaleBits   Postscale ratio (0-15)
 */
#define configTimer2(prescale, poscaleBits) \
    T2CONbits.T2CKPS = prescale;         \
    T2CONbits.T2OUTPS = poscaleBits

#define enableTimer2bit() T2CONbits.TMR2ON = 0b1    // Enable Timer2
#define disableTimer2() T2CONbits.TMR2ON = 0b0      // Disable Timer2
#define clearInterrupt_Timer2PR2() PIR1bits.TMR2IF = 0b0  // Clear TMR2/PR2 match interrupt flag

/**
 * enableInterrupt_Timer2PR2() - Enable Timer2 to PR2 match interrupt
 * @param priority  Interrupt priority (1=High, 0=Low)
 *
 * Interrupt triggered when TMR2 value equals PR2
 */
#define enableInterrupt_Timer2PR2(priority) \
    PIE1bits.TMR2IE = 0b1;                 /* Enable TMR2/PR2 Match Interrupt */ \
    IPR1bits.TMR2IP = priority;            /* Set interrupt priority */        \
    clearInterrupt_Timer2PR2()

#define interruptByTimer2PR2() PIR1bits.TMR2IF  // Check if it is Timer2/PR2 match interrupt

/**
 * disableInterrupt_Timer2PR2() - Disable Timer2 to PR2 match interrupt
 */
#define disableInterrupt_Timer2PR2() \
    PIE1bits.TMR2IE = 0b0;          \
    clearInterrupt_Timer2PR2()

/**
 * setTimer2InterruptPeriod() - Set Timer2 interrupt period
 * @param period    Period time (microseconds µs)
 * @param prescale  Prescale ratio value (1, 4, 16)
 * @param postscale Postscale ratio value (1-16)
 *
 * Calculation formula: PR2 = (period × FOSC) / (4 × prescale × postscale × 10^6) - 1
 * This value sets the period for TMR2 counting from 0 to PR2
 */
#define setTimer2InterruptPeriod(period, prescale, postscale) PR2 = (byte)((period) / (1000000.0 / _XTAL_FREQ) / 4 / prescale / postscale - 1)

#pragma endregion Timer2

/* ========== Timer3 ========== */
#pragma region Timer3
/**
 * Timer3 is a 16-bit timer/counter
 * Features:
 * - 16-bit timer/counter (TMR3H:TMR3L)
 * - Programmable prescaler (1:1, 1:2, 1:4, 1:8)
 * - Selectable internal or external clock source
 * - Can be used as Capture/Compare time base for the CCP module
 * - Can generate interrupt on overflow
 *
 * Reference: https://ww1.microchip.com/downloads/en/devicedoc/39631e.pdf#page=137
 */

/* Timer3 Prescaler Selection (T3CKPS<1:0>) */
#define TIMER3_PRESCALE_1 0b00  // 1:1 Prescale (No prescale)
#define TIMER3_PRESCALE_2 0b01  // 1:2 Prescale
#define TIMER3_PRESCALE_4 0b10  // 1:4 Prescale
#define TIMER3_PRESCALE_8 0b11  // 1:8 Prescale

/**
 * enableTimer3() - Enable Timer3
 * @param prescale  Prescale ratio (TIMER3_PRESCALE_x)
 */
#define enableTimer3(prescale) \
    T3CONbits.TMR3ON = 0b1;  /* Enable Timer3 */ \
    T3CONbits.T3CKPS = prescale /* Set prescale ratio */

#define disableTimer3() T3CONbits.TMR3ON = 0b0  // Disable Timer3
#define clearInterrupt_Timer3Overflow() PIR2bits.TMR3IF = 0b0  // Clear overflow interrupt flag

/**
 * enableInterrupt_Timer3Overflow() - Enable Timer3 overflow interrupt
 * @param priority  Interrupt priority (1=High, 0=Low)
 */
#define enableInterrupt_Timer3Overflow(priority) \
    PIE2bits.TMR3IE = 0b1;                    /* Enable TMR3 Overflow Interrupt */ \
    IPR2bits.TMR3IP = priority;               /* Set interrupt priority */       \
    clearInterrupt_Timer3Overflow()

#define interruptByTimer3Overflow() PIR2bits.TMR3IF  // Check if it is Timer3 overflow interrupt

/**
 * setTimer3InterruptPeriod() - Set Timer3 interrupt period
 * @param period    Period time (microseconds µs)
 * @param prescale  Prescale ratio value (1, 2, 4, 8)
 *
 * Calculation formula: TMR3 = 65535 - (period × FOSC) / (4 × prescale × 10^6) + 1
 */
#define setTimer3InterruptPeriod(period, prescale) TMR3 = (unsigned short)(65535 - (period) / (1000000.0 / _XTAL_FREQ) / 4 / prescale + 1)

#pragma endregion Timer3

/* ========== CCP Module Control / CCP Module Control ========== */
#pragma region PWM_Control
/**
 * Enhanced Capture/Compare/PWM (ECCP) Module
 *
 * CCP module provides three main functions:
 * 1. Capture Mode: Capture the timer value when an external event occurs
 * 2. Compare Mode: Trigger an event when the timer value matches a set value
 * 3. PWM Mode: Generate Pulse Width Modulation output signal
 *
 * PIC18F4520 has two CCP modules: CCP1 (Enhanced) and CCP2
 */

/* CCP Module Mode Selection (CCP1M<3:0> / CCP2M<3:0>) */
#define ECCP_MODE_OFF 0b0000            // Off - Reset CCP Module
#define ECCP_MODE_RESERVED 0b0001       // Reserved
#define ECCP_MODE_COMPARE_TOM 0b0010    // Compare Mode - Toggle Output on Match
#define ECCP_MODE_CAPTURE 0b0011        // Capture Mode - Every Edge
#define ECCP_MODE_CAPTURE_EFE 0b0100    // Capture Mode - Every Falling Edge
#define ECCP_MODE_CAPTURE_ERE 0b0101    // Capture Mode - Every Rising Edge
#define ECCP_MODE_CAPTURE_R04 0b0110    // Capture Mode - Every 4th Rising Edge
#define ECCP_MODE_CAPTURE_R16 0b0111    // Capture Mode - Every 16th Rising Edge
#define ECCP_MODE_COMPARE_SOM 0b1000    // Compare Mode - Initialize low, Set to high on match
#define ECCP_MODE_COMPARE_COM 0b1001    // Compare Mode - Initialize high, Clear to low on match
#define ECCP_MODE_COMPARE_RIO 0b1010    // Compare Mode - Generate software interrupt only
#define ECCP_MODE_COMPARE_TSE 0b1011    // Compare Mode - Trigger Special Event (Reset TMR1/TMR3)
#define ECCP_MODE_PWM_HH 0b1100         // PWM Mode - P1A,P1C High active; P1B,P1D High active
#define ECCP_MODE_PWM_HL 0b1101         // PWM Mode - P1A,P1C High active; P1B,P1D Low active
#define ECCP_MODE_PWM_LH 0b1110         // PWM Mode - P1A,P1C Low active; P1B,P1D High active
#define ECCP_MODE_PWM_LL 0b1111         // PWM Mode - P1A,P1C Low active; P1B,P1D Low active

/**
 * setCCP1Mode() / setCCP2Mode() - Set CCP module mode
 * @param eccpMode  Mode selection (ECCP_MODE_x)
 *
 * CCP1CON / CCP2CON Register Configuration
 */
#define setCCP1Mode(eccpMode) CCP1CONbits.CCP1M = eccpMode  // Set CCP1 mode
#define setCCP2Mode(eccpMode) CCP2CONbits.CCP2M = eccpMode  // Set CCP2 mode

/**
 * setCCP1PwmDutyCycle() - Set CCP1 PWM duty cycle
 * @param length    High time length (microseconds µs)
 * @param prescale  Timer2 prescale ratio value (1, 4, 16)
 *
 * PWM Duty Cycle = (CCPR1L:DC1B) × TOSC × Timer2 Prescale Ratio
 * 10-bit resolution: CCPR1L (High 8 bits) + DC1B (Low 2 bits)
 */
#define setCCP1PwmDutyCycle(length, prescale)                                                 \
    do {                                                                                      \
        unsigned int value = (unsigned int)((length) / (1000000.0 / _XTAL_FREQ) / prescale); \
        CCP1CONbits.DC1B = (byte)value & 0b11;  /* Low 2 bits */                             \
        CCPR1L = (byte)(value >> 2);            /* High 8 bits */                            \
    } while (0)

/**
 * setCCP2PwmDutyCycle() - Set CCP2 PWM duty cycle
 * @param length    High time length (microseconds µs)
 * @param prescale  Timer2 prescale ratio value (1, 4, 16)
 *
 * PWM Duty Cycle = (CCPR2L:DC2B) × TOSC × Timer2 Prescale Ratio
 * 10-bit resolution: CCPR2L (High 8 bits) + DC2B (Low 2 bits)
 */
#define setCCP2PwmDutyCycle(length, prescale)                                                 \
    do {                                                                                      \
        unsigned int value = (unsigned int)((length) / (1000000.0 / _XTAL_FREQ) / prescale); \
        CCP2CONbits.DC2B = (byte)value & 0b11;  /* Low 2 bits */                             \
        CCPR2L = (byte)(value >> 2);            /* High 8 bits */                            \
    } while (0)

#pragma endregion PWM_Control

/* ========== PIC18F4520 Pin Definitions ========== */
#pragma region PIC18F4520_Pins
/**
 * PIC18F4520 40-pin DIP package pin definitions
 *
 * Pin macro format: PORT, PINx
 * Used for pinMode(), digitalWrite(), etc. macros
 *
 * Each pin may have multiple functions:
 * - Digital I/O
 * - Analog Input (ANx)
 * - Special Functions (UART, SPI, I2C, CCP, etc.)
 */

/* PORTA Pins */
#define PIN_MCLR       // Pin 1:  MCLR / VPP / RE3 (Master Clear/Programming Voltage)
#define PIN_RA0 A, A0  // Pin 2:  RA0 / AN0 (Analog Channel 0)
#define PIN_RA1 A, A1  // Pin 3:  RA1 / AN1 (Analog Channel 1)
#define PIN_RA2 A, A2  // Pin 4:  RA2 / AN2 / VREF- / CVREF (Negative Reference Voltage)
#define PIN_RA3 A, A3  // Pin 5:  RA3 / AN3 / VREF+ (Positive Reference Voltage)
#define PIN_RA4 A, A4  // Pin 6:  RA4 / T0CKI / C1OUT (Timer0 External Clock)
#define PIN_RA5 A, A5  // Pin 7:  RA5 / AN4 / SS / HLVDIN / C2OUT (SPI Slave Select)

/* PORTE Pins */
#define PIN_RE0 E, E0  // Pin 8:  RE0 / RD / AN5 (Parallel Slave Port Read)
#define PIN_RE1 E, E1  // Pin 9:  RE1 / WR / AN6 (Parallel Slave Port Write)
#define PIN_RE2 E, E2  // Pin 10: RE2 / CS / AN7 (Parallel Slave Port Chip Select)

/* Oscillator Pins */
#define PIN_OSC1       // Pin 13: OSC1 / CLKI / RA7 (Oscillator Input)
#define PIN_OSC2       // Pin 14: OSC2 / CLKO / RA6 (Oscillator Output)

/* PORTC Pins */
#define PIN_RC0 C, C0  // Pin 15: RC0 / T1OSO / T13CKI (Timer1 Oscillator Output)
#define PIN_RC1 C, C1  // Pin 16: RC1 / T1OSI / CCP2 (Timer1 Oscillator Input/CCP2)
#define PIN_RC2 C, C2  // Pin 17: RC2 / CCP1 / P1A (CCP1 PWM Output)
#define PIN_RC3 C, C3  // Pin 18: RC3 / SCK / SCL (SPI Clock/I2C Clock)
#define PIN_RC4 C, C4  // Pin 23: RC4 / SDI / SDA (SPI Data Input/I2C Data)
#define PIN_RC5 C, C5  // Pin 24: RC5 / SDO (SPI Data Output)
#define PIN_RC6 C, C6  // Pin 25: RC6 / TX / CK (UART Transmit)
#define PIN_RC7 C, C7  // Pin 26: RC7 / RX / DT (UART Receive)

/* PORTD Pins */
#define PIN_RD0 D, D0  // Pin 19: RD0 / PSP0 (Parallel Slave Port Data 0)
#define PIN_RD1 D, D1  // Pin 20: RD1 / PSP1 (Parallel Slave Port Data 1)
#define PIN_RD2 D, D2  // Pin 21: RD2 / PSP2 (Parallel Slave Port Data 2)
#define PIN_RD3 D, D3  // Pin 22: RD3 / PSP3 (Parallel Slave Port Data 3)
#define PIN_RD4 D, D4  // Pin 27: RD4 / PSP4 (Parallel Slave Port Data 4)
#define PIN_RD5 D, D5  // Pin 28: RD5 / PSP5 / P1B (ECCP P1B Output)
#define PIN_RD6 D, D6  // Pin 29: RD6 / PSP6 / P1C (ECCP P1C Output)
#define PIN_RD7 D, D7  // Pin 30: RD7 / PSP7 / P1D (ECCP P1D Output)

/* PORTB Pins */
#define PIN_RB0 B, B0  // Pin 33: RB0 / INT0 / FLT0 / AN12 (External Interrupt 0)
#define PIN_RB1 B, B1  // Pin 34: RB1 / INT1 / AN10 (External Interrupt 1)
#define PIN_RB2 B, B2  // Pin 35: RB2 / INT2 / AN8 (External Interrupt 2)
#define PIN_RB3 B, B3  // Pin 36: RB3 / AN9 / CCP2 (Alternate CCP2)
#define PIN_RB4 B, B4  // Pin 37: RB4 / KBI0 / AN11 (Keyboard Interrupt 0)
#define PIN_RB5 B, B5  // Pin 38: RB5 / KBI1 / PGM (Low-Voltage Programming)
#define PIN_RB6 B, B6  // Pin 39: RB6 / KBI2 / PGC (Programming Clock)
#define PIN_RB7 B, B7  // Pin 40: RB7 / KBI3 / PGD (Programming Data)
#pragma endregion PIC18F4520_Pins

/* ========== Pin Control ========== */
#pragma region PinControl
/**
 * Digital I/O Pin Control Macros
 *
 * Usage:
 * - pinMode(PIN_RA0, PIN_OUTPUT);  // Set RA0 as output
 * - digitalWrite(PIN_RA0, 1);      // Set RA0 output high
 * - byte state = pinState(PIN_RA0); // Read RA0 output status
 */

/* Pin Direction Setting (TRISx Register) */
#define PIN_INPUT 0b1    // Set as Input (High impedance)
#define PIN_OUTPUT 0b0  // Set as Output

/**
 * pinMode() - Set pin input/output direction
 * @param pin   Pin (Use PIN_Rxx macro)
 * @param mode  Direction (PIN_INPUT or PIN_OUTPUT)
 *
 * Operate the corresponding bit of the TRISx register
 */
#define pinMode(pin, mode) _pinGetPortBits(TRIS, pin)._pinGetPinBit(R, pin) = mode

/**
 * digitalWrite() - Set pin output level
 * @param pin   Pin (Use PIN_Rxx macro)
 * @param value Output value (0=Low, 1=High)
 *
 * Operate the corresponding bit of the LATx register (Latch)
 * Using LAT instead of PORT avoids Read-Modify-Write issues
 */
#define digitalWrite(pin, value) _pinGetPortBits(LAT, pin)._pinGetPinBit(L, pin) = value

/**
 * pinState() - Read pin output latch status
 * @param pin Pin (Use PIN_Rxx macro)
 * @return    Current output latch value (0 or 1)
 */
#define pinState(pin) _pinGetPortBits(LAT, pin)._pinGetPinBit(L, pin)

/* PORTB Internal Pull-up Resistor Control */
#define PORTB_PULLUP_ENABLE 0b0    // Enable PORTB internal pull-up resistors
#define PORTB_PULLUP_DISABLE 0b1  // Disable PORTB internal pull-up resistors

/**
 * setPortBPullup() - Set PORTB internal pull-up resistors
 * @param state  State (PORTB_PULLUP_ENABLE/DISABLE)
 *
 * PORTB pins RB<7:4> have programmable internal weak pull-up resistors
 * Can be used for buttons and other applications when set as input, saving external pull-up resistors
 */
#define setPortBPullup(state) INTCON2bits.RBPU = state
#pragma endregion PinControl

/* ========== ADC Control ========== */
#pragma region AD_Control
/**
 * 10-bit Analog-to-Digital Converter (ADC)
 *
 * PIC18F4520 has 13 analog input channels (AN0-AN12)
 * ADC converts analog voltage to a 10-bit digital value (0-1023)
 *
 * Conversion Formula: Digital Value = (Input Voltage / Reference Voltage) × 1023
 *
 * Reference: https://ww1.microchip.com/downloads/en/devicedoc/39631e.pdf#page=228
 */

/**
 * setANPinADConfig() - Set analog/digital pin configuration
 * @param value  4-bit value, each bit corresponds to a group of pins
 * 0 = Analog Input, 1 = Digital I/O
 *
 * PCFG<3:0> bits of the ADCON1 register
 * Reference: https://ww1.microchip.com/downloads/en/devicedoc/39631e.pdf#page=226
 */
#define setANPinADConfig(value) ADCON1bits.PCFG = value

/**
 * setANPinVoltageReferenceConfig() - Set ADC reference voltage source
 * @param conf0  VREF- source: 1 = AN2 pin, 0 = VSS (Ground)
 * @param conf1  VREF+ source: 1 = AN3 pin, 0 = VDD (Supply Voltage)
 *
 * Using external reference voltage can improve conversion accuracy
 * Default reference voltages are VDD and VSS
 */
#define setANPinVoltageReferenceConfig(conf0, conf1) \
    ADCON1bits.VCFG1 = conf0;                      \
    ADCON1bits.VCFG0 = conf1

/**
 * setANPinAnalogChannelSelect() - Select the analog channel to convert
 * @param value  Channel number (0-12, corresponding to AN0-AN12)
 *
 * Set CHS<3:0> bits of ADCON0
 */
#define setANPinAnalogChannelSelect(value) ADCON0bits.CHS = value

/**
 * startADConverter() - Start ADC conversion
 *
 * Set the GO/DONE bit to start conversion
 * This bit is automatically cleared upon completion
 */
#define startADConverter() ADCON0bits.GO = 1

/**
 * getADConverter() - Read ADC conversion result
 * @return  10-bit conversion result (0-1023)
 *
 * Read ADRESH:ADRESL registers
 */
#define getADConverter() ADRES

/**
 * enableADConverter() - Enable and initialize ADC module
 *
 * Performs the following settings:
 * - Enable ADC module (ADON = 1)
 * - Set result right-justified (ADFM = 1): High 6 bits in ADRESH, Low 8 bits in ADRESL
 * - Set ADC clock source (Automatically selected based on system frequency)
 * - Set acquisition time (Automatically calculated based on clock)
 */
#define enableADConverter()                                     \
    ADCON0bits.ADON = 0b1;           /* Enable ADC */         \
    ADCON2bits.ADFM = 0b1;           /* Result right-justified */ \
    ADCON2bits.ADCS = AD_CLOCK_SOURCE; /* Set ADC clock source */ \
    ADCON2bits.ACQT = AD_ACQUISITION_TIME /* Set acquisition time */

#define clearInterrupt_ADConverter() PIR1bits.ADIF = 0  // Clear ADC interrupt flag

/**
 * enableInterrupt_ADConverter() - Enable ADC conversion complete interrupt
 * @param priority  Interrupt priority (1=High, 0=Low)
 *
 * Interrupt triggered when ADC conversion is complete
 */
#define enableInterrupt_ADConverter(priority) \
    clearInterrupt_ADConverter();           \
    PIE1bits.ADIE = 1;                      /* Enable ADC interrupt */ \
    IPR1bits.ADIP = priority                /* Set interrupt priority */

#define interruptByADConverter() PIR1bits.ADIF  // Check if it is ADC conversion complete interrupt

#pragma endregion AD_Control

/* ========== Interrupt Control ========== */
#pragma region InterruptControl
/**
 * PIC18F4520 Interrupt System
 *
 * Supports two priority levels:
 * - High Priority: Vector address 0x0008
 * - Low Priority: Vector address 0x0018
 *
 * Interrupt sources include:
 * - External Interrupts (INT0, INT1, INT2)
 * - Timer Overflow
 * - ADC Conversion Complete
 * - UART Transmit/Receive
 * - PORTB State Change
 * - And more...
 */

/**
 * enableInterruptPriorityMode() - Enable interrupt priority mode
 * @param state  1 = Enable two priority levels, 0 = Disable (Compatibility Mode)
 *
 * IPEN bit of the RCON register
 * Enables distinction between High/Low priority interrupts
 */
#define enableInterruptPriorityMode(state) RCONbits.IPEN = state

/**
 * enableGlobalInterrupt() - Global Interrupt Enable
 * @param state  1 = Enable, 0 = Disable
 *
 * When IPEN = 0 (Compatibility Mode):
 * - 1 = Enable all unmasked interrupts
 * - 0 = Disable all interrupts
 *
 * When IPEN = 1 (Priority Mode):
 * - 1 = Enable all High Priority interrupts
 * - 0 = Disable all interrupts
 */
#define enableGlobalInterrupt(state) INTCONbits.GIE = state

/**
 * enablePeripheralInterrupt() - Peripheral Interrupt Enable
 * @param state  1 = Enable, 0 = Disable
 *
 * When IPEN = 0 (Compatibility Mode):
 * - 1 = Enable all unmasked peripheral interrupts
 * - 0 = Disable all peripheral interrupts
 *
 * When IPEN = 1 (Priority Mode):
 * - 1 = Enable all Low Priority peripheral interrupts
 * - 0 = Disable all Low Priority peripheral interrupts
 */
#define enablePeripheralInterrupt(state) INTCONbits.PEIE = state

/* ---------- External Interrupt INT0 (RB0 Pin) ---------- */
#define clearInterrupt_RB0External() INTCONbits.INT0IF = 0b0  // Clear INT0 interrupt flag

/**
 * enableInterrupt_RB0External() - Enable RB0/INT0 external interrupt
 *
 * INT0 is fixed as High Priority, priority cannot be set
 * The INT0IF flag must be cleared in software when an interrupt occurs
 */
#define enableInterrupt_RB0External() \
    clearInterrupt_RB0External();   \
    INTCONbits.INT0IE = 0b1

#define interruptByRB0External() INTCONbits.INT0IF  // Check if it is INT0 interrupt

/* ---------- External Interrupt INT1 (RB1 Pin) ---------- */
#define clearInterrupt_RB1External() INTCON3bits.INT1IF = 0b0  // Clear INT1 interrupt flag

/**
 * enableInterrupt_RB1External() - Enable RB1/INT1 external interrupt
 * @param priority  Interrupt priority (1=High, 0=Low)
 *
 * The INT1IF flag must be cleared in software when an interrupt occurs
 */
#define enableInterrupt_RB1External(priority)     \
    clearInterrupt_RB1External();               \
    INTCON3bits.INT1IE = 0b1;                   /* Enable INT1 interrupt */ \
    INTCON3bits.INT1IP = priority               /* Set priority */

#define interruptByRB1External() INTCON3bits.INT1IF  // Check if it is INT1 interrupt

/* ---------- External Interrupt INT2 (RB2 Pin) ---------- */
#define clearInterrupt_RB2External() INTCON3bits.INT2IF = 0b0  // Clear INT2 interrupt flag

/**
 * enableInterrupt_RB2External() - Enable RB2/INT2 external interrupt
 * @param priority  Interrupt priority (1=High, 0=Low)
 *
 * The INT2IF flag must be cleared in software when an interrupt occurs
 */
#define enableInterrupt_RB2External(priority)     \
    clearInterrupt_RB2External();               \
    INTCON3bits.INT2IE = 0b1;                   /* Enable INT2 interrupt */ \
    INTCON3bits.INT2IP = priority               /* Set priority */

#define interruptByRB2External() INTCON3bits.INT2IF  // Check if it is INT2 interrupt

/* ---------- PORTB Change Interrupt (RB7:RB4) ---------- */
#define clearInterrupt_RBPortChange() INTCONbits.RBIF = 0b0  // Clear PORTB change interrupt flag

/**
 * enableInterrupt_RBPortChange() - Enable PORTB<7:4> state change interrupt
 * @param priority  Interrupt priority (1=High, 0=Low)
 *
 * Triggered when the state of any pin in RB<7:4> changes
 * Commonly used for keyboard scanning, button detection, etc.
 * The RBIF flag must be cleared in software
 */
#define enableInterrupt_RBPortChange(priority) \
    clearInterrupt_RBPortChange();           \
    INTCONbits.RBIE = 0b1;                   /* Enable PORTB change interrupt */ \
    INTCON2bits.RBIP = priority              /* Set priority */

#define interruptByRBPortChange() INTCONbits.RBIF  // Check if it is PORTB change interrupt

/* ---------- UART Transmit Interrupt ---------- */
#define clearInterrupt_TransmitUART() PIR1bits.TXIF = 0b0  // Clear UART transmit interrupt flag

/**
 * enableInterrupt_TransmitUART() - Enable UART transmit complete interrupt
 * @param priority  Interrupt priority (1=High, 0=Low)
 *
 * Triggered when TXREG is empty (ready to write the next byte)
 */
#define enableInterrupt_TransmitUART(priority) \
    clearInterrupt_TransmitUART();           \
    PIE1bits.TXIE = 0b1;                    /* Enable UART transmit interrupt */ \
    IPR1bits.TXIP = priority                /* Set priority */

#define interruptByTransmitUART() PIR1bits.TXIF  // Check if it is UART transmit interrupt

/* ---------- UART Receive Interrupt ---------- */
#define clearInterrupt_ReceiveUART() PIR1bits.RCIF = 0b0  // Clear UART receive interrupt flag

/**
 * enableInterrupt_ReceiveUART() - Enable UART receive complete interrupt
 * @param priority  Interrupt priority (1=High, 0=Low)
 *
 * Triggered when RCREG has data available for reading
 */
#define enableInterrupt_ReceiveUART(priority) \
    clearInterrupt_ReceiveUART();           \
    PIE1bits.RCIE = 0b1;                    /* Enable UART receive interrupt */ \
    IPR1bits.RCIP = priority                /* Set priority */

#define interruptByReceiveUART() PIR1bits.RCIF  // Check if it is UART receive interrupt

#pragma endregion InterruptControl

/* ========== UART Serial Communication ========== */
#pragma region UART
/**
 * EUSART (Enhanced Universal Synchronous Asynchronous Receiver Transmitter)
 *
 * Supported Features:
 * - Full-duplex asynchronous communication
 * - Programmable Baud Rate
 * - 9-bit data transmission
 * - Hardware flow control
 * - Auto-baud rate detection
 *
 * Pins:
 * - RC6/TX: Transmit
 * - RC7/RX: Receive
 */

/* serialPrintf buffer size, can be overridden by defining before include */
#ifndef SEIAL_PRINTF_STATIC_SIZE
#define SEIAL_PRINTF_STATIC_SIZE 16
#endif
char serialPrintfCache[SEIAL_PRINTF_STATIC_SIZE];  // printf format string buffer

/**
 * serialReceiveEnable() - Enable/Disable continuous receive mode
 * @param state  1 = Enable, 0 = Disable
 *
 * CREN bit controls continuous reception
 * Disabling and re-enabling can clear overflow error (OERR)
 */
#define serialReceiveEnable(state) RCSTAbits.CREN = state

/**
 * serialReceiveOverrunError() - Check for receive overrun error
 * @return  1 = Overrun error occurred, 0 = No error
 *
 * Occurs when the receive buffer is full and new data is received
 * Clear method: Set CREN to 0 then back to 1
 */
#define serialReceiveOverrunError() RCSTAbits.OERR

/**
 * serialReceiveFramingError() - Check for framing error
 * @return  1 = Framing error occurred, 0 = No error
 *
 * Occurs when the received stop bit is incorrect
 * Clear method: Read RCREG and receive the next valid byte
 */
#define serialReceiveFramingError() RCSTAbits.FERR

/**
 * serialBegin() - Initialize UART serial port
 * @param baudRate              Baud rate (e.g., 9600, 115200)
 * @param receiveInterruptPriority Receive interrupt priority (1=High, 0=Low)
 *
 * Automatically calculates the baud rate generator value
 * Configures for asynchronous mode, enables transmit and receive
 */
inline void serialBegin(long baudRate, byte receiveInterruptPriority) {
    pinMode(PIN_RC6, PIN_OUTPUT);  // Set RC6(TX) as output
    pinMode(PIN_RC7, PIN_INPUT);   // Set RC7(RX) as input

    // Set Baud Rate
    TXSTAbits.SYNC = 0;    // Asynchronous mode
    BAUDCONbits.BRG16 = 1; // Use 16-bit Baud Rate Generator

    long baudRateGenerator = _XTAL_FREQ / baudRate;
    TXSTAbits.BRGH = baudRate > 2400;  // High Baud Rate Select bit

    // Calculate prescaler ratio based on settings
    // Reference: https://ww1.microchip.com/downloads/en/devicedoc/39631e.pdf#page=207
    if (!TXSTAbits.SYNC && !BAUDCONbits.BRG16 && !TXSTAbits.BRGH)
        baudRateGenerator /= 64;
    else if (!TXSTAbits.SYNC && BAUDCONbits.BRG16 != TXSTAbits.BRGH)
        baudRateGenerator /= 16;
    else
        baudRateGenerator /= 4;

    // Set Baud Rate Generator registers
    SPBRGH = (byte)(baudRateGenerator >> 8);  // High byte
    SPBRG = (byte)baudRateGenerator;         // Low byte

    // Enable serial port
    RCSTAbits.SPEN = 1;  // Enable serial port (Set RX/TX pins to serial port function)
    TXSTAbits.TXEN = 1;  // Enable transmit
    RCSTAbits.CREN = 1;  // Enable continuous receive
    enableInterrupt_ReceiveUART(receiveInterruptPriority);

    /* Transmitter Architecture Description:
     * TSR   : Shift Register, data currently being transmitted
     * TXREG : Transmit Buffer, next data to be transmitted
     * TXSTAbits.TRMT: Set to 1 when TSR is empty
     */
    /* Receiver Architecture Description:
     * RSR   : Shift Register, data currently being received
     * RCREG : Receive Buffer, data that has been fully received (Read this register to get data)
     */
}

/**
 * serialAvailableForWrite() - Check if available for transmission
 * @return  1 = Writable, 0 = Busy
 *
 * Check if the transmit shift register (TSR) is empty
 */
#define serialAvailableForWrite() TXSTAbits.TRMT

/**
 * serialWrite() - Transmit a single character
 * @param c  Character to transmit
 *
 * Blocking transmission, waits until writable
 */
void serialWrite(char c) {
    while (!serialAvailableForWrite());  // Busy wait
    TXREG = c;                          // Write to TXREG to start transmission
}

/**
 * serialPrint() - Transmit a string
 * @param text  Null-terminated string
 *
 * Transmit character by character until '\0' is encountered
 */
void serialPrint(char *text) {
    for (int i = 0; text[i] != '\0'; i++) {
        while (!serialAvailableForWrite());  // Busy wait
        TXREG = text[i];                    // Write to TXREG to start transmission
    }
}

/**
 * serialPrintf() - Formatted output
 * @param format  Format string (same as printf)
 * @param ...     Variable arguments
 *
 * Formats using sprintf then transmits
 * Note: Buffer size defined by SEIAL_PRINTF_STATIC_SIZE
 */
#define serialPrintf(format, ...)              \
    sprintf(serialPrintfCache, format, __VA_ARGS__); \
    serialPrint(serialPrintfCache)

/**
 * serialRead() - Blocking read of a single character
 * @return  Received character
 *
 * Waits until data is available for reading
 */
char serialRead() {
    while (!interruptByReceiveUART());  // Wait for receive interrupt flag
    return RCREG;                      // Read and return data
}

/* Serial Port Callback Function Pointers */
void (*serialOnReadLine)(char *line, byte len);  // Callback when a complete line is received
void (*serialOnReadChar)(char c);                // Callback when a single character is received

/* Serial Port Receive Buffer */
char serialBuffer[64];      // Receive buffer (64 bytes)
byte serialBufferLen = 0;   // Current buffer length
char serialLastChar = '\0'; // Last received character (for CRLF handling)

/**
 * processSerialReceive() - Process serial port reception
 * @return  true = Data processed, false = No data
 *
 * This function should be called in the main loop or interrupt service routine
 * Functions:
 * - Handle overflow error
 * - Handle backspace key (0x7F)
 * - Handle newline (CR/LF/CRLF)
 * - Echo input character
 * - Call callback functions
 */
bool processSerialReceive() {
    if (interruptByReceiveUART()) {
        // Clear overflow error
        if (serialReceiveOverrunError()) {
            serialReceiveEnable(0);  // Disable receive
            Nop();                   // Short delay
            serialReceiveEnable(1);  // Re-enable receive
        }
        char c = serialRead();
        // Skip character if framing error occurred
        if (!serialReceiveFramingError()) {
            switch (c) {
            case '\x7f':  // DEL key (Backspace)
                if (!serialBufferLen)
                    break;
                // Echo backspace effect: Backspace + Space + Backspace
                serialWrite('\b');
                serialWrite(' ');
                serialWrite('\b');
                serialBuffer[--serialBufferLen] = '\0';
                break;
            case '\r':  // CR (Carriage Return)
            case '\n':  // LF (Line Feed)
                // Handle CRLF: Skip if LF and the previous was CR
                if (c == '\n' && !serialBufferLen && serialLastChar == '\r')
                    break;
                // serialWrite('\n');  // Echo newline
                serialBuffer[serialBufferLen] = '\0';
                // Call line read complete callback
                if (serialOnReadLine)
                    serialOnReadLine(serialBuffer, serialBufferLen);
                serialBuffer[serialBufferLen = 0] = '\0';  // Clear buffer
                break;
            case 0xff:  // Invalid character, skip
                break;
            default:
                // Call character received callback
                if (serialOnReadChar)
                    serialOnReadChar(c);
                // Check if buffer has space
                if (serialBufferLen < (byte)sizeof(serialBuffer) - 1) {
                    //serialWrite(c);                      // Echo character
                    serialBuffer[serialBufferLen++] = c; // Add to buffer
                }
            }
            serialLastChar = c;  // Record last character
        }
        return true;
    }
    return false;
}

#pragma endregion UART

/* ========== Servo Motor Control ========== */
/**
 * setCCP1ServoAngle() - Set servo motor angle
 * @param angle     Target angle (0~180 degrees)
 * @param prescale  Timer2 prescale ratio value (1, 4, 16)
 *
 * Servo Motor PWM Signal Description:
 * - Period: 20ms (50Hz)
 * - Pulse Width: 0.45ms ~ 2.45ms corresponds to 0° ~ 180°
 *
 * Pulse Width Calculation Formula:
 * pulse_width = 450µs + (angle / 180°) × (2450µs - 450µs)
 *
 * Note: Must be set up beforehand:
 * 1. Set Timer2 period to 20ms
 * 2. Set CCP1 to PWM mode
 * 3. Set RC2 as output
 */
#define setCCP1ServoAngle(angle, prescale) setCCP1PwmDutyCycle(450 + ((2450 - 450) / 180.0) * (angle), prescale)