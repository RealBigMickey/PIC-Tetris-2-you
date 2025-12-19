
/* Litterally ran out of Data memory, thus the first song uses const to be stored in Program memory :P */
const int note[] = {1318, 988, 1046, 1174, 1046, 988, 880, 880, 1046, 1318, 1174, 1046, 988, 1046, 1174, 1318, 1046, 880, 880, 880, 988, 1046, 1174, 1480, 1760, 1568, 1396, 1318, 1046, 1318, 1174, 1046, 988, 988, 1046, 1174, 1318, 1046, 880, 880, 0, 1318, 988, 1046, 1174, 1046, 988, 880, 880, 1046, 1318, 1174, 1046, 988, 1046, 1174, 1318, 1046, 880, 880, 880, 988, 1046, 1174, 1480, 1760, 1568, 1396, 1318, 1046, 1318, 1174, 1046, 988, 988, 1046, 1174, 1318, 1046, 880, 880, 0, 1318, 1046, 1174, 988, 1046, 880, 830, 988, 0, 1318, 1046, 1174, 988, 1046, 1318, 1760, 1662, -1};
const int interval[] = {417, 208, 208, 417, 208, 208, 417, 208, 208, 417, 208, 208, 625, 208, 417, 417, 417, 417, 208, 417, 208, 208, 625, 208, 417, 208, 208, 625, 208, 417, 208, 208, 417, 208, 208, 417, 417, 417, 417, 417, 417, 417, 208, 208, 417, 208, 208, 417, 208, 208, 417, 208, 208, 625, 208, 417, 417, 417, 417, 208, 417, 208, 208, 625, 208, 417, 208, 208, 625, 208, 417, 208, 208, 417, 208, 208, 417, 417, 417, 417, 417, 417, 833, 833, 833, 833, 833, 833, 833, 417, 417, 833, 833, 833, 833, 417, 417, 833, 833, 417};
int padoru_note[] = {880, 0, 880, 0, 1480, 0, 1318, 0, 1174, 0, 988, 0, 880, 0, 880, 0, 1480, 0, 1318, 0, 1174, 0, 1046, 0, 988, 0, 988, 0, 1568, 0, 1480, 0, 1318, 0, 1046, 0, 1760, 0, 1760, 0, 1760, 0, 1568, 0, 1318, 0, 1480, 0, -1};
int padoru_interval[] = {284, 31, 284, 31, 284, 31, 284, 31, 284, 31, 853, 94, 284, 31, 284, 31, 284, 31, 284, 31, 284, 31, 853, 94, 284, 31, 284, 31, 284, 31, 284, 31, 284, 31, 853, 94, 284, 31, 284, 31, 284, 31, 284, 31, 284, 31, 284, 31, 316};
#define TETRIS_NUM 100
#define PADORU_NUM 49

volatile int NUM_NOTE = TETRIS_NUM;
volatile int note_index = 0;

volatile char song = 0;

void Timer2_PWM_Init();
void Timer0_Init();
void tone(int freq);
void noTone();
void music_stop();
void music_restart();

void Timer2_PWM_Init(void)
{
    
    TRISCbits.TRISC2 = 0;   // CCP1 output

    // CCP1 PWM mode
    CCP1CONbits.CCP1M = 0b1100;

    // Timer2 setup
    T2CONbits.T2CKPS = 0b11; // prescaler 1:16
    T2CONbits.TMR2ON = 1;
}

void Timer0_Init(void)
{
    INTCON2bits.TMR0IP = 0;      // 0 = Low priority
    // Timer0 configuration 16-bit
    T0CONbits.T08BIT = 0;        // 0 = 16-bit mode
    T0CONbits.T0CS = 0;          // 0 = use internal instruction cycle clock (Fosc/4)
    T0CONbits.PSA = 0;           // 0 = prescaler assigned
    T0CONbits.T0PS = 0b111;      // prescaler 1:256
    T0CONbits.TMR0ON = 0;        // Turn off Timer0 first

    // Calculate preload for the first note
    // Assuming interval[note_index] unit is milliseconds
    int _tmp_interval;
    if (song == 0) _tmp_interval = interval[note_index];
    else _tmp_interval = padoru_interval[note_index];
    uint32_t counts = (uint32_t)(_tmp_interval) * 1000UL / 128UL;
    if(counts > 65535) counts = 65535;
    uint16_t preload = 65535 - counts + 1;

    TMR0H = preload >> 8;
    TMR0L = preload & 0xFF;

    // Interrupt settings
    INTCONbits.TMR0IF = 0;       // Clear Timer0 interrupt flag
    INTCONbits.TMR0IE = 1;       // Enable Timer0 interrupt
    INTCONbits.PEIE = 1;         // Enable peripheral interrupts
    INTCONbits.GIE = 1;          // Enable global interrupts

    T0CONbits.TMR0ON = 1;        // Start Timer0
}


void tone(int freq)
{
    if (freq == 0) {
        CCPR1L = 0;
        CCP1CONbits.DC1B = 0;
        return;
    }
    
    T2CONbits.TMR2ON = 0;
    TMR2 = 0;

    uint32_t pr2 = (_XTAL_FREQ / (4UL * 16UL * freq)) - 1;
    if(pr2 > 255) pr2 = 255;

    PR2 = (uint8_t)pr2;

    uint16_t duty = (PR2 + 1) * 2;  // 50% duty cycle
    CCPR1L = duty >> 2;
    CCP1CONbits.DC1B = duty & 0x03;

    T2CONbits.TMR2ON = 1;
}

void noTone(void)
{
    CCPR1L = 0;            // Set high 8 bits of Duty Cycle to 0
    CCP1CONbits.DC1B = 0;  // Set low 2 bits of Duty Cycle to 0
}

void music_stop(void) {
    // Stop Timer0
    T0CONbits.TMR0ON = 0;

    // Stop PWM
    noTone();
    
    
    // Clear interrupt flag to avoid immediate ISR trigger
    INTCONbits.TMR0IF = 0;
}

void music_restart(void) {
    note_index = 0;  // Start from the first note
        
    // Play the first note
    if (song == 0) {
        NUM_NOTE = TETRIS_NUM;
        tone(note[note_index]);
    } else {
        NUM_NOTE = PADORU_NUM;
        tone(padoru_note[note_index]);
    }

    // Calculate Timer0 preload (interval unit is ms)
    int _tmp_interval;
    if (song == 0) _tmp_interval = interval[note_index];
    else _tmp_interval = padoru_interval[note_index];
    
    uint32_t counts = (uint32_t)(_tmp_interval) * 1000UL / 128UL;
    if(counts > 65535) counts = 65535;
    uint16_t preload = 65536 - counts;

    TMR0H = preload >> 8;
    TMR0L = preload & 0xFF;

    // Start Timer0
    T0CONbits.TMR0ON = 1;

    // Clear interrupt flag to ensure ISR triggers correctly
    INTCONbits.TMR0IF = 0;
}
