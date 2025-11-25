#include <Arduino.h>
#include "ssd1306.h"
#include "i2c.h"
#include <msp430.h>
#include <msp430f5529.h>

// ======== GLOBALS ========
// I denne sektion 

/* Variablerne vi bruger til at gemme de to værdier, som vi finder fra vores capture timer A0
Bemærk at de er volatile (Det betyder at vi kan ændre dem i en interrupt service routine)
at de er unsigned (Altså at de kun kan være positive, det bliver vigtigt senere)*/ 
volatile uint16_t cap1_delta = 0;
volatile uint16_t cap2_delta = 0;

/* Variablerne hvor vi gemmer vores beregninger, både frekvensen (Tællinger per sekund)
og rpm (Omgange per minut). Bemærk de er af typen float, så der er plads til kommatal*/
volatile float freq1 = 0.0f;
volatile float freq2 = 0.0f;
volatile float rpm1 = 0.0f;
volatile float rpm2 = 0.0f;

/* Variabler hvor vi gemmer den nye frekvens, igen volatile, igen unsigned*/
volatile uint8_t new_freq1 = 0;
volatile uint8_t new_freq2 = 0;

/* Eventuelle småting */
const float PULSES_PER_REV = 48.0f;  // Tælling på en omgang. Vores motor har 48 rises på en omgang.
uint16_t duty_cycle; // En unsigned integer til at gemme vores duty-cycle værdi i. 


// ======================================================
//  CLOCK: 25 MHz SMCLK
// ======================================================
void init_SMCLK_25MHz() {
    WDTCTL = WDTPW | WDTHOLD; // Stop watch-dog timeren. Det skal vi bare.

    // De her porte bliver sat til deres analoge funktion.
    P5SEL |= BIT2 + BIT3;
    P5SEL |= BIT4 + BIT5;

    __bis_SR_register(SCG0);
    UCSCTL0 = 0x0000;
    UCSCTL1 = DCORSEL_7;
    UCSCTL2 = FLLD_0 + 610;  
    __bic_SR_register(SCG0);

    do {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
        SFRIFG1 &= ~OFIFG;
    } while (SFRIFG1 & OFIFG);

    UCSCTL3 = SELREF__REFOCLK;
    UCSCTL4 = SELA__XT1CLK | SELS__DCOCLK | SELM__DCOCLK;
    UCSCTL5 = DIVS__1;
}


// ======================================================
//  PWM on Timer A1 — OUTMOD_2 (center-aligned)
// ======================================================
void init_timerA1(void) {
    TA1CTL = TASSEL_2 | MC_3 | ID_0;  // Tassel_2, eller SMCLK (sub master), er en clock vi selv har sat i funktionen over over til 25MHz.
                                      // MC står for Mode Control, og mode control 3 er 'up/down' hvilket betyder den tæller fra 0 op til CCR0 og tilbage igen til 0. Altså en symmetrisk figur omkring CCR0.
                                      // ID_0 er 'input divider', hvilket betyder den dividere clocken. Ret brugbart hvis man skal have flere timere på samme clock der kører lidt forskudt af hinanden. Vi bruge bare ID_0, hvilket betyder vi dividerer med 1.
    TA1CCR0 = 1024;                   // PWM periodetiden, i form a den værdi vi gerne vil tælle op til
    TA1CCR1 = 512;                    // Tælletiden, der definerer hvornår vores signal toggles 
    TA1CCTL1 = OUTMOD_2;              // (out)put (mod)e 2 er Toggle/reset tilstand, hvilket giver at timeren toggler signalet med CCR1, og tæller tilbage igen (på grund af MC3), og toggler så igen ved CCR1. Se side 13 i TimerA user guide. 
    duty_cycle = (float)(TA1CCR1*100/TA1CCR0); // Vi udregner duty-cyclen i procent
}

// ======================================================
//  TIMER A0 CAPTURE — frequency measurement
// ======================================================
void init_timerA0(void) {

    TA0CTL = TASSEL__ACLK | MC__CONTINUOUS | TACLR;  // Det her er opsætningen af timer 0, den som står for at capture vores værdier
                                                     // Tassel_1, ACLK (analog clock), er den clock source der er i microcontrolleren. Den kører på 32kHz cirka.
                                                     // MC_2 ELLER MC__CONTINUOUS tæller fra 0 til 0FFFFh og forfra. 0FFFFh er det samme som vores clock-værdi. 
                                                     // TACLR er en function der nulstiller clock divideren. Vi bruger ikke en clockdivider, så det er lidt mærkeligt at nulstille den. 

    // CCR1 Capture med control register 1
    TA0CCTL1 = CM_1 | CCIS_0 | CAP | CCIE | SCS;    // Capture Mode 1 tilstand betyder at den reagere ved en 'rising edge'
                                                    // Capture Compare Input Select 0, det har noget at gøre med hvilken input pin det skal være?
                                                    // CAP = 1 bruges til at sætte den i capture mode
                                                    // Capture Compare Interupt Enable betyder at man faktisk kan interrupte den her timer
                                                    // Synchronize Capture Source betyder at timeren og input signalet skal være synkroniseret. 
    // CCR2 capture
    TA0CCTL2 = CM_1 | CCIS_0 | CAP | CCIE | SCS;    // Vi opsætter et register 2. 

    P1DIR &= ~(BIT2 | BIT3);  // Pin 1.2 og 1.3 er vores indgangspins, det vælger vi her
    P1SEL |= (BIT2 | BIT3); // Sætter dem til analog mode
}


// ======================================================
//  TIMER A0 ISR (CCR1 & CCR2 capture events)
// ======================================================
#pragma vector = TIMER0_A1_VECTOR // Bemærk navnet her. Det er TIMEREN 0 der sætter gang i den her ISR. IKKE timeren 1.  
__interrupt void TimerA0_ISR(void) 
{
    static uint16_t last1 = 0; // Vores to variabler hvor vi gemmer den seneste værdi fra CCR1 og CCR2
    static uint16_t last2 = 0; // Bemærk at det er en lokalvariabel, så den burde bliver glemt efter hver gang æ. MEN den er static, så den 

    switch (TA0IV) // Tilstand baseret på hvad der står i Interupt Vectoren for Timeren 0
    {
        case TA0IV_TACCR1:      // ---- CCR1 event ---- Hvis der står CCR1 i IV. 
        {
            uint16_t now = TA0CCR1; // Dette er værdien vi læser lige nu
            uint16_t diff = now - last1; // Vi beregner forskellen mellem nu værdien og den forrige værdi. 
                                        // Bemærk at vi ikke laver noget tjek på om den er større eller lavere 
                                        // end noget bestemt. Det fordi vi bruger unsigned integers. Så vi vil 
                                        // aldrig kunne få et negativt tal når vi trækker dem fra hinanden. 
                                        // I stedet tæller vi bare op igen, når vi rammer nul, og det giver os 
                                        // stadig den rigtige forskel mellem de tog tal.
            last1 = now; // Vi sætter last til at være vores nuværende værdi. 

            cap1_delta = diff; // Den målte forskel gemmes, så vi kan bruge den senere. Lige nu bruges den ikke
            freq1 = 32768.0f / (float)diff; // Vi udregner tællinger per sekund. Det gør vi ved at tage ACLK og dele med differencen.
            rpm1 = (freq1 * 60.0f) / PULSES_PER_REV; // Derfra regner vi omgang per minut.  

            new_freq1 = 1; // Ikke i brug endnu. 
        }
        break;

        case TA0IV_TACCR2:      // ---- CCR2 event ---- Hvis der står CC2 i IV.
        {
            uint16_t now = TA0CCR2;
            uint16_t diff = now - last2;
            last2 = now;

            cap2_delta = diff;
            freq2 = 32768.0f / (float)diff;
            rpm2 = (freq2 * 60.0f) / PULSES_PER_REV;

            new_freq2 = 1; // Ikke i brug endnu
        }
        break;

        default:
            break;
    }
}


// ======================================================
//                        MAIN
// ======================================================
int main() {

    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timeren

    init_SMCLK_25MHz(); // Kør vores submasterclock funktion
    __delay_cycles(100000);

    // OLED init
    i2c_init();
    __delay_cycles(100000);
    ssd1306_init();
    __delay_cycles(1000000);
    reset_diplay();
    __delay_cycles(1000000);

    // TIMERS
    init_timerA1();
    init_timerA0();

    // LED for debug
    P1DIR |= BIT0;
    P1OUT |= BIT0;

    // PWM pin
    P2DIR |= BIT0;
    P2SEL |= BIT0;

    __enable_interrupt();

    char buffer[20];

    while (1) // Her printer vi reelt set bare alle vores tal. 
    {
        //sprintf(buffer, "DUTY: %03u%%", duty_cycle);
        //ssd1306_printText(0, 0, buffer);

        // TA0 raw timestamps (debug)
        sprintf(buffer, "TA0CCR1:%05u", TA0CCR1);
        ssd1306_printText(0, 1, buffer);

        sprintf(buffer, "TA0CCR2:%05u", TA0CCR2);
        ssd1306_printText(0, 2, buffer);

        // updated by ISR
        sprintf(buffer, "F1:%5d", (int)freq1);
        //sprintf(buffer, "FREQ1:%6.1f", freq1);
        ssd1306_printText(0, 3, buffer);

        sprintf(buffer, "RPM1:%5d", (int)rpm1);
        ssd1306_printText(0, 4, buffer);

        sprintf(buffer, "F2:%5d", (int)freq2);
        //sprintf(buffer, "FREQ2:%6.1f", freq2);
        ssd1306_printText(0, 5, buffer);

        sprintf(buffer, "RPM2:%5d", (int)rpm2);
        ssd1306_printText(0, 6, buffer);
    }
}
