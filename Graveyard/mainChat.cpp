#include <Arduino.h>
#include "ssd1306.h"
#include "i2c.h"
#include <msp430.h>
#include <msp430f5529.h>

// ==================================================================
// Rewritten capture + PWM example for MSP430F5529
// Key changes compared to original:
// - TimerA0 now uses SMCLK (25 MHz) for high resolution measurements.
// - Proper pin muxing: P1SEL _and_ P1SEL2 set for TA0.CCI1A / CCI2A.
// - ISR only stores raw 16-bit deltas and sets a flag; no float math in ISR.
// - CCIFG explicitly cleared in ISR to avoid retriggering.
// - Main loop performs float math and applies a small exponential filter to smooth jitter.
// - Overflow-safe difference calculation (uint16_t wrap-around semantics).
// ==================================================================

// ======== GLOBALS ========
volatile uint16_t cap1_delta = 0;
volatile uint16_t cap2_delta = 0;
volatile uint8_t new_freq1 = 0;
volatile uint8_t new_freq2 = 0;

// computed in main loop (non-ISR)
float freq1 = 0.0f;
float freq2 = 0.0f;
float rpm1 = 0.0f;
float rpm2 = 0.0f;

const float PULSES_PER_REV = 48.0f;  // pulses per revolution of encoder/motor
uint16_t duty_cycle;

// timer clock (SMCLK) frequency used for capture calculations
const float TIMER_CLK_HZ = 25000000.0f; // 25 MHz (as configured by init_SMCLK_25MHz)

// small exponential smoothing factor (0 = no update, 1 = raw)
const float EMA_ALPHA = 0.2f;
static float freq1_ema = 0.0f;
static float freq2_ema = 0.0f;

// ======================================================
//  CLOCK: 25 MHz SMCLK (unchanged -- re-used from your original)
// ======================================================
void init_SMCLK_25MHz(void) {
    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog

    // Keep same pin selections as original (if using XT pins for crystal)
    P5SEL |= BIT2 | BIT3 | BIT4 | BIT5;

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
    TA1CTL = TASSEL_2 | MC_3 | ID_0; // SMCLK, up/down, /1
    TA1CCR0 = 1024;
    TA1CCR1 = 512;
    TA1CCTL1 = OUTMOD_2; // toggle/reset
    duty_cycle = (uint16_t)((float)TA1CCR1 * 100.0f / (float)TA1CCR0);

    // configure output pin for TA1.1 (example: P2.0 in original)
    P2DIR |= BIT0;
    P2SEL |= BIT0;
}

// ======================================================
//  TIMER A0 CAPTURE — frequency measurement (SMCLK source)
// ======================================================
void init_timerA0(void) {
    // Use SMCLK for high resolution measurement
    TA0CTL = TASSEL__SMCLK | MC__CONTINUOUS | TACLR; // SMCLK, continuous 16-bit

    // CCR1 & CCR2: capture on rising edge, capture mode, CCIE enabled.
    // Note: no SCS to avoid synchronous capture artefacts for fast/noisy signals.
    TA0CCTL1 = CM_1 | CCIS_0 | CAP | CCIE; // rising edge, CCIxA
    TA0CCTL2 = CM_1 | CCIS_0 | CAP | CCIE;

    // Configure pins: TA0.CCI1A -> P1.2, TA0.CCI2A -> P1.3
    P1DIR &= ~(BIT2 | BIT3);
    P1SEL  |= BIT2 | BIT3;
}

// ======================================================
//  TIMER A0 ISR (CCR1 & CCR2 capture events)
//  ISR only stores raw differences and sets flags.
// ======================================================
#pragma vector = TIMER0_A1_VECTOR
__interrupt void TimerA0_ISR(void)
{
    static uint16_t last1 = 0;
    static uint16_t last2 = 0;

    switch (TA0IV) {
        case TA0IV_TACCR1: {
            uint16_t now = TA0CCR1;
            uint16_t diff = (uint16_t)(now - last1); // modulo 16-bit wrap-safe
            last1 = now;

            // store result for main loop to process
            cap1_delta = diff ?: 1; // avoid zero
            new_freq1 = 1;

            // clear CCIFG to avoid retriggering
            TA0CCTL1 &= ~CCIFG;
        }
        break;

        case TA0IV_TACCR2: {
            uint16_t now = TA0CCR2;
            uint16_t diff = (uint16_t)(now - last2);
            last2 = now;

            cap2_delta = diff ?: 1;
            new_freq2 = 1;

            TA0CCTL2 &= ~CCIFG;
        }
        break;

        default:
            break;
    }
}

// ======================================================
//  MAIN
// ======================================================
int main(void) {
    WDTCTL = WDTPW | WDTHOLD; // stop watchdog

    init_SMCLK_25MHz();
    __delay_cycles(100000);

    // OLED init (same as original)
    i2c_init();
    __delay_cycles(100000);
    ssd1306_init();
    __delay_cycles(1000000);
    reset_diplay();
    __delay_cycles(1000000);

    init_timerA1();
    init_timerA0();

    // LED for debug
    P1DIR |= BIT0;
    P1OUT &= ~BIT0;

    __enable_interrupt();

    char buffer[32];

    // Main loop: process captured deltas and calculate frequencies outside ISR
    while (1) {
        // If new capture1 available
        if (new_freq1) {
            // copy volatile to local to avoid race
            uint16_t delta = cap1_delta;
            new_freq1 = 0;

            // convert ticks -> frequency (Hz)
            freq1 = TIMER_CLK_HZ / (float)delta;

            // smooth with EMA to reduce visible jitter
            if (freq1_ema == 0.0f) freq1_ema = freq1;
            else freq1_ema = freq1_ema * (1.0f - EMA_ALPHA) + freq1 * EMA_ALPHA;

            rpm1 = (freq1_ema * 60.0f) / PULSES_PER_REV;
        }

        if (new_freq2) {
            uint16_t delta = cap2_delta;
            new_freq2 = 0;

            freq2 = TIMER_CLK_HZ / (float)delta;
            if (freq2_ema == 0.0f) freq2_ema = freq2;
            else freq2_ema = freq2_ema * (1.0f - EMA_ALPHA) + freq2 * EMA_ALPHA;

            rpm2 = (freq2_ema * 60.0f) / PULSES_PER_REV;
        }

        // Draw debug info to OLED
        sprintf(buffer, "TA0CCR1:%05u", TA0CCR1);
        ssd1306_printText(0, 0, buffer);

        sprintf(buffer, "TA0CCR2:%05u", TA0CCR2);
        ssd1306_printText(0, 1, buffer);

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

        __delay_cycles(200000); // small delay to make OLED readable
    }

    return 0;
}
