#include <Arduino.h>
#include <math.h>
#include <msp430.h>
#include <msp430f5529.h>
#include <stdint.h>
#include <stdio.h>

#include "ssd1306.h"
#include "i2c.h"
#include <msp430.h>



// SCALER is how many encoder pulses we get for each motor revolution.
#define SCALER 48.0

// The gear ratio from the motor to the shaft is 30:1.
#define GEAR_RATIO 30.0

// Timer flags for the Timer A0 ISR.
volatile char t_flag1 = 0, t_flag2 = 0;

// Variables to store the time between pulses from the motor encoders.
volatile uint16_t captured_value1 = 0, captured_value2 = 0;
volatile uint16_t last_delta1 = 0;
volatile uint16_t last_delta2 = 0;

// freqx is the raw frequency from the motor encoders.
// The actual rotational frequency is freqx / SCALER.
volatile float freq1 = 0, freq2 = 0;
float freqMax = 525.0;

// Function to set up the OLED-display.
void OLED_init() {
  i2c_init();
  __delay_cycles(100000);
  ssd1306_init();
  __delay_cycles(100000);
  reset_diplay();
  __delay_cycles(100000);
  ssd1306_printText(0, 0, "Hello world");
}

// Function to initialize the SMCLK to 20 MHz.
void init_SMCLK_20MHz() {
  // Stop the watchdog timer
  WDTCTL = WDTPW | WDTHOLD;

  // Select XT2 for SMCLK (Pins 5.2 and 5.3)
  P5SEL |= BIT2 + BIT3;
  P5SEL |= BIT4 + BIT5;

  // Configure DCO to 20 MHz
  // Disable FLL control loop
  __bis_SR_register(SCG0);
  // Set lowest possible DCOx and MODx
  UCSCTL0 = 0x0000;
  // Select DCO range (DCORSEL_7 for max range)
  UCSCTL1 = DCORSEL_7;
  // FLLD = 1, Multiplier N = 762 for ~25 MHz DCO   - 610 for 20MHz
  UCSCTL2 = FLLD_0 + 610;

  // Calculated by f DCOCLK = 32.768 kHz × 610 = 20 MHz
  __bic_SR_register(SCG0); // Enable FLL control loop

  // Loop until XT2, XT1, and DCO stabilize
  do {
    UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG); // Clear fault flags
    SFRIFG1 &= ~OFIFG;                          // Clear oscillator fault flags
  } while (SFRIFG1 & OFIFG); // Wait until stable

  UCSCTL3 = SELREF__REFOCLK; // Set FLL reference to REFO
  UCSCTL4 = SELA__XT1CLK | SELS__DCOCLK |
            SELM__DCOCLK; // Set ACLK = XT1; SMCLK = DCO; MCLK = DCO
  UCSCTL5 = DIVS__1;      // Set SMCLK divider to 1 (no division)
}

// Function to initialize TimerA0 to run ISR every ms.
void timerA0_capture_init() {
  // Set clock source to ACLK, f = 32.768 Hz.
  // Set Input Divider (ID) to 1.
  // Set Mode Control (MC) to Continuous mode (counts to max = 65.535).
  TA0CTL = TASSEL_1 | ID_0 | MC_2;
  // ^ Output frequency becomes  0,5 Hz.

  // CM_1   -> Capture on rising edge.
  // CCIS_0 -> Capture input on CCI0A (P1.2 and P1.3).
  // CAP    -> Set the timer to Capture mode.
  // CCIE   -> Enable the Capture Compare interrupt.
  // SCS    -> Synchronize the capture input signal with the timer clock.
  TA0CCTL1 = CM_3 | CCIS_0 | CAP | CCIE | SCS;
  TA0CCTL2 = CM_3 | CCIS_0 | CAP | CCIE | SCS;

  // Configure P1.2 as the capture input for TA0CCR1.
  // Configure P1.3 as the capture input for TA0CCR2.
  P1DIR &= ~(BIT2 | BIT3);
  P1SEL |= (BIT2 | BIT3);
}

// Function to initialize TimerA1 for center-aligned PWM.
// 50% duty cycle and PWM frequency of 9.760 Hz.
void timerA1_PWM_init() {
  // Set clock source to SMCLK, f = 19.988.480 Hz.
  // Set Input Divider (ID) to 1.
  // Set Mode Control (MC) to Up/Down mode.
  TA1CTL = TASSEL_2 | ID_0 | MC_3;
  // New frequency: f = 19.988.480 Hz / 1 = 19.988.480 Hz.

  // Output frequency: f = 19.988.480 Hz / (2 * TA1CCR0) = 9.760 Hz
  TA1CCR0 = 1024;

  // Set the output high when TA1R reaches 512.
  TA1CCR1 = 512;

  // Set timerA1 to reset/set output mode.
  TA1CCTL1 = OUTMOD_2;

  // Duty cycle becomes (TA1CCR1 / TA1CCR0) * 100%:
  // (512 / 1024) * 100% = 50%

  // Route the PWM output to pin P2.0.
  P2DIR |= BIT0;
  P2SEL |= BIT0;
}

int main() {
  // Initialize SMCLK to 20 MHz.
  init_SMCLK_20MHz();

  // Initialize timer and PWM.
  timerA0_capture_init();
  timerA1_PWM_init();

  // OLED display setup.
  OLED_init();

  // Enable global interrupts
  __enable_interrupt();

  // Variables to
  float duty_cycle = 0.0;
  float RPS = 0.0;
  float RPM = 0.0;
  float shaft_RPM = 0.0;
  unsigned int counter1 = 0;
  unsigned int counter2 = 0;

  // Buffers etc. for printing.
  char duty_cycle_buffer[32] = {};
  char freq_buffer[32] = {};
  char RPS_buffer[32] = {};
  char RPM_buffer[32] = {};
  char temp_buffer[32] = {};

  float freq1_av = 0;
  float freq2_av = 0;
  uint16_t i = 0;
  uint16_t n = 0;

  // Print logic for duty cycle and motor speed.
  while (1) {
    if (t_flag1) {
      t_flag1 = 0;
      counter1++;

      freq1_av += (float)(32768.0 / last_delta1);
      i++;

      if (i >= 10) {
        freq1 = freq1_av / 10.0;
        i = 0;
      }
    }

    if (t_flag2) {
      t_flag2 = 0;
      counter2++;

      freq2_av += (float)(32768.0 / last_delta2);
      n++;

      if (n >= 10) {
        freq2 = freq2_av / 10.0;
      }
    }

    if (counter1 >= 100) {
      counter1 = 0;

      // The RPS is the raw frequency divided by # of pulses per revolution.
      RPS = freq1 / SCALER;
      // The motor's RPM must be its RPS multiplied by 60.
      RPM = RPS * 60.0;
      // The shaft's RPM must be the motor RPM divided by the gear ratio.
      shaft_RPM = RPM / GEAR_RATIO;

      // Print the raw frequency from moter encoder 1.
      dtostrf(freq1, 0, 2, temp_buffer);
      sprintf(freq_buffer, "Freq1: %sHz    ", temp_buffer);
      ssd1306_printText(0, 1, freq_buffer);

      // Print the RPS of the motor.
      dtostrf(RPS, 0, 2, temp_buffer);
      sprintf(RPS_buffer, "Motor RPS: %s    ", temp_buffer);
      ssd1306_printText(0, 3, RPS_buffer);

      // Print the RPM of the motor.
      dtostrf(RPM, 0, 2, temp_buffer);
      sprintf(RPM_buffer, "Motor RPM: %s    ", temp_buffer);
      ssd1306_printText(0, 4, RPM_buffer);

      // Print the RPM of the shaft.
      dtostrf(shaft_RPM, 0, 2, temp_buffer);
      sprintf(RPM_buffer, "Shaft RPM: %s    ", temp_buffer);
      ssd1306_printText(0, 5, RPM_buffer);
    }

    if (counter2 >= 100) {
      counter2 = 0;

      // The RPS is the raw frequency divided by # of pulses per revoltion.
      RPS = freq2 / SCALER;
      // The motor's RPM must be its RPS multiplied by 60.
      RPM = RPS * 60.0;
      // The shaft's RPM must be the motor RPM divided by the gear ratio.
      shaft_RPM = RPM / GEAR_RATIO;

      // Print the raw frequency from motor encoder 2.
      dtostrf(freq2, 0, 2, temp_buffer);
      sprintf(freq_buffer, "Freq2: %sHz    ", temp_buffer);
      ssd1306_printText(0, 2, freq_buffer);

      // Print the RPS of the motor.
      dtostrf(RPS, 0, 2, temp_buffer);
      sprintf(RPS_buffer, "Motor RPS: %s    ", temp_buffer);
      ssd1306_printText(0, 3, RPS_buffer);

      // Print the RPM of the motor.
      dtostrf(RPM, 0, 2, temp_buffer);
      sprintf(RPM_buffer, "Motor RPM: %s    ", temp_buffer);
      ssd1306_printText(0, 4, RPM_buffer);

      // Print the RPM of the shaft.
      dtostrf(shaft_RPM, 0, 2, temp_buffer);
      sprintf(RPM_buffer, "Shaft RPM: %s    ", temp_buffer);
      ssd1306_printText(0, 5, RPM_buffer);
    }

    // The duty cycle is the ratio between TA1CCR0 and TA1CCR1.
    duty_cycle = (float)(100.0 * TA1CCR1) / TA1CCR0;

    // Print the duty cycle.
    dtostrf(duty_cycle, 0, 2, temp_buffer);
    sprintf(duty_cycle_buffer, "Duty cycle: %s%%", temp_buffer);
    ssd1306_printText(0, 0, duty_cycle_buffer);
  }
}

// Timer A0 Interrupt Service Routine.
#pragma vector = TIMER0_A1_VECTOR
__interrupt void Timer_A0_ISR(void) {
  // Variables to store
  static unsigned int last1 = 0, last2 = 0;
  static int i = 0, n = 0;

  switch (TA0IV) {
  case 0x02: // Interrupt caused by CCR1 = P1.2.
             // Handle the captured value for the first encoder pulse.
    captured_value1 = (TA0CCR1 - last1);
    last1 = TA0CCR1;
    i++;
    // Only calculate the frequency every other encoder pulse.
    if (i >= 2) {
      if (captured_value1 != 0) {
        last_delta1 = captured_value1;
      }

      captured_value1 = 0;
      i = 0;
      t_flag1 = 1;
    }
    break;

  case 0x04: // Interrupt caused by CCR2 = P1.3.
             // Handle the captured value for the second encoder pulse.
    captured_value2 = (TA0CCR2 - last2);
    last2 = TA0CCR2;
    n++;
    // Only calculate the frequency every other encoder pulse.
    if (n >= 2) {
      if (captured_value2 != 0) {
        last_delta2 = captured_value2;
      }

      captured_value2 = 0;
      n = 0;
      t_flag2 = 1;
    }
    break;

  default:
    break;
  }
}