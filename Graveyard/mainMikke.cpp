#include <Arduino.h>
#include <math.h>
#include <msp430.h>
#include <msp430f5529.h>
#include <stdint.h>
#include <stdio.h>

#include "i2c.h"
#include "ssd1306.h"

// SCALER is how many encoder pulses we get for each motor revolution.
#define SCALER 24.0
// The gear ratio from the motor to the shaft is 30:1.
#define GEAR_RATIO 30.0

// Arbitrary values to avoid saturation and desaturation.
#define TA1CCR1_MAX 800
#define TA1CCR1_MIN 100

float Ga = 5.0;
float Gm = 0.2513;
float step_factor = 3.9793076;
float G = 0;
// Should be ~1920 for 50% duty cycle (TA1CCR1 = 512).
float desired_freq = 600.0;
float freq_error = 0;
int error = 0;
unsigned int TA1CCR1_ph = 0;

// Timer flags for the Timer A0 ISR.
volatile char t_flag = 0;
// Variables to store the captured values from the motor encoders.
unsigned int captured_value = 0;

// freqx is the raw frequency from the motor encoders.
// The actual rotational frequency is freq / SCALER.
float freq = 0;

// Function to set up the OLED-display.
void OLED_init()
{
  i2c_init();
  __delay_cycles(100000);
  ssd1306_init();
  __delay_cycles(100000);
  reset_diplay();
  __delay_cycles(100000);
  ssd1306_printText(0, 0, "Hello world");
}

// Function to initialize the SMCLK to 20 MHz.
void init_SMCLK_20MHz()
{
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
  do
  {
    UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG); // Clear fault flags
    SFRIFG1 &= ~OFIFG;                          // Clear oscillator fault flags
  } while (SFRIFG1 & OFIFG); // Wait until stable

  UCSCTL3 = SELREF__REFOCLK; // Set FLL reference to REFO
  UCSCTL4 = SELA__XT1CLK | SELS__DCOCLK |
            SELM__DCOCLK; // Set ACLK = XT1; SMCLK = DCO; MCLK = DCO
  UCSCTL5 = DIVS__1;      // Set SMCLK divider to 1 (no division)
}

// Function to initialize TimerA0 to run ISR every ms.
void timerA0_capture_init()
{
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
  TA0CCTL1 = CM_1 | CCIS_0 | CAP | CCIE | SCS;
  TA0CCTL2 = CM_1 | CCIS_0 | CAP | CCIE | SCS;

  // Configure P1.2 as the capture input for TA0CCR1.
  // Configure P1.3 as the capture input for TA0CCR2.
  P1DIR &= ~(BIT2 | BIT3);
  P1SEL |= (BIT2 | BIT3);
}

// Function to initialize TimerA1 for center-aligned PWM.
// 50% duty cycle and PWM frequency of 9.760 Hz.
void timerA1_PWM_init()
{
  // Set clock source to SMCLK, f = 19.988.480 Hz.
  // Set Input Divider (ID) to 1.
  // Set Mode Control (MC) to Up/Down mode.
  TA1CTL = TASSEL_2 | ID_0 | MC_3;
  // New frequency: f = 19.988.480 Hz / 1 = 19.988.480 Hz.

  // Output frequency: f = 19.988.480 Hz / (2 * TA1CCR0) = 9.760 Hz
  TA1CCR0 = 1024;

  // Set the output high when TA1R reaches 512.
  TA1CCR1 = 768;

  // Set timerA1 to toggle reset output mode.
  TA1CCTL1 = OUTMOD_2;

  // Duty cycle becomes (TA1CCR1 / TA1CCR0) * 100%:
  // (512 / 1024) * 100% = 50%

  // Route the PWM output to pin P2.0.
  P2DIR |= BIT0;
  P2SEL |= BIT0;
}

int main()
{
  // Initialize SMCLK to 20 MHz.
  init_SMCLK_20MHz();

  // Initialize timer and PWM.
  timerA0_capture_init();
  timerA1_PWM_init();

  // OLED display setup.
  OLED_init();

  // Enable global interrupts
  __enable_interrupt();

  // Variables to store display values.
  float duty_cycle = 0;
  float RPS = 0;
  float shaft_RPS = 0;
  float RPM = 0;
  float shaft_RPM = 0;
  unsigned int counter = 0;

  // Variables necessary for averaging frequencies.
  float freq_av = 0;
  unsigned int i = 0;

  // Buffers etc. for printing.
  char duty_cycle_buffer[32] = {};
  char freq_buffer[32] = {};
  char RPS_buffer[32] = {};
  char RPM_buffer[32] = {};
  char temp_buffer[32] = {};

  // Print logic for duty cycle and motor speed.
  while (1)
  {
    if (t_flag)
    {
      t_flag = 0;
      counter++;

      freq_av += freq;
      i++;

      if (i == 10)
      {
        freq = freq_av / 10.0;
        freq_av = 0.0;
        i = 0;

        // The duty cycle is the ratio between TA1CCR1 and TA1CCR0.
        duty_cycle = (float)(100.0 * TA1CCR1) / TA1CCR0;
        // The RPS is the raw frequency divided by # of pulses per revolution.
        RPS = freq / SCALER;
        // The shaft RPS is the motor RPS divided by the gear ratio.
        shaft_RPS = RPS / GEAR_RATIO;
        // The RPM is the RPS multiplied by 60.
        RPM = RPS * 60.0;
        // The shaft RPM is the motor RPM divided by the gear ratio.
        shaft_RPM = RPM / GEAR_RATIO;

        // Printing with delay.
        if (counter >= 100)
        {
          counter = 0;

          // Print the duty cycle.
          dtostrf(duty_cycle, 0, 2, temp_buffer);
          sprintf(duty_cycle_buffer, "Duty cycle: %s%%", temp_buffer);
          ssd1306_printText(0, 0, duty_cycle_buffer);

          // Print the raw frequency from moter encoder 1.
          dtostrf(freq, 0, 2, temp_buffer);
          sprintf(freq_buffer, "Freq: %sHz    ", temp_buffer);
          ssd1306_printText(0, 1, freq_buffer);

          // Print the RPS of the motor.
          dtostrf(RPS, 0, 2, temp_buffer);
          sprintf(RPS_buffer, "Motor RPS: %s    ", temp_buffer);
          ssd1306_printText(0, 2, RPS_buffer);

          // Print the RPS of the shaft.
          dtostrf(shaft_RPS, 0, 2, temp_buffer);
          sprintf(RPS_buffer, "Shaft RPS: %s    ", temp_buffer);
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
      }
    }
  }
}

// Timer A0 Interrupt Service Routine.
#pragma vector = TIMER0_A1_VECTOR
__interrupt void Timer_A0_ISR(void)
{
  // Variables to store
  static unsigned int last = 0;

  switch (TA0IV)
  {
  case 0x02: // Interrupt caused by CCR1 = P1.2.
             // Handle the captured value for the first encoder pulse.
    // Handle overflow if last is greater than TA0CCR1.
    if (last > TA0CCR1)
    {
      captured_value = 65535 - last + TA0CCR1;
    }
    else
    {
      captured_value = (TA0CCR1 - last);
    }

    if (captured_value <= 0)
    {
      captured_value = 1;
    }

    freq = (float)(32768.0 / captured_value);

    captured_value = 0;
    t_flag = 1;

    G = Ga * Gm;

    freq_error = desired_freq - freq;
    // Can cause errors as the MSP430 is bad at floats.
    error = (int)(freq_error / step_factor);

    TA1CCR1_ph = (unsigned int)(TA1CCR1 + error * G);

    if (TA1CCR1_ph > TA1CCR1_MAX)
    {
      TA1CCR1_ph = TA1CCR1_MAX;
    }
    else if (TA1CCR1_ph < TA1CCR1_MIN)
    {
      TA1CCR1_ph = TA1CCR1_MIN;
    }
    else
    {
      TA1CCR1 = TA1CCR1_ph;
    }

    break;

  case 0x04: // Interrupt caused by CCR2 = P1.3.
             // Handle the captured value for the second encoder pulse.
    last = TA0CCR2;
    break;

  default:
    break;
  }
}