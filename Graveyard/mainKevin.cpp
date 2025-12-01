#include <Arduino.h>
#include "i2c.h"
#include "ssd1306.h"
#include <msp430.h>

// i sætter kald af initialiserings funktioner fra opgave 4 kode ind over while(1)
volatile char t_flag1 = 0;
volatile char sat_hi = 0, sat_lo = 0; // duty saturation indicators
// Simple open-loop max test after startup
#define MAX_TEST_TICKS 20  // 2s @ 10 Hz
static unsigned int max_test_counter = 0;
static float f1_peak = 0.0f;

int captured_value1 = 0;
float freq1 = 0, freq2 = 0;
float freq_quadrature = 0;  // Quadrature frekvens (A til B)
volatile unsigned int time_A = 0;  // Tidsstempel for kanal A
volatile unsigned int time_B = 0;  // Tidsstempel for kanal B

// Simpel PI-controller for robust fejlfinding
#define SIMPLE_PI 1

// PI Controller parametre - NÆSTEN REN P-CONTROLLER
float S_demand = 400.0;     // Ønsket encoder frekvens (Hz)
unsigned int duty_cmd = 512; // PWM duty cycle command
volatile char control_flag = 0;
volatile unsigned int startup_counter = 0;  // Startup delay counter
float freq1_avg = 0.0;      // Averaged frequency for smoother control
float I_s = 0.0f;           // SIMPLE_PI integrator state (vises på display)

void init_SMCLK_25MHz()
{
  WDTCTL = WDTPW | WDTHOLD; // Stop the watchdog timer

  P5SEL |= BIT2 + BIT3; // Select XT2 for SMCLK (Pins 5.2 and 5.3)
  P5SEL |= BIT4 + BIT5;
  // Configure DCO to 25 MHz
  __bis_SR_register(SCG0); // Disable FLL control loop
  UCSCTL0 = 0x0000;        // Set lowest possible DCOx and MODx
  UCSCTL1 = DCORSEL_7;     // Select DCO range (DCORSEL_7 for max range)
  UCSCTL2 = FLLD_0 + 610;  // FLLD = 1, Multiplier N = 762 for ~25 MHz DCO   - 610 for 20Mhz
  // calculated by f DCOCLK  =32.768kHz×610=20MHz
  __bic_SR_register(SCG0); // Enable FLL control loop

  // Loop until XT2, XT1, and DCO stabilize
  do
  {
    UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG); // Clear fault flags
    SFRIFG1 &= ~OFIFG;                          // Clear oscillator fault flags
  } while (SFRIFG1 & OFIFG); // Wait until stable

  UCSCTL3 = SELREF__REFOCLK;                            // Set FLL reference to REFO
  UCSCTL4 = SELA__XT1CLK | SELS__DCOCLK | SELM__DCOCLK; // Set ACLK = XT1; SMCLK = DCO; MCLK = DCO
  UCSCTL5 = DIVS__1;                                    // Set SMCLK divider to 1 (no division)
}

void init_pwm()
{
  // SMCLK = 20 MHz (fra init_SMCLK_25MHz)
  // Center-aligned PWM i up/down mode:
  // f_PWM = f_source / (IDx × 2 × CCR0)
  // Med f_source = 20 MHz, IDx = 1, CCR0 = 1024:
  // f_PWM = 20MHz / (1 × 2 × 1024) = 9765.625 Hz ≈ 9.77 kHz
  TA1CCR0  = 1024;              // 10-bit opløsning (2^10)
  TA1CCR1  = 512;               // 50% duty cycle start

  // Brug reset/set (OUTMOD_7) for klar PWM-polaritet i up/down mode
  TA1CCTL1 = OUTMOD_7;

  // Timer1_A: SMCLK, up/down mode, clear timer (IDx = 1 implicit)
  TA1CTL   = TASSEL_2 | MC_3 | TACLR;
}

void init_capture()
{
  // --- Opsæt input pins med pull-down ---
  P1DIR &= ~(BIT2 | BIT3);      // P1.2 og P1.3 som input
  P1REN |= (BIT2 | BIT3);       // Enable pull resistor
  P1OUT &= ~(BIT2 | BIT3);      // Pull-down (trækker til GND for at undgå støj)
  P1SEL |=  (BIT2 | BIT3);      // Primær periferi funktion (TA0.1, TA0.2)

  // --- Timer_A0 opsætning ---
  TA0CTL = TASSEL_1 | MC_2 | TACLR;   // ACLK (32768 Hz), Continuous mode, clear timer

  // --- CCR1: Capture på P1.2 (TA0.1) ---
  TA0CCTL1 = CM_1        // Fang RISING edge
           | CCIS_0      // Capture input = CCI1A (P1.2)
           | SCS         // Synkron capture
           | CAP         // Capture mode
           | CCIE;       // Enable interrupt

  // --- CCR2: Capture på P1.3 (TA0.2) ---
  TA0CCTL2 = CM_1        // Rising edge
           | CCIS_0      // Capture input = CCI2A (P1.3)
           | SCS
           | CAP
           | CCIE;
  
  // --- CCR0: Periodic interrupt for PI controller (10 Hz) ---
  // ACLK = 32768 Hz, CCR0 = 3277 → interrupt hver 0.1s (10 Hz)
  TA0CCR0 = 3277;
  TA0CCTL0 = CCIE;        // Enable CCR0 interrupt
}

int main()
{
  WDTCTL = WDTPW | WDTHOLD; // always disable watch for using delay function
  init_SMCLK_25MHz();       // set 25 MHz for SMCLK
  // kald initialiserings funktionerne ADC12, initPWM
  __delay_cycles(100);
  i2c_init();
  __delay_cycles(100);
  ssd1306_init();
  __delay_cycles(20);
  reset_diplay();
  __delay_cycles(20);
  ssd1306_clearDisplay();
  __delay_cycles(20);
  ssd1306_printText(0, 1, "Ready...");
  ssd1306_printText(0, 2, "Waiting signal");
  
  // Konfigurer P2.0 FØRST
  P2DIR |= BIT0;               // P2.0 som output
  P2SEL |= BIT0;               // P2.0 som TA1.1 PWM funktion
  
  // START PWM timer
  init_pwm();
 
  init_capture();
  P1DIR |= BIT0; // internal LED p1.0 as output
  P4DIR |= BIT7; // internal LED on P4.7 as output

  P2DIR |= BIT3 | BIT2;        // P2.2 og P2.3 som output (debug LED)

  unsigned int Xd = 0;

  __enable_interrupt();
  char data[20];
  // Rate limiting counters
  static unsigned int update_counter1 = 0;
  
  // Start med 50% duty cycle
  TA1CCR1 = 512;
  
  while (1)
  {
    if (t_flag1)
    {
      t_flag1 = 0; // Clear flag
      update_counter1++;
      
      // Opdater kun hver 10. gang for at undgå flimmer
      if (update_counter1 >= 10) {
        update_counter1 = 0;
        
        // Konverter float til int (sprintf %f virker ikke på MSP430)
        int f1_int = (int)freq1;
        int sp_int = (int)S_demand;
        // Vis den effektive duty baseret på control-kommandoen (ikke den inverterede CCR1)
        int duty_percent = (duty_cmd * 100) / TA1CCR0;      // kommanderet duty (0..100)
        int duty_eff_pct = 100 - duty_percent;              // effektiv duty ved invers mapping
        
        __delay_cycles(20000);
        
        sprintf(data, "F1:%d SP:%d  ", f1_int, sp_int);
        ssd1306_printText(0, 0, data);
        
        int error_int = (int)(freq1 - S_demand);
        sprintf(data, "E:%d CCR:%d     ", error_int, TA1CCR1);
        ssd1306_printText(0, 1, data);
        
        int int_val = (int)I_s;
        sprintf(data, "I:%d D:%d R:%d   ", int_val, duty_percent, duty_eff_pct);
        ssd1306_printText(0, 2, data);

        // Max speed and saturation / cap
        int mx = (int)f1_peak;
        if (sat_hi) {
          sprintf(data, "MX:%d HI  ", mx);
        } else if (sat_lo) {
          sprintf(data, "MX:%d LO  ", mx);
        } else {
          sprintf(data, "MX:%d     ", mx);
        }
        ssd1306_printText(0, 3, data);
      }
    }
    
    // Kort delay i main loop
    __delay_cycles(100000);
  }
}

#pragma vector = TIMER0_A1_VECTOR
__interrupt void Timer_A1_ISR(void)
{
  static unsigned int last1 = 0;
  static int i = 0, n = 0;
  static unsigned int last2 = 0;
  switch (TA0IV)
  {
  case 0x02: // CCR1  P1.2 - Kanal A (rising edge)
    P2OUT ^= BIT2;
    
    time_A = TA0CCR1;  // Gem tidsstempel for kanal A
    i++;
    
    // Beregn frekvens hver 2. edge (fuld periode)
    if (i >= 2) {
      // Beregn periode fra forrige måling (2 edges = fuld periode)
      if (last1 > TA0CCR1)
        captured_value1 = 65535 - last1 + TA0CCR1;
      else
        captured_value1 = (TA0CCR1 - last1);
      
      // Validér måling (undgå ekstreme værdier)
      if (captured_value1 > 10 && captured_value1 < 60000) {
        freq1 = (float)(32768.0 / captured_value1);
      }
      
      last1 = TA0CCR1;
      captured_value1 = 0;
      i = 0;
      t_flag1 = 1;
    }
    break;
    
  case 0x04: // CCR2 (P1.3) - Kanal B (rising edge)
    P2OUT ^= BIT3;
    
    time_B = TA0CCR2;  // Gem tidsstempel for kanal B
    
    // Kun beregn hvis A kom før B (korrekt rækkefølge)
    if (time_A > 0) {
      // Beregn tid fra A til B (quadrature periode)
      int time_diff;
      if (time_B > time_A)
        time_diff = time_B - time_A;
      else
        time_diff = 65535 - time_A + time_B;
      
      // Ignorer hvis tid er urimeligt (støj eller forkert rækkefølge)
      if (time_diff > 5 && time_diff < 30000) {
        // Quadrature frekvens = reciprok af tid A→B
        freq_quadrature = (float)(32768.0 / time_diff);
      }
    }
    
    break;
  default:
    break;
  }
}

// Timer A0 CCR0 interrupt - Periodic PI controller (10 Hz)
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A0_CCR0_ISR(void)
{
  // Re-schedule next 10 Hz tick in continuous mode
  TA0CCR0 += 3277;  // 32768 Hz / 3277 ≈ 10 Hz

  P4OUT ^= BIT7;  // Debug LED toggle (10 Hz blink)
  
  // State for SIMPLE_PI og slew
  static int duty_prev = 650;
  
  // Startup delay - vent 1.5 sekunder før PI aktiveres
  if (startup_counter < 15) {  // 15 * 0.1s = 1.5s
    startup_counter++;
    TA1CCR1 = 650;  // Start med fast 63% duty cycle
    freq1_avg = freq1;  // Opdater average hele tiden under startup
    I_s = 0.0f;
    return;
  }

  // Efter startup: tvungen max-duty i kort tid for at måle peak Hz
  if (max_test_counter < MAX_TEST_TICKS) {
    if (freq1 > f1_peak) f1_peak = freq1;
    duty_cmd = 1000;
    TA1CCR1 = (unsigned int)(TA1CCR0 - duty_cmd);
    max_test_counter++;
    return;
  }
  
  // Kun reguler hvis motor kører (freq1 > 10 Hz)
  if (freq1 < 10) {
    TA1CCR1 = 650;  // Kickstart med 63% duty
    I_s = 0.0f;   // Reset integrator
    freq1_avg = freq1;
    return;
  }
  
  // Smooth freq1 med moving average KUN til display (ikke til regulering)
  freq1_avg = 0.7 * freq1_avg + 0.3 * freq1;
  // Opdater peak løbende (ikke kun i max-test)
  if (freq1 > f1_peak) f1_peak = freq1;

  // Enkel PI: positiv fejl → øg duty
  const float Kp_s = 1.2f;
  const float Ki_s = 0.08f; // pr 0.1s tick: hurtigere udjævning af offset

  float error_s = (S_demand - freq1); // brug rå måling for hurtigere respons

  // Foreløbigt u
  float u = (float)duty_prev + (Kp_s * error_s) + I_s;
  int duty_signed = (int)u;
  if (duty_signed < 250) duty_signed = 250;
  if (duty_signed > 1024) duty_signed = 1024; // tættere på toppunktet (CCR0=1024)

  // Anti-windup: integrer kun hvis ikke skubber ind i saturation
  if (!((duty_signed >= 1015 && error_s > 0.0f) || (duty_signed <= 250 && error_s < 0.0f))) {
    I_s += Ki_s * error_s;
    if (I_s > 400.0f) I_s = 400.0f;
    if (I_s < -400.0f) I_s = -400.0f;
  }

  // Re-beregn u og anvend blid slew
  u = (float)duty_prev + (Kp_s * error_s) + I_s;
  duty_signed = (int)u;
  if (duty_signed < 250) duty_signed = 250;
  if (duty_signed > 1015) duty_signed = 1015;

  int max_step = (abs((int)error_s) > 80) ? 80 : 24;
  int delta = duty_signed - duty_prev;
  if (delta > max_step) duty_signed = duty_prev + max_step;
  else if (delta < -max_step) duty_signed = duty_prev - max_step;
  
  duty_cmd = (unsigned int)duty_signed;
  // Inverter duty for korrekt hardware-polaritet under OUTMOD_7
  TA1CCR1 = (unsigned int)(TA1CCR0 - duty_cmd);

  // Opdater satureringsflag til display
  sat_hi = (duty_signed >= 1015);
  sat_lo = (duty_signed <= 250);

  // Opdater state for næste tick
  duty_prev = duty_signed;
  // SIMPLE_PI: ingen yderligere state
}