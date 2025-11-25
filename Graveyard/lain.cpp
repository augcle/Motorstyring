#include <Arduino.h>
#include "ssd1306.h"
#include "i2c.h"
#include <msp430.h>
#include <msp430f5529.h>

int captured_value1 = 0;
int captured_value2 = 0;


float freq1 = 0, freq2 = 0;
float freqMax = 525.0;

volatile char t_flag1 = 0, t_flag2 = 0;

// Opsætning af vores Sub Master Clock (SMC) til at køre 25MHz
void init_SMCLK_25MHz() {
    WDTCTL = WDTPW | WDTHOLD; // Stop the watchdog timer
  // Vi vælger at port 5.2 og 5.3 skal være aktiverede til deres analoge funktion
  P5SEL |= BIT2 + BIT3; // Select XT2 for SMCLK (Pins 5.2 and 5.3)
  P5SEL |= BIT4 + BIT5;


  // Configure DCO to 25 MHz
  __bis_SR_register(SCG0); // Disable FLL control loop. Vi skal gøre det her for at få lov til at redigere i registeret.
  UCSCTL0 = 0x0000;        // Set lowest possible DCOx and MODx
  UCSCTL1 = DCORSEL_7;     // Select DCO range (DCORSEL_7 for max range)
  UCSCTL2 = FLLD_0 + 610;  // FLLD = 1, Multiplier N = 762 for ~25 MHz DCO   - 610 for 20Mhz
  // calculated by f DCOCLK  =32.768kHz×610=20MHz
  __bic_SR_register(SCG0); // Enable FLL control loop. Husk at enable igen, for at lukke registeret korrekt. 

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

void init_timerA1(void) { // Opsætning af vores timeren A1. Bruges til at styre PWM signalet ud på P2.0.
    TA1CTL = TASSEL_2 | MC_3 | ID_0; // Tassel 2 er clock-kilden "Sub Master Clock" som kører på 25MHz. MC1 står for timer mode, og 3 betyder up/down, altså center aligned. TACLR betyder timer clear.
    TA1CCR0 = 1024; // Den skal tælle op til 1024 clock-cyklusser før den resetter.
    TA1CCR1 = 512; // Dette er vores initial 'duty cycle', på 50%. Det betyder at vi tæller op til 512, som er halvdelen af vores frekvens på 1024. 
    TA1CCTL1 = OUTMOD_2; // Den her bruger vi for at fortæller, at vi bruger output mode 2. Det betyder vi toggler selve outputtet ved CCR1, og resetter timeren ved CCR0. 
    //P2DIR |= BIT0;      // Sæt P2.0 som output
    //P2SEL |= BIT0;      // Sæt P2.0 til dens PWM funktion
}

void init_timerA0(void) { // Vores anden timer, A0. Den holder styr på at hente værdien fra vores input pin, altså capture. 
    TA0CTL = TASSEL_1 | MC_2 | ID_0; // Aclock 
    
    TA0CCTL1 = CM_1 | CCIS_0 | CAP | CCIE | SCS;
    TA0CCTL2 = CM_1 | CCIS_0 | CAP | CCIE | SCS;
    
    P1DIR &= ~(BIT2 | BIT3);
    P1SEL |= (BIT2 | BIT3); 
     
}

#pragma vector=TIMER0_A1_VECTOR
__interrupt void Timer_A1_ISR(void) {

    static unsigned int last1 = 0;
    static int i = 0, n = 0;
    static unsigned int last2 = 0;

    switch(TA0IV) {
        case 0x02:

            if (last1 > TA0CCR1)
            captured_value1 = 65535-last1 + TA0CCR1;
            else
            captured_value1 = (TA0CCR1 - last1);
            last1 = TA0CCR1; 
            
            P2OUT ^= BIT2;
            i++;

            if (i == 2)
            {
            freq1 = (float)(32768.0 / captured_value1);

            captured_value1 = 0;
            t_flag1 = 1;
            i = 0;
            }

        break;
        case 0x04:

            if (last2 > TA0CCR2)
             captured_value2 = 65535-last2 + TA0CCR2;
            else
            captured_value2 = (TA0CCR2 - last2);
            last2 = TA0CCR2;

            P2OUT ^= BIT3;
            n++;

            if (n == 2)
            {
            freq2 = (float)(32768.0 / captured_value2);
            
            captured_value2 = 0;
            t_flag2 = 1;
            n = 0;
            }

        break;
        default:
        break;
    }
}

int main() {
    WDTCTL = WDTPW | WDTHOLD; // Stop the watchdog timer.

    // Sæt sub-master-clock til 25MHz (ish)
    init_SMCLK_25MHz();
    __delay_cycles(100000); // Lille delay

    // Skærmopsætning
    i2c_init();             
    __delay_cycles(100000); // Lille delay
    ssd1306_init();
    __delay_cycles(1000000); // Lille delay
    reset_diplay();
    __delay_cycles(1000000); // Lille delay
    

    // Timeropsætning
    init_timerA1();
    init_timerA0();

    P1DIR |= BIT0; // internal LED p1.0 as output
    P1OUT |= BIT0;

    P2DIR |= BIT3 | BIT2 | BIT0;
    P2SEL |= BIT0; // enable PWM output

    unsigned int Xd = 0;

    __enable_interrupt();
    float Gm = 1;
    float Gin = 0;

    float Xe, Xf;
    int error = 0;
    float G = 1;
    float adc_res_av = 0;
    int k = 0, m = 0, n = 0;
    char data[12];
    unsigned int freq1_av, freq2_av;

    char buffer[20];
    unsigned int duty_cycle = 50;

  while (1) {

    sprintf(buffer, "DUTY: %03u%%", duty_cycle); 
    ssd1306_printText(0, 0, buffer);

    sprintf(buffer, "TA0CCR1:%06u", TA0CCR1); 
    ssd1306_printText(0, 1, buffer);

    sprintf(buffer, "FREQ1:%06u", freq1); 
    ssd1306_printText(0, 2, buffer);

    sprintf(buffer, "TA0CCR2:%06u", TA0CCR2); 
    ssd1306_printText(0, 3, buffer);

    sprintf(buffer, "FREQ2:%06u", freq2); 
    ssd1306_printText(0, 4, buffer);

    if (t_flag1) {
        t_flag1=0;
    }
    
    if (t_flag2){
        t_flag2=0;
	}
  }

}