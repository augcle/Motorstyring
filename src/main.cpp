#include <Arduino.h>
#include "ssd1306.h"
#include "i2c.h"
#include <msp430.h>
#include <msp430f5529.h>

// ======== GLOBALS ========
// I denne sektion definerer vi variabler, som skal bruges igennem hele programmet.

/* Variablerne vi bruger til at gemme de to værdier, som vi finder fra vores capture timer A0
Bemærk at de er volatile (Det betyder at vi kan ændre dem i en interrupt service routine)
at de er unsigned (Altså at de kun kan være positive, det bliver vigtigt senere)*/ 
volatile float cap1_delta = 0;
// volatile uint16_t cap2_delta = 0; Denne værdi bliver ikke længere brugt, efter vi skiftede til quad. freq.

/* Variablerne hvor vi gemmer vores beregninger, både frekvensen (Tællinger per sekund)
og rpm (Omgange per minut). Bemærk de er af typen float, så der er plads til kommatal*/
volatile float freq1 = 0.0f;
// volatile float freq2 = 0.0f; Bliver ikke brugt længere
// volatile float rpm1 = 0.0f; Bliver ikke brugt længere
// volatile float rpm2 = 0.0f; Bliver ikke brugt længere. 

/* Variabler hvor vi flagger at der er kommet et interrupt.*/
volatile uint8_t new_freq1 = 0;
// volatile uint8_t new_freq2 = 0; Bliver ikke brugt længere

/* Variabler der bruges til at fastholde en frevkens og beregne hvor meget vi skal regulerer med for at få den ønskede frekvens. 
Der bruges float, fordi vi regner med at få nogle kommatal igennem vores udregninger. */
const float demand_freq = 2000.0; // Målt ved 12V 50PVM.
const float max_freq = 4329.0; // Målt ved 12V 100PWM
const float gain_modulation = 1024 / max_freq; // (GM) En skaleringsfaktor der konverterer vores maskimale tilladte frekvens til en faktor mellem 0 og 1024
const float gain_adjust = 0.25; // (GA) Vi bruger den her, så vi ikke skalere vores modulation med 100% af fejlværdien, men tager det i lidt mindre skridt. Altså udjævne stigningerne, så vi ikke skyder over mål. 
const float gain = gain_adjust*gain_modulation; // Vores endelige gain. Vi bruger den her til at skalare vores målinger, så de falder mellem 0 - 1024. Så vi kan bruge en værdi til at definere TA1CCR1. Altså vores PWM duty cycle.

/* Eventuelle småting */
const float PULSES_PER_REV = 48.0f;  // Tælling på en omgang. Vores motor har 48 rises på en omgang.
// uint16_t duty_cycle; // En unsigned integer til at gemme vores duty-cycle værdi i. Brugesikke længere. 
uint16_t average_freq = 0; // Varibel der bruges til at få et mere stabilt tal i vores målinger
uint16_t display_freq = 0; // Varibel der bruges til at vise vores mere stabile måling
int16_t errorCC = 0; // Varibel der bruges til at gemme hvor meget som TA1CCR1 skal reguleres med.

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
    TA1CCR1 = 512;                    // Tælletiden, der definerer hvornår vores signal toggles. Denne ændres på løbende.
    TA1CCTL1 = OUTMOD_2;              // (out)put (mod)e 2 er Toggle/reset tilstand, hvilket giver at timeren toggler signalet med CCR1, og tæller tilbage igen (på grund af MC3), og toggler så igen ved CCR1. Se side 13 i TimerA user guide. 
    // duty_cycle = (float)(TA1CCR1*100/TA1CCR0); // Vi udregner duty-cyclen i procent. Denne variabel bruges ikke længere. 
}

// ======================================================
//  TIMER A0 CAPTURE — frequency measurement
// ======================================================
void init_timerA0(void) {

    TA0CTL = TASSEL__ACLK | MC__CONTINUOUS | ID_0;  // Det her er opsætningen af timer 0, den som står for at capture vores værdier
                                                     // Tassel_1, ACLK (analog clock), er den clock source der er i microcontrolleren. Den kører på 32.768 Hz.
                                                     // MC_2 ELLER MC__CONTINUOUS tæller fra 0 til 0FFFFh og forfra. 0FFFFh er det samme som 2^16=65535. 
                                                     // TACLR er en function der nulstiller clock divideren. Vi bruger ikke en clockdivider, så det er lidt unødvendigt at nulstille den. 

    // CCR1 Capture, med control register 1
    TA0CCTL1 = CM_1 | CCIS_0 | CAP | CCIE | SCS;    // Capture Mode 1 tilstand betyder at den reagere ved en 'rising edge'
                                                // Capture Compare Input Select 0, 
                                                // CAP = 1 bruges til at sætte den i capture mode
                                                // Capture Compare Interupt Enable betyder at man faktisk kan interrupte den her timer
                                                // Synchronize Capture Source betyder at timeren og input signalet skal være synkroniseret. 
    // CCR2 capture
    TA0CCTL2 = CM_1 | CCIS_0 | CAP | CCIE | SCS;    // Vi opsætter et register 2. Samme overordnede registrer der sættes.  

    // Pinopsætning
    P1DIR &= ~(BIT2 | BIT3);  // Pin 1.2 og 1.3 er vores indgangspins
    P1SEL |= (BIT2 | BIT3); // Sætter dem til analog mode, i stedet for GPIO

    P1REN |= (BIT2 | BIT3);   // Vi sættes vores pins til at have modstand på sig
    P1OUT &= ~(BIT2 | BIT3);  // Og vælger den skal være pull-down. Det betyder at vores signal-pins først registrer noget, hvis spændingen er tilpas høj. Dermed opfanger vi ikke rigig noget støj, fra omgivelserne. 
}


// ======================================================
//  TIMER A0 ISR (CCR1 & CCR2 capture events)
// ======================================================
#pragma vector = TIMER0_A1_VECTOR // Bemærk navnet her. Det er TIMEREN 0 der sætter gang i den her ISR. IKKE timeren 1.  
__interrupt void Timer_A0_ISR(void) 
{
    static uint16_t last1 = 0;  // Vores to variabler hvor vi gemmer den seneste værdi fra CCR1 og CCR2.
                                // Bemærk den er static, hvilket betyder den forbliver i hukommelsen, selv når funktionen er færdig med at køre.
                                // Man kunne også have sat den som en global variabel, det ville have opnået det samme, men så ville alle andre funktioner også kunne have set den, og ændret på den.  
    // static uint16_t last2 = 0; // Ikke i brug længere. 

    switch (TA0IV) // Tilstand baseret på hvad der står i Interupt Vectoren for Timeren 0
    {
        case TA0IV_TACCR1:      // ---- CCR1 event ---- Hvis der står CCR1 i IV. Kan også betegnes som case 2 eller 0x02. 
        {
            uint16_t now = TA0CCR1; // Dette er værdien vi læser lige nu
            uint16_t diff = now - last1; // Vi beregner forskellen mellem nu værdien og den forrige værdi. 
                                        // Bemærk at vi ikke laver noget tjek på om den er større eller lavere 
                                        // end noget bestemt. Det er fordi vi bruger unsigned integers. Så vi vil 
                                        // aldrig kunne få et negativt tal når vi trækker dem fra hinanden. 
                                        // I stedet tæller vi bare op igen, når vi rammer nul, og det giver os 
                                        // stadig den rigtige forskel mellem de tog tal.
            if (diff < 5) {     //Et check-statement der frasorterer 'dårlige' målinger. 
                                // Altså hvis differencen ligger alt for tæt på hinanden, bliver den gemt, men ikke brugt til en beregning
                last1 = now;    // Gem den indlæste værdi i vores seneste værdi. Så vi kan bruge den senere.   
                cap1_delta = 16.384; // Hvis vores difference er meget lille, så sætter vi vores forskel til en kostant. Konstanten kommer fra at tager clock-frekvensen, og dividere med demand frekvensen. På denne måde, vil vi havne med en frekvensforskel på 0, altså at vi har 'ramt' vores demand.  
                break;          // Vi hopper ud af switchen
            }

            last1 = now; // Vi sætter last1 til at være vores nuværende værdi, så vi kan bruge den ved næste interrupt.

            cap1_delta = diff; // Vi gemmer vores difference

            new_freq1 = 1; // Brugt til at sætte et flag, om at der er nye værdier som kan beregnes. Dette er et statement der bliver håndteret i hovedloopet.
        }
        break;

        case TA0IV_TACCR2:      // ---- CCR2 event ---- Hvis der står CC2 i IV. Kan også betegnes som case 4 eller 0x04.
        {
            last1 = TA0CCR2;
        }
        break;

        default: // Ikke strengt nødvendig, men god stil lige at inkludere en default state for switchen. 
            break;
    }
}


// ======================================================
//                        MAIN
// ======================================================
int main() {

    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timeren

    init_SMCLK_25MHz(); // Kør vores submasterclock funktion
    __delay_cycles(100000); // Et lille delay, ikke strengt nødvendig. 

    // OLED init
    // Forbindelsespins til skærmen: SDA 3.0 / SCL 3.1
    i2c_init();
    __delay_cycles(100000);
    ssd1306_init();
    __delay_cycles(1000000);
    reset_diplay();
    __delay_cycles(1000000);

    // Kald timer funktionerne
    init_timerA1();
    init_timerA0();

    // LED for debug, ikke rigtig brugt til noget andet end at se at microcontrolleren har strøm og kan køre koden.
    P1DIR |= BIT0;
    P1OUT |= BIT0;

    // Opsætning af PWM pin
    P2DIR |= BIT0;
    P2SEL |= BIT0;

    __enable_interrupt(); // Vi enabler vores interrupt


    /* Små variabler, der bruges i while loopet*/
    char buffer[20]; // Bruges til at gemme den tekst der skrives til skærmen
    int counter; // Bruges når vi skal beregne vores frekvensgennemsnit
    uint new_duty; // Vi udregner vores duty-cycle, og printer denne varibel.

    while (1) // Her håndtere vi vores matematik, og printer skærmen 
    {
        if (new_freq1==1) // Hvis der er en ny værdi at læse, så laver vi følgende beregnineger
        {
            freq1 = 32768.0f / (float)cap1_delta; // Vi udregner tællinger per sekund. Det gør vi ved at tage ACLK frekvensen og dele med tids-differencen mellem de sidste to tællinger.

            average_freq += freq1; // Vi lægger værdien til vores gennemsnits-variabel
            counter++; // Og øge gennemsnits-tælleren

            if (counter==10) { // Hvis den rammer 10 så
                display_freq = (float)(average_freq / 10); // Tager vi gennemsnittet af variablen, og gemmer den i display_freq
                average_freq=0; // Og nulstiller dem begge
                counter=0;
            }

            errorCC = gain * (demand_freq-freq1); // Her udregner vi den justering der skal foretages på TA1CCR1
            TA1CCR1 = TA1CCR1+errorCC; // Og har tilføjer vi faktisk justeringen

            new_duty = (float)(TA1CCR1*100/TA1CCR0); // Og så udregner vi den nye duty-cycle

            new_freq1=0; // Og clear vores flag
        }

        // Her printer vi reelt set bare alle vores værdier, så vi kan se dem på skærmen
        // Vores demand_freq
        sprintf(buffer, "Demand freq:%04u", (int)demand_freq);
        ssd1306_printText(0, 1, buffer);

        // Encoder-frekvensen vi måler
        sprintf(buffer, "Encoder Freq:%5d", (int)display_freq);
        ssd1306_printText(0, 2, buffer);

        // Vores korrektions-mængde til TA1CCR1
        sprintf(buffer, "Error CC:%05u", errorCC);
        ssd1306_printText(0, 3, buffer);

        // Vores nuværende duty-cycle
        sprintf(buffer, "Duty: %03u%%", new_duty);
        ssd1306_printText(0, 4, buffer);

        // Vores nyværende TA1CCR1 værdi
        sprintf(buffer, "TA1CCR1:%03u", TA1CCR1);
        ssd1306_printText(0, 5, buffer);

        //Denne er ikke i brug
        //sprintf(buffer, "Freq:%04d", (int)(demand_freq-freq1)); 
        //ssd1306_printText(0, 6, buffer);

    }
}
