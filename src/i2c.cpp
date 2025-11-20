#include <msp430.h>
#include "i2c.h"
#include "ssd1306.h"
void i2c_init(void) {
    // Configure pins for I2C (P3.0 -> SDA, P3.1 -> SCL)
    P3SEL |= BIT0 + BIT1;  // Set P3.0 and P3.1 as I2C pins

    // Reset the USCI_B0 module
    UCB0CTL1 |= UCSWRST;

    // Set USCI_B0 in I2C mode, Master mode, Synchronous mode
    UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;

    // Use SMCLK as clock source
    UCB0CTL1 = UCSSEL_2 + UCSWRST;

    // Set the clock frequency (assuming SMCLK is 1 MHz)
    UCB0BR0 = 10;    // SCL frequency = SMCLK / 10 = 100kHz
    UCB0BR1 = 0;
    unsigned long prescaler = smclk_frequency / i2c_frequency;
    UCB0BR0 = prescaler & 0xFF;        // Lower byte of the prescaler
    UCB0BR1 = (prescaler >> 8) & 0xFF; // Upper byte of the prescaler
    // Clear the reset bit to start I2C operation
    UCB0CTL1 &= ~UCSWRST;
}

void i2c_write(unsigned char slave_address, unsigned char *DataBuffer, unsigned char ByteCtr) {
    while (UCB0CTL1 & UCTXSTP);  // Ensure stop condition was sent

    UCB0I2CSA = slave_address;   // Set the slave address
    UCB0CTL1 |= UCTR + UCTXSTT;  // Set as transmitter and generate START condition
    
     while (ByteCtr) {                                                    // Check TX byte counter
                                                 // Load TX buffer
            
    while (!(UCB0IFG & UCTXIFG));  // Wait until the TX buffer is ready
    UCB0TXBUF = *DataBuffer;              // Send the data byte
                DataBuffer++;
    while (!(UCB0IFG & UCTXIFG));  // Wait until the data is transmitted
       ByteCtr--; 
     }
    UCB0CTL1 |= UCTXSTP;           // Generate STOP condition

    while (UCB0CTL1 & UCTXSTP);    // Wait for the STOP condition to be sent
}


unsigned char i2c_read_byte(unsigned char slave_address) {
    unsigned char received_data;

    while (UCB0CTL1 & UCTXSTP);  // Ensure stop condition was sent

    UCB0I2CSA = slave_address;   // Set the slave address
    UCB0CTL1 &= ~UCTR;           // Set as receiver
    UCB0CTL1 |= UCTXSTT;         // Generate START condition

    while (UCB0CTL1 & UCTXSTT);  // Wait for the START condition to be sent
    UCB0CTL1 |= UCTXSTP;         // Generate STOP condition after receiving

    while (!(UCB0IFG & UCRXIFG));  // Wait until the data is received
    received_data = UCB0RXBUF;     // Read the received data

    return received_data;
}

// void setup() {
//     WDTCTL = WDTPW | WDTHOLD;  // Stop watchdog timer

//     i2c_init();  // Initialize I2C

//     // Example: Writing 0x55 to a slave device at address 0x3C
//     i2c_write_byte(0x3C, 0x55);

//     // Example: Reading a byte from a slave device at address 0x3C
//     unsigned char data = i2c_read_byte(0x3C);
// }

// void loop() {
//     // Continuously perform I2C operations in the loop
// }
