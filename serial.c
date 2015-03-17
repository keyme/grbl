/*
  serial.c - Low level functions for sending and recieving bytes via the serial port
  Part of Grbl

  Copyright (c) 2011-2014 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

/* This code was initially inspired by the wiring_serial module by David A. Mellis which
   used to be a part of the Arduino project. */ 

#include <avr/interrupt.h>
#include "system.h"
#include "serial.h"
#include "motion_control.h"
#include "protocol.h"
#include "report.h"


uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_buffer_head = 0;
volatile uint8_t rx_buffer_tail = 0;

uint8_t tx_buffer[TX_BUFFER_SIZE];
uint8_t tx_buffer_head = 0;
volatile uint8_t tx_buffer_tail = 0;

uint8_t checksum = 0;  //sum all bytes between newlines.


void serial_init()
{
  // Set baud rate
  #if BAUD_RATE < 57600
    uint16_t UBRR0_value = ((F_CPU / (8L * BAUD_RATE)) - 1)/2 ;
    UCSR0A &= ~(1 << U2X0); // baud doubler off  - Only needed on Uno XXX
  #else
    uint16_t UBRR0_value = ((F_CPU / (4L * BAUD_RATE)) - 1)/2;
    UCSR0A |= (1 << U2X0);  // baud doubler on for high baud rates, i.e. 115200
  #endif
  UBRR0H = UBRR0_value >> 8;
  UBRR0L = UBRR0_value;
            
  // enable rx and tx
  UCSR0B |= 1<<RXEN0;
  UCSR0B |= 1<<TXEN0;
	
  // enable interrupt on complete reception of a byte
  UCSR0B |= 1<<RXCIE0;
	  
  // defaults to 8-bit, no parity, 1 stop bit
}



void serial_sendchar(uint8_t data) {
  // Calculate next head
  uint8_t next_head = tx_buffer_head + 1;
  if (next_head == TX_BUFFER_SIZE) { next_head = 0; }

  // Wait until there is space in the buffer
  while (next_head == tx_buffer_tail) { 
    if (SYS_EXEC & EXEC_RESET) { return; } // Only check for abort to avoid an endless loop.
  }
  // Store data and advance head
  tx_buffer[tx_buffer_head] = data;
  tx_buffer_head = next_head;
  
  // Enable Data Register Empty Interrupt to make sure tx-streaming is running
  UCSR0B |=  (1 << UDRIE0); 
}

void serial_write(uint8_t data) {
  if (data == '\n') {
    serial_sendchar(checksum);
    checksum = 0;
  }
  else { 
    checksum+=data;
  }
  serial_sendchar(data);

}



// Data Register Empty Interrupt handler
ISR(SERIAL_UDRE)
{
  uint8_t tail = tx_buffer_tail; // Temporary tx_buffer_tail (to optimize for volatile)
  
  { 
    // Send a byte from the buffer	
    UDR0 = tx_buffer[tail];
  
    // Update tail position
    tail++;
    if (tail == TX_BUFFER_SIZE) { tail = 0; }
  
    tx_buffer_tail = tail;
  }
  
  // Turn off Data Register Empty Interrupt to stop tx-streaming if this concludes the transfer
  if (tail == tx_buffer_head) { UCSR0B &= ~(1 << UDRIE0); }
}


uint8_t serial_read()
{
  uint8_t tail = rx_buffer_tail; // Temporary rx_buffer_tail (to optimize for volatile)
  if (rx_buffer_head == tail) {
    return SERIAL_NO_DATA;
  } else {
    uint8_t data = rx_buffer[tail];
    
    tail++;
    if (tail == RX_BUFFER_SIZE) { tail = 0; }
    rx_buffer_tail = tail;

    return data;
  }
}


ISR(SERIAL_RX)
{
  uint8_t data = UDR0;
  uint8_t next_head;
  
  // Pick off runtime command characters directly from the serial stream. These characters are
  // not passed into the buffer, but these set system state flag bits for runtime execution.
  switch (data) {
    case CMD_COUNTER_REPORT: request_report(REQUEST_COUNTER_REPORT,0); break; 
    case CMD_VOLTAGE_REPORT: request_report(REQUEST_VOLTAGE_REPORT,0); break; 
    case CMD_STATUS_REPORT: request_report(REQUEST_STATUS_REPORT,0); break;
    case CMD_LIMIT_REPORT: request_report(REQUEST_LIMIT_REPORT,0); break; 
    case CMD_CYCLE_START: SYS_EXEC |= EXEC_CYCLE_START; break; // Set as true
    case CMD_FEED_HOLD:  SYS_EXEC |= EXEC_FEED_HOLD; break; // Set as true
    case CMD_RESET:     mc_reset(); break; // Call motion control reset routine.
    default: // Write character to buffer    
      next_head = rx_buffer_head + 1;
      if (next_head == RX_BUFFER_SIZE) { next_head = 0; }
    
      // Write data to buffer unless it is full.
      if (next_head != rx_buffer_tail) {
        rx_buffer[rx_buffer_head] = data;
        rx_buffer_head = next_head;    
        
      }
      //TODO: else alarm on overflow?
  }
}


void serial_reset_read_buffer() 
{
  rx_buffer_tail = rx_buffer_head;
}
