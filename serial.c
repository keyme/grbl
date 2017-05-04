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
#include "gqueue.h"

DECLARE_QUEUE(tx_buf, uint8_t, TX_BUFFER_SIZE);
DECLARE_QUEUE(rx_buf, uint8_t, RX_BUFFER_SIZE);

static uint8_t checksum = 0;  //sum all bytes between newlines.

void serial_init()
{
  queue_init(&tx_buf, sizeof(uint8_t), TX_BUFFER_SIZE);
  queue_init(&rx_buf, sizeof(uint8_t), RX_BUFFER_SIZE);

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

void serial_sendchar(uint8_t data) 
{
  // As this is an interrupt driven UART, we can simply spin forever
  // and the service routine will drain the queue until there is
  // enough room
  while (queue_is_full(&tx_buf));

  queue_enqueue(&tx_buf, &data);

  // Enable Data Register Empty Interrupt to make sure tx-streaming is running
  UCSR0B |= (1 << UDRIE0);
}

void serial_write(uint8_t data)
{
  checksum += data;
  serial_sendchar(data);
  if (data == '\n') {
    serial_sendchar(checksum);
    checksum = 0;
  }
}

// Data Register Empty Interrupt handler
ISR(SERIAL_UDRE)
{
  uint8_t data = 0;

  queue_dequeue(&tx_buf, &data);
  
  // Send a byte from the buffer
  UDR0 = data;

  // Turn off Data Register Empty Interrupt to stop tx-streaming if this concludes the transfer
  if (queue_is_empty(&tx_buf)) {
    UCSR0B &= ~(1 << UDRIE0);
  }
  
}

// Read data from rx_buffer at tail value 
uint8_t serial_read()
{
  if (queue_is_empty(&rx_buf)) {
    return SERIAL_NO_DATA;
  }
  
  uint8_t data = 0;
  queue_dequeue(&rx_buf, &data);
  return data;
}

ISR(SERIAL_RX)
{
  uint8_t data = UDR0;

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
    if (!queue_is_full(&rx_buf)) {
      queue_enqueue(&rx_buf, &data);
    }
  }
}

void serial_reset_read_buffer()
{
  queue_init(&rx_buf, sizeof(uint8_t), RX_BUFFER_SIZE);
}
