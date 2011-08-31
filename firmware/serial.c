/* --------------------------------------------------------------------------                                 
    USBCAN Main
    File name: serial.c
    Author: David Snowdon, David Favaloro
    Description: Part of the USBCAN program

    Copyright (C) David Snowdon, David Favaloro 2011. 
    
    Date: 26-03-2011
   -------------------------------------------------------------------------- */

/* 
 * This file is part of Sunswift USBCAN.
 * 
 * Sunswift USBCAN is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 
 * Sunswift USBCAN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 
 * You should have received a copy of the GNU General Public License
 * along with Sunswift USBCAN.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iomacros.h>
#include <signal.h>
#include <msp430x14x.h>

#include <stdlib.h>

#include "hardware.h"

#include "scandal_uart.h"

static char * tx_buf_start;
static char * tx_buf_ptr;
static int tx_buf_size;
volatile int uart0_free;
static char   received_byte; 
volatile char   is_received; 

void
init_uart() {
	UCTL0 = SWRST | CHAR; /* Leave in reset mode, set to 8 bit data (8N1) */
	UTCTL0 = SSEL_SMCLK; /* SMCLK */

	/* This is a workaround as the UART seems to 
	 * automatically generate an interrupt when interrups are enabled
	 */
	tx_buf_ptr = NULL;
	tx_buf_start = NULL;
	tx_buf_size = 0;
	uart0_free = 1;

	/* See mspgcc.sourceforge.net/baudrate.html */
	UART_baud_rate(115200, CLOCK_SPEED); 
	
	ME1 |= UTXE0 | URXE0; /* Module enable Tx and Rx */
/*	P3OUT |= UTXD0; */
	/* Set USART0 Tx high */
/*	P3DIR |= UTXD0; */
	/* Set USART0 Tx to output */
/*	P3SEL |= UTXD0 | URXD0; */
	/* Set USART0 pins to peripheral mode */

	
	UCTL0 &= ~SWRST; /* take out of reset */
	IE1 |= UTXIE0 | URXIE0; /* Enable interrupts for USART0 */
	
}

/* Send the first byte of a buffer to UART0,
 * initialising the pointers used by the UART0 interrupt handler in the process
 */
int
uart0_tx(char * buf, int buf_len) {
	if (uart0_free) {
		uart0_free = 0;
		/* dint ?*/
		tx_buf_ptr = buf;
		tx_buf_start = buf;
		tx_buf_size = buf_len;
		TXBUF0 = *tx_buf_ptr++;
		return 0;
		/* restore interrupts ?*/
	}
	/* else */
	return 1;
}

/* Send the next RS232 byte when ready */
wakeup interrupt (UART0TX_VECTOR) 
uart0_tx_int(void) {
	if (tx_buf_ptr < tx_buf_start + tx_buf_size) {
		TXBUF0 = *tx_buf_ptr++;
	} else {
		uart0_free = 1;
	}
}

interrupt (UART0RX_VECTOR)
uart0_rx_int(void) {
	received_byte = RXBUF0;
	is_received = 1; 
}


/* See http://mspgcc.sourceforge.net/baudrate.html */
extern void UART_baud_rate      (u32 rate, u32 clock_speed){
  if(clock_speed != 7372800)
    return; 

  UART_flush_tx(); 

  //  UCTL0 |= SWRST; /* Put into reset */

  switch(rate){
  case 300:
    UBR00=0x00; UBR10=0x60; UMCTL0=0x00; /* uart0 7372800Hz 1200bps */
    break; 

  case 600:
    UBR00=0x00; UBR10=0x30; UMCTL0=0x00; /* uart0 7372800Hz 1200bps */
    break; 

  case 1200:
    UBR00=0x00; UBR10=0x18; UMCTL0=0x00; /* uart0 7372800Hz 1200bps */
    break; 

  case 2400:
    UBR00=0x00; UBR10=0x0c; UMCTL0=0x00; /* uart0 7372800Hz 2400bps */
    break; 

  case 4800:
    UBR00=0x00; UBR10=0x06; UMCTL0=0x00; /* uart0 7372800Hz 4800bps */
    break; 

  case 9600:
    /* Clock = 7.37Mhz, Baud Rate = 9600bps, Divisor = 768 */
    UBR00=0x00; UBR10=0x03; UMCTL=0x00;
    break; 

  case 38400:
    UBR00=0xC0; UBR10=0x00; UMCTL0=0x00; /* uart0 7372800Hz 38400bps */
    break;

  case 115200:
    /* Clock = 7.37Mhz, Baud Rate = 115200bps, Divisor = 64 */
    UBR00=0x40; UBR10=0x00; UMCTL=0x00;
    break; 
  }

  //  UCTL0 &= ~SWRST; /* take out of reset */
}


extern void UART_SendByte       (u08 data){
  UART_flush_tx(); 
  uart0_tx(&data, 1); 
}

extern void UART_flush_tx       (void){
  while(uart0_free != 1)
    ;  
}

extern u08  UART_ReceiveByte    (void){
  while(!is_received)
    nop();
  
  is_received = 0;
  
  return received_byte; 
}

extern void UART_PrintfProgStr  (u08* buf){
  int i; 

  for(i=0; buf[i] != '\0'; i++)
    ;

  UART_flush_tx(); 

  uart0_tx(buf, i); 
}

extern void UART_PrintfEndOfLine(void){
  UART_flush_tx(); 
  uart0_tx("\n\r", 2); 
}

extern int UART_Init           (void){
 init_uart(); 
 return 0; 
}

extern u08  UART_CheckReceived  (void){
  return is_received;
}

extern u08  UART_is_received    (void){
  return is_received;
}

void UART_PrintfU4(u08 Data){
    /* Send 4-bit hex value */
    u08 Character = Data&0x0f;
    if (Character>9)
    {
        Character+='A'-10;
    }
    else
    {
        Character+='0';
    }
    UART_SendByte(Character);
}

void UART_Printfu08(u08 Data){
    /* Send 8-bit hex value */
    UART_PrintfU4(Data>>4);
    UART_PrintfU4(Data   );
}

void UART_Printfu16(u16 Data){
    /* Send 16-bit hex value */
    UART_PrintfU4(Data>>12);
    UART_PrintfU4(Data>> 8);
    UART_PrintfU4(Data>> 4);
    UART_PrintfU4(Data    );
}
