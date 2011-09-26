/* --------------------------------------------------------------------------                                 
    USBCAN Main
    File name: usbcan.c
    Author: David Snowdon, David Favaloro
    Description: The main USBCAN program

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

#include <io.h>
#include <signal.h>
#include <iomacros.h>
#include <string.h>
#include <msp430x14x.h>

#include <scandal/timer.h>
#include <scandal/leds.h>
#include <scandal/can.h>
#include <scandal/engine.h>
#include <scandal/spi.h>
#include <scandal/devices.h>
#include <scandal/utils.h>
#include <scandal/message.h>
#include <scandal/types.h>
#include <scandal/uart.h>
#include <scandal/eeprom.h>
#include <scandal/error.h>
#include <scandal/tritium.h>
#include <scandal/stdio.h>

#include <arch/can.h>
#include <arch/uart.h>

#include <project/spi_devices.h>
#include <project/hardware.h>
#include <project/serial.h>

#define WDTCTL_INIT     WDTPW|WDTHOLD

/* Global variables */ 

uint8_t txbuf[32]; 
uint8_t txnum = 0; 

/* ---------------------- 
   CAN to Serial #defines 
   ---------------------- */ 
/* Error codes */
#define ERROR     1

/* ASCII Definitinos */
#define BACKSPACE 0x08
#define ESCAPE    0x1B
#define SPACE     0x20
#define BEEP      0x07
#define DEL       0x7F

#define START_CHARACTER		'\n'
#define AVERAGE_SAMPLES         16

/* Port configuration */
void init_ports(void){
	P1OUT = 0x00;
	P1SEL = 0x00;
	P1DIR = 0x00;
	P1IES = 0x00;
	P1IE  = 0x00;
	
	P2OUT = 0x00;
	P2SEL = 0x00;
	P2DIR = 0x00;
	P2IES = CAN_INT;
	P2IE  = 0x00;
	
	P3OUT = 0x00;
	P3SEL = TX | RX;
	P3DIR = TX;
	
	P4OUT = 0x00;
	P4SEL = 0x00;
	P4DIR = 0x00;
	
	P5OUT = CAN_CS;
	P5SEL = SIMO1 | SOMI1 | UCLK1;
	P5DIR = CAN_CS | SIMO1 | UCLK1 | YELLOW_LED_BIT | RED_LED_BIT;
	
	P6SEL = MEAS_12V_PIN;  
}

void init_clock(void){
	volatile unsigned int i;
	
	/* XTAL = LF crystal, ACLK = LFXT1/1, DCO Rset = 4, XT2 = ON */
	BCSCTL1 = 0x04;
	
	/* Clear OSCOFF flag - start oscillator */
	_BIC_SR( OSCOFF );
	do{
		/* Clear OSCFault flag */
		IFG1 &= ~OFIFG; 
		/* Wait for flag to set */
		for( i = 255; i > 0; i-- )
			;
	} while(( IFG1 & OFIFG ) != 0);
	
	/* Set MCLK to XT2CLK and SMCLK to XT2CLK */
	BCSCTL2 = 0x88; 
}

/*--------------------------------------------------
  Interrupt handling for CAN
  --------------------------------------------------*/
void enable_can_interrupt(){
	P2IE = CAN_INT;
}

void disable_can_interrupt(){
	P2IE = 0x00;
}

interrupt (PORT2_VECTOR) port2int(void) {
	can_interrupt();
	P2IFG = 0x00;
}

/*--------------------------------------------------
  CAN to serial functions
  --------------------------------------------------*/
void encode_raw_in_queue(uint8_t c, int raw){
	if(!raw){
		switch(c){
		case 'q':
		case 'r':
		case 'C':
		case '\\':
			txbuf[txnum++] = '\\';
		}
	}
	
	txbuf[txnum++] = c; 
}

void print_byte(u08 byte){
	u08 b0, b1;
	b1 = ((byte & 0xF0)>>8);
	b0 = (byte & 0xF);
	
	if(b1<0xa)
		UART_SendByte(b1+'0');
	else
		UART_SendByte(b1 - 10 + 'a');
	
	if(b0<0xa)
		UART_SendByte(b0+'0');
	else
		UART_SendByte(b0 - 10 + 'a');
}

void print_int(s32 val);
void print_uint(u32 val);
void serial_send_ascii(u32 id_buf, u08* buf, u08 length);
void serial_send_ascii_std(u32 id_buf, u08* buf, u08 length);
void print_string(u08*	buf);
u32 read_dec_num(void);
u08 read_signed_num(s32* num);

void serial_send_ascii(u32 id, u08* buf, u08 length){
	s32 value;
	u32 uvalue = 0; 
	u08	i;
	
	value = (id >> 18) & 0xFF;
	switch(value){
    case CHANNEL_TYPE:
		UART_printf("C:\t");
		
		value = (id >> 26) & 0x07;
		print_int(value); UART_SendByte('\t');
		
		value = (id >> 18) & 0xFF;
		print_int(value); UART_SendByte('\t');
		
		value = (id >> 10) & 0xFF;
		print_int(value); UART_printf("\t");
		
		value = (id >> 0) & 0x03FF;
		print_int(value); UART_printf("\t");
		
		value = (u32)buf[0] << 24;
		value |= (u32)buf[1] << 16;
		value |= (u32)buf[2] << 8;
		value |= (u32)buf[3] << 0;
		print_int(value); UART_printf("\t");
		
		uvalue = (u32)buf[4] << 24;
		uvalue |= (u32)buf[5] << 16;
		uvalue |= (u32)buf[6] << 8;
		uvalue |= (u32)buf[7] << 0;
		
		print_uint(uvalue); // Time is unsigned
		UART_printf("\r\n");
		
		break;
		
	case HEARTBEAT_TYPE:
		UART_printf("H:\t");
		
		value = (id >> PRI_OFFSET) & 0x07;
		print_int(value); UART_SendByte('\t');
		
		value = (id >> TYPE_OFFSET) & 0xFF;
		print_int(value); UART_SendByte('\t');
		
		value = (id >> HEARTBEAT_NODE_ADDR_OFFSET) & 0xFF;
		print_int(value); UART_printf("\t");
		
		value = (id >> HEARTBEAT_NODE_TYPE_OFFSET) & 0x03FF;
		print_int(value); UART_printf("\t");
		
		value = (u32)buf[0] << 24;
		value |= (u32)buf[1] << 16;
		value |= (u32)buf[2] << 8;
		value |= (u32)buf[3] << 0;
		print_int(value); UART_printf("\t");
		
		uvalue = (u32)buf[4] << 24;
		uvalue |= (u32)buf[5] << 16;
		uvalue |= (u32)buf[6] << 8;
		uvalue |= (u32)buf[7] << 0;
		
		print_uint(uvalue); //Time is unsigned
		UART_printf("\r\n");
		
		break;
		
	case CONFIG_TYPE:
		UART_printf("Config:\t");
		
		value = (id >> PRI_OFFSET) & 0x07;
		print_int(value); UART_SendByte('\t');
		
		value = (id >> TYPE_OFFSET) & 0xFF;
		print_int(value); UART_SendByte('\t');
		
		value = (id >> CONFIG_NODE_ADDR_OFFSET) & 0xFF;
		print_int(value); UART_printf("\t");
		
		value = (id >> CONFIG_PARAM_OFFSET) & 0x03FF;
		print_int(value); UART_printf("\t");
		
		value = (u32)buf[0] << 24;
		value |= (u32)buf[1] << 16;
		value |= (u32)buf[2] << 8;
		value |= (u32)buf[3] << 0;
		print_int(value); UART_printf("\t");
		
		uvalue = (u32)buf[4] << 24;
		uvalue |= (u32)buf[5] << 16;
		uvalue |= (u32)buf[6] << 8;
		uvalue |= (u32)buf[7] << 0;
		
		print_uint(uvalue); //Time is unsigned
		UART_printf("\r\n");
		
		break;
		
	case SCANDAL_ERROR_TYPE:
		UART_printf("SE:\t");
		
		value = (id >> PRI_OFFSET) & 0x07;
		print_int(value); UART_SendByte('\t');
		
		value = (id >> TYPE_OFFSET) & 0xFF;
		print_int(value); UART_SendByte('\t');
		
		value = (id >> SCANDAL_ERROR_NODE_ADDR_OFFSET) & 0xFF;
		print_int(value); UART_printf("\t");
		
		value = (id >> SCANDAL_ERROR_NODE_TYPE_OFFSET) & 0x03FF;
		print_int(value); UART_printf("\t");
		
		value = (u32)buf[0];
		print_int(value); UART_printf("\t");
		
		uvalue = (u32)buf[4] << 24;
		uvalue |= (u32)buf[5] << 16;
		uvalue |= (u32)buf[6] << 8;
		uvalue |= (u32)buf[7] << 0;
		
		print_uint(uvalue); // Time is unsigned
		UART_printf("\r\n");
		
		break;
		
	case USER_ERROR_TYPE:
		UART_printf("UE:\t");
		
		value = (id >> PRI_OFFSET) & 0x07;
		print_int(value); UART_SendByte('\t');
		
		value = (id >> TYPE_OFFSET) & 0xFF;
		print_int(value); UART_SendByte('\t');
		
		value = (id >> USER_ERROR_NODE_ADDR_OFFSET) & 0xFF;
		print_int(value); UART_printf("\t");
		
		value = (id >> USER_ERROR_NODE_TYPE_OFFSET) & 0x03FF;
		print_int(value); UART_printf("\t");
		
		value = (u32)buf[0];
		print_int(value); UART_printf("\t");
		
		uvalue = (u32)buf[4] << 24;
		uvalue |= (u32)buf[5] << 16;
		uvalue |= (u32)buf[6] << 8;
		uvalue |= (u32)buf[7] << 0;
		
		print_int(uvalue);
		UART_printf("\r\n");
		
		break;
		
	case TIMESYNC_TYPE:
		UART_printf("T:\t");
		
		value = (id >> PRI_OFFSET) & 0x07;
		print_int(value); UART_SendByte('\t');
		
		value = (id >> TYPE_OFFSET) & 0xFF;
		print_int(value); UART_SendByte('\t');
		
		{
		  uint64_t bigval; 

		  bigval = (uint64_t)buf[0]; 
		  bigval <<= 8; bigval |= (uint64_t)buf[1];
		  bigval <<= 8; bigval |= (uint64_t)buf[2];
		  bigval <<= 8; bigval |= (uint64_t)buf[3];
		  bigval <<= 8; bigval |= (uint64_t)buf[4];
		  bigval <<= 8; bigval |= (uint64_t)buf[5];
		  bigval <<= 8; bigval |= (uint64_t)buf[6];
		  bigval <<= 8; bigval |= (uint64_t)buf[7];
		  print_uint(bigval / 1000000000);		
		  print_uint(bigval % 1000000000);
		  //print_uint(bigval >> 32); UART_printf("\t");
		  //print_uint(bigval & 0xFFFFFFFF); UART_printf("\n\r");
		}
		UART_printf("\r\n");
		
		break;
		
	default:
		UART_printf("%d", (id >> 24));
		UART_printf("%d", (id>>16));
		UART_printf("%d", (id >> 8));
		UART_printf("%d", (id>>0));
		UART_printf("\t");

		for(i=0; i<8; i++){
			UART_printf("%d ", buf[i]);
		}
		UART_printf("\r\n");
		break;
	}
}

void serial_send_ascii_std(u32 id, u08* buf, u08 length) {
	int i = 0;
	UART_printf("WS:\tid: %d\tvalue:\t", id);
	for(i=0; i<8; i++){
		UART_printf("%d ", buf[i]);
	}
	UART_printf("\n\r");
}

u08 read_signed_num(s32 *num){
	char c;
	u08 neg = 0;
	*num = 0;
	u08 digits = 0;
	
	c = UART_ReceiveByte();
	
	if(c == '-'){
		neg = 1;
		UART_SendByte(c);
		c = UART_ReceiveByte();
	}
	
	while((c != '\n') && (c != '\r') && (c != '\t')){
		if(c == ESCAPE){ 
			UART_printf("\r\n");
			return ERROR;
		}
		else if (c == BACKSPACE && digits > 0){
			UART_SendByte(BACKSPACE);
			UART_SendByte(SPACE);
			UART_SendByte(BACKSPACE);
			
			digits--;
			*num /= 10;
			c = UART_ReceiveByte();
			
		}
		else if((c < '0') || (c > '9')){
			c = UART_ReceiveByte();
			UART_SendByte(BEEP);
			continue;
		}
		else {
			UART_SendByte(c);
			*num *= 10;
			*num += (c - '0');
			digits++;
			c = UART_ReceiveByte();
		}
	}
	
	if(neg == 1)
		*num *= -1;
	return NO_ERR;
}

void print_uint(u32 val){
	u08 buf[10];
	u08 i;
	
	i = 9;
	
	if(val == 0){
		UART_SendByte('0');
		return;
	}
	
	while(val > 0){
		buf[i] = '0' + (val % 10);
		val /= 10;		
		i--;
	}
	
	i++;
	
	for(;i<10;i++){
		UART_SendByte(buf[i]);
	}

}

void print_int(s32 val){
	if(val == 0){
		UART_SendByte('0');
		return;
	}
	
	if(val < 0){
		UART_SendByte('-');
		val *= -1;
		print_uint(val);
	}else
	  print_uint(val);
}

/* Handle a character press */ 
void handle_command(char mode){
	uint32_t num;
	uint32_t dest;
	uint32_t value;
	int32_t signed_val;
	uint32_t param;
	
	uint32_t msg_node, msg_channel, msg_type;
	uint32_t source_node, source_channel;
	
	can_msg      msg;
	uint8_t 		err,i;
	
	s32 average_array[AVERAGE_SAMPLES];
	
    UART_SendByte(mode);
	
    UART_printf("\r\n");
    switch(mode){  
	case '?':
	case 'h':
	case 'H':
        UART_printf("Help:\n\r"
	       "?,h: Show this help\n\r"
	       "l: Listen to one channel\n\r"
	       "o: Observe one node\n\r"
	       "m: Monitor all messages\n\r"
	       "v: Show Average for one channel\n\r"
	       "e: Report error status of serial node\n\r"
	       "c: Send a channel message\n\r"
	       "i: Set up an in-channel\n\r"
	       "b: Change the b scaling parameter for an out-channel\n\r"
	       "x: Change the m scaling parameter for an out-channel\n\r"
	       "a: Change the address of a node\n\r"
	       "u: User configuration\n\r"
	       "r: raw mode -- exit using 'q'\n\r"
	       "t: timesync message\n\r"
	       "d: display timesync messages\n\r"
	       "w: go into wavesculptor mode\n\r"
	       "\n\r");
        break;
		
	case 'l': /* Listen to one channel*/
        UART_printf("Source node: ");
        err = read_signed_num(&source_node);
        if (err != NO_ERR)
			break;
        UART_printf("\t\tSource channel: ");
        err = read_signed_num(&source_channel);
        if (err != NO_ERR)
			break;
        UART_printf("\n\r");
		
		while(!UART_is_received()) {             /* loop forever */
			can_poll();
			
			err = can_get_msg(&msg);
			
			if(err == NO_ERR){
				
				msg_type = (msg.id >> 18) & 0xFF;
				msg_node = (msg.id >> 10) & 0xFF;
				msg_channel = (msg.id >> 0) & 0x03FF;
				
				if((msg_type == 0) && (msg_node == source_node) && (msg_channel == source_channel)){
					serial_send_ascii(msg.id, msg.data, msg.length);
				}
			}else{
				
			}
		}
		UART_ReceiveByte();
		break;
		
    case 'v': /* aVerage*/
		UART_printf("Source node: ");
		err = read_signed_num(&source_node);
		if (err != NO_ERR)
			break;
		UART_printf("\t\tSource channel: ");
		err = read_signed_num(&source_channel);
		if (err != NO_ERR)
			break;
		UART_printf("\n\r");
		
		while(!UART_is_received()) {             /* loop forever */
			s32 average_value=0;
			
			can_poll();
			err = can_get_msg(&msg);
			
			if(err == NO_ERR){
				
				msg_type = (msg.id >> 18) & 0xFF;
				msg_node = (msg.id >> 10) & 0xFF;
				msg_channel = (msg.id >> 0) & 0x03FF;
				
				if((msg_type == 0) && (msg_node == source_node) && (msg_channel == source_channel)){
					
					for(i = 1; i<AVERAGE_SAMPLES; i++)
						average_array[i] = average_array[i-1];
					
					average_array[0] = (u32)msg.data[0] << 24;
					average_array[0] |= (u32)msg.data[1] << 16;
					average_array[0] |= (u32)msg.data[2] << 8;
					average_array[0] |= (u32)msg.data[3] << 0;
					
					for(i = 0; i < AVERAGE_SAMPLES; i++)
						average_value += average_array[i];
					
					UART_printf("Value: ");
					print_int(average_value / AVERAGE_SAMPLES);
					UART_printf("\n\r");
				}
			}else{
				
			}
		}
		UART_ReceiveByte();
		break;
		
		
    case 'o': /* Observe a node */
		UART_printf("Source node: ");
		
		err = read_signed_num(&source_node);
		if (err != NO_ERR)
			break;
		
		while(!UART_is_received()) {             /* loop forever */
			can_poll();
			
			err = can_get_msg(&msg);
			
			if(err == NO_ERR){
				
				msg_node = (msg.id >> 10) & 0xFF;
				
				if(msg_node == source_node){
					serial_send_ascii(msg.id, msg.data, msg.length);
				}
			}else{
				
			}
		}
		UART_ReceiveByte();
		break;
		
    case 'd': /* Observe time synchronisation messages */
		while(!UART_is_received()) {             /* loop forever */
			can_poll();			
			err = can_get_msg(&msg);
			if(err == NO_ERR){
			  int msg_type; 
			        msg_type = (msg.id >> 18) & 0xFF;
				if(msg_type == TIMESYNC_TYPE)
				  serial_send_ascii(msg.id, msg.data, msg.length);
			}else{
				
			}
		}
		UART_ReceiveByte();
		break;
		
    case 'm':
		while(!UART_is_received()) {             /* loop forever */
			can_poll();

			err = can_get_msg(&msg);

			if (msg.ext) {
				if(err == NO_ERR){
					serial_send_ascii(msg.id, msg.data, msg.length);
				}
			} else {
				serial_send_ascii_std(msg.id, msg.data, msg.length);
			}
		}
		UART_ReceiveByte();
		break;

	case 'e':
        while(!UART_is_received()) {             /* loop forever */
			u08 rx_err;
			u08 tx_err;
            
			can_poll();
			
			err = can_get_msg(&msg);
			
			rx_err = MCP2510_read_rx_errors();
			tx_err = MCP2510_read_tx_errors();
			
			if(err != NO_MSG_ERR ||
			   rx_err != 0 ||
			   tx_err != 0){
				
				if(msg.length != 8)
					UART_printf("Length not 8\n\r");
				
				UART_printf("Error: ");
				print_int(err);
				
				UART_printf("\ttx: ");
				print_int(tx_err);
				
				UART_printf("\trx: ");
				print_int(rx_err);
                
				UART_printf("\n\r");
			}
        }
        UART_ReceiveByte();
		break;
		
	case 'c':
		/* Send a channel... */
		UART_printf("Source:");
		err = read_signed_num(&dest);
		if (err != NO_ERR)
            break;
		UART_printf("\tSource#:");
		err = read_signed_num(&num);
		if (err != NO_ERR)
            break;
		UART_printf("\tValue:");
		err = read_signed_num(&value);
		if (err != NO_ERR)
            break;
		
		msg.id = scandal_mk_channel_id(0x00, (dest&0xFF), num);
		
		msg.data[0] = (value >> 24) & 0xFF;
		msg.data[1] = (value >> 16) & 0xFF ;
		msg.data[2] = (value >> 8) & 0xFF;
		msg.data[3] = (value & 0xFF);
		
		value = sc_get_timer();
		msg.data[4] = (value >> 24) & 0xFF;
		msg.data[5] = (value >> 16) & 0xFF ;
		msg.data[6] = (value >> 8) & 0xFF;
		msg.data[7] = (value & 0xFF);
		msg.length = 8;
		UART_printf("\r\n");
		
		can_send_msg(&msg, 0);
		break;
		
	case 'i':
		/* Set up an "in-channel" */
		UART_printf("Node:");	      /* Node number to change */
		err = read_signed_num(&dest);
		if (err != NO_ERR)
            break;
		
		UART_printf("\tIn#: ");    /* In channel number */
		err = read_signed_num(&value);
		if (err != NO_ERR)
            break;
		msg.data[0] = (value >> 8) & 0xFF;
		msg.data[1] = (value & 0xFF);
		
		UART_printf("\tSource:"); /* Source node */
		err = read_signed_num(&value);
		if (err != NO_ERR)
            break;
		msg.data[2] = value & 0xFF;
		
		UART_printf("\tChan#:");  /* Source Num */
		err = read_signed_num(&value);
		if (err != NO_ERR)
            break;
		msg.data[3] = (value >> 8) & 0xFF;
		msg.data[4] = (value & 0xFF);
		
        UART_printf("\r\n");
		
		msg.id = scandal_mk_config_id(0x00, (dest&0xFF), CONFIG_IN_CHAN_SOURCE);
		msg.length = 8;
		
		can_send_msg(&msg, 0);
		break;
		
	case 'b':
	case 'x':
        /* Set up an "in-channel" */
		UART_printf("Node:");	      /* Node number to change */
		err = read_signed_num(&dest);
		if (err != NO_ERR)
            break;
		
		UART_printf("\t#: ");    /* In channel number */
		err = read_signed_num(&value);
		if (err != NO_ERR)
            break;
		
		msg.data[0] = (value >> 8) & 0xFF;
		msg.data[1] = (value & 0xFF);
		
		UART_printf("\tNew "); /* Source node */
		if(mode == 'b')
            UART_printf("B:");
		else
            UART_printf("M:");
		
		err = read_signed_num(&signed_val);
		if (err != NO_ERR)
            break;
		msg.data[2] = (signed_val>>24) & 0xFF;
		msg.data[3] = (signed_val>>16) & 0xFF;
		msg.data[4] = (signed_val>>8) & 0xFF;
		msg.data[5] = (signed_val>>0) & 0xFF;
		
        UART_printf("\r\n");

        if(mode == 'b')
			msg.id = scandal_mk_config_id(0x00, (dest&0xFF), CONFIG_OUT_CHAN_B);
        else
			msg.id = scandal_mk_config_id(0x00, (dest&0xFF), CONFIG_OUT_CHAN_M);
        
        msg.length = 8;
        can_send_msg(&msg, 0);
        break;
		
    case 'a':
		/* Set the address of a node */
		UART_printf("Dest:");
		err = read_signed_num(&dest);
		if (err != NO_ERR)
			break;
		UART_printf("\tAddr:");
		err = read_signed_num(&num);
		if (err != NO_ERR)
			break;
		UART_printf("\r\n");
		
		msg.id = scandal_mk_config_id(0x00, (dest&0xFF), CONFIG_ADDR);
		msg.data[0] = num & 0xFF;
		msg.length = 8;
		
		UART_printf("%d", msg.id >> 24);
		UART_printf("%d", msg.id>>16);
		UART_printf("%d", msg.id >> 8);
		UART_printf("%d", msg.id>>0);
		
		UART_printf("\t");;
		for(i=0; i<8; i++){
			UART_printf("%d ", msg.data[i]);
		}
		UART_printf("\r\n");
		
		can_send_msg(&msg, 1);
		break;
	
    case 'u':
		/* Change a user configuration parameter */
		UART_printf("Node:");    /* Node number to change */
		err = read_signed_num(&dest);
		if (err != NO_ERR)
			break;
		
		UART_printf("\tParameter#: ");    /* In channel number */
		err = read_signed_num(&param);
		if (err != NO_ERR)
			break;
		
		UART_printf("\tArgument 1: "); /* Source node */
		err = read_signed_num(&signed_val);
		if (err != NO_ERR)
			break;
		
		msg.data[0] = (signed_val>>24) & 0xFF;
		msg.data[1] = (signed_val>>16) & 0xFF;
		msg.data[2] = (signed_val>>8) & 0xFF;
		msg.data[3] = (signed_val>>0) & 0xFF;
		
		UART_printf("\tArgument 2: ");
		err = read_signed_num(&signed_val);
		if (err != NO_ERR)
			break;
		
		msg.data[4] = (signed_val>>24) & 0xFF;
		msg.data[5] = (signed_val>>16) & 0xFF;
		msg.data[6] = (signed_val>>8) & 0xFF;
		msg.data[7] = (signed_val>>0) & 0xFF;
		
		UART_printf("\r\n");
		
		msg.id = scandal_mk_user_config_id(0x00, 
										   (dest&0xFF), 
										   param);
		
		msg.length = 8;
		can_send_msg(&msg, 0);      
		break; 

    case 'r':
		{
			int going = 1; 
			uint8_t raw_pkt[14]; 
			uint8_t rcvd_so_far = 0; 
			uint8_t receiving = 0; 
			uint8_t c; 
			uint8_t escaped = 0; 
			uint8_t chksum = 0;
			uint8_t in_chksum = 0; 
			
			while(going){
				
				while(UART_is_received()){
					c = UART_ReceiveByte();
					
					if(!escaped){
						if(c == 'q'){
							going = 0; 
							continue;
						}
						
						if(c == 'r') /* Ignore 'r', since this is how we
										get into this mode */ 
							continue; 
						
						if(c == '\\'){ /* Go into escaped mode */ 
							escaped = 1; 
							continue;
						}
						
						if(c == 'C'){ /* Start of a packet */
							rcvd_so_far = 0; 
							receiving = 1; 
							in_chksum = 0; 
							continue;
						}
					}
					
					if(receiving){ /* If have detected the start 
									  of a packet */
						raw_pkt[rcvd_so_far++] = c;
						
						if(rcvd_so_far < 14)
							in_chksum += c;
						
						/* Check to see if we've got everything */ 
						if(rcvd_so_far >= 14){
							/* Check to see that the last sent byte
							   is the correct checksum */
							if((in_chksum & 0xFF) == raw_pkt[13]){
								/* Send out the message */ 
								msg.id = 
									(uint32_t)raw_pkt[0] << 24 |
									(uint32_t)raw_pkt[1] << 16 | 
									(uint32_t)raw_pkt[2] << 8 | 
									(uint32_t)raw_pkt[3] << 0; 
								
								for(i=0; i<8; i++)
									msg.data[i] = 
										raw_pkt[i + 4];
								
								msg.length = raw_pkt[12]; 
								
								can_send_msg(&msg, 0); 
							}
						}
					}
					escaped = 0; 
				}
				
				can_poll(); 
				err = can_get_msg(&msg);
				
				if(err == NO_ERR){
					txnum = 0; 
					chksum = 0; 
					
					//	    toggle_yellow_led(); 
					
					encode_raw_in_queue('C', 1); 
					
#define ENC_BYTE(x) {encode_raw_in_queue(x, 0); chksum += (x);}
					
					ENC_BYTE((msg.id >> 24) & 0xFF);
					ENC_BYTE((msg.id >> 16) & 0xFF);
					ENC_BYTE((msg.id >> 8) & 0xFF);
					ENC_BYTE((msg.id >> 0) & 0xFF);
					
					for(i=0; i<8; i++)
						ENC_BYTE(msg.data[i]); 
					
					ENC_BYTE(msg.length);
					/* Add the checksum */ 
					ENC_BYTE(chksum); 
					
					/* Send it all out using the native MSP430 string 
					   sending gumph */
					UART_flush_tx(); 
					UART_tx(txbuf, txnum); 
				}else{
					
				}
			}
		}
		break;
		
	case 't':
		/* Send a Time Synchronisation packet... */
		//		UART_printf("Priority (0-7):");
		//		err = read_signed_num(&value);
		//		if (err != NO_ERR)
		//            break;
		
		msg.id = scandal_mk_timesync_id(CRITICAL_PRIORITY);
		
		UART_printf("New time (s):");
		err = read_signed_num(&value);
		if (err != NO_ERR)
		  break;
		
		{
		  uint64_t newtime; 
		  
		  if(value < 0)
		    value = -value;
		  
		  newtime = (uint64_t)value * 1000;
		  
		  msg.data[0] = (newtime >> 56) & 0xFF;
		  msg.data[1] = (newtime >> 48) & 0xFF;
		  msg.data[2] = (newtime >> 40) & 0xFF;
		  msg.data[3] = (newtime >> 32) & 0xFF;
		  msg.data[4] = (newtime >> 24) & 0xFF;
		  msg.data[5] = (newtime >> 16) & 0xFF;
		  msg.data[6] = (newtime >> 8) & 0xFF;
		  msg.data[7] = (newtime >> 0) & 0xFF;
		  msg.length = 8;
		  
		  can_send_msg(&msg, 0);
		}
		UART_printf("\r\n");
		
		break;

	 case 'w': // wavesculptor mode...
		{
			int going = 1;
			can_msg msg;
			uint8_t c;
			sc_time_t last_ws_command_transmission = sc_get_timer();
			float velocity = 1.0; // velocity in metres per second
			float bus_current = 1.0; // perentage of bus current max
			float motor_current = 1.0;

			while(going) {

				// do can stuff
				can_poll();

				// help to empty the rx buffers
				can_get_msg(&msg);

				while(UART_is_received()) {
					c = UART_ReceiveByte();

//					toggle_red_led(); 

					if(c == 'q') {
						going = 0; 
					}

					if (c == 'u') {
						velocity = velocity + 1.0;
						UART_printf("Increasing velocity by 1.0m/s\n\r");
					}

					if (c == 'd') {
						velocity = velocity - 1.0;
						UART_printf("Decreasing velocity by 1.0m/s\n\r");
					}


				} //UART_is_received

				if (sc_get_timer() > last_ws_command_transmission + 200) {

					UART_printf("sending drive command at time: %d", sc_get_timer());

					UART_printf(", speed: %d", (int)velocity);

					UART_printf(", motor_current: %d", (int)(motor_current*100));

					UART_printf(" and bus_current: %d\r\n", (int)(bus_current*100));

					scandal_send_ws_drive_command(DC_DRIVE, velocity, motor_current);
					scandal_send_ws_drive_command(DC_POWER, 0.0, bus_current);
					scandal_send_ws_id(DC_BASE, "TRIb", 4);

					last_ws_command_transmission = sc_get_timer();

				}

			} // while going
		} // case block

		UART_printf("\n\r");

		break;
    }
}



/* Main function */
int main(void) {
  sc_time_t     my_timer;  

  P5OUT = CAN_CS;

  dint();

  WDTCTL = WDTCTL_INIT;               //Init watchdog timer
  
  init_ports();
  init_clock();
  sc_init_timer();
  UART_Init(115200); 

  //  scandal_init();

  init_can();
  can_register_id(0x00, 0x00, 0x00, CAN_EXT_MSG);
  can_register_id(0x00, 0x00, 0x00, CAN_STD_MSG);

  UART_baud_rate(115200, CLOCK_SPEED); 

  eint();

  UART_printf("\n\rCAN to USB\r\n");
  UART_printf("David Snowdon, 2009\r\n");
  UART_printf("University of New South Wales\r\n");
  UART_printf("Press \"h\" for help\r\n");

  my_timer = sc_get_timer();

  UART_printf(">");

  while(1){
    char mode; 

    handle_scandal(); 

    if(UART_is_received()){
      mode = UART_ReceiveByte();
      handle_command(mode); 
      toggle_red_led();
      UART_printf(">");
    }

	UART_printf("\r\n");

	if(sc_get_timer() >= my_timer + 1000) {
		toggle_yellow_led();

		/* Update the timer */
		my_timer = sc_get_timer();
	}
  }
}

