/* --------------------------------------------------------------------------                                 
    USBCAN Main
    File name: spi_driver.c
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
#include <io.h>

#include "spi_devices.h" 
#include "scandal_spi.h"

#include "scandal_led.h"

/* Initialize the SPI driver */         

/* Local variables */ 
u08 spi_selected_device; 

/*! Initialise the SPI driver to the default values */ 
u08 init_spi(){
  ME2 |= USPIE1;
  
  U1CTL  = SYNC+MM+CHAR;
  U1TCTL = STC | SSEL1 | SSEL0 | CKPL ;
  UBR01 = 0x04;
  UBR11 = 0x00;
  UMCTL1 = 0x00;

  spi_selected_device = SPI_DEVICE_NONE; 
  spi_deselect_all(); 
  return(0); 
}  

/*! Select a particular device */ 
u08 spi_select_device(u08	device){  
  if(spi_selected_device != SPI_DEVICE_NONE) 
    return(1); 
   
  spi_deselect_all(); 
   
  switch(device){ 
  case MCP2510:  
    ENABLE_MCP2510();
    break; 
  } 
   
  spi_selected_device = device; 	 
  return(0); 
} 

/*! Deselect all devices */ 
void spi_deselect_all(){ 
  DISABLE_MCP2510();
  spi_selected_device = SPI_DEVICE_NONE; 
} 

/*! Perform an SPI read/write */ 
u08 spi_transfer(u08 out_data){ 
  u08	value; 

  IFG2 &= ~URXIFG1;

  while((IFG2 & UTXIFG1) == 0)
      ;
  
  TXBUF1 = out_data;

  while((IFG2 & URXIFG1) == 0)
      ;

  value = RXBUF1;

  return(value); 
} 
