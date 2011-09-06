/* --------------------------------------------------------------------------                                 
    USBCAN Main
    File name: spi_devices.c
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
  
#ifndef __SPIDEVICES__ 
#define __SPIDEVICES__ 

#include <io.h>

#define BIT(x) (1<<x)

#define ERR_MARKER 0xDEADBEEF

#define MCP2510			0              
#define SPI_NUM_DEVICES         1
#define SPI_DEVICE_NONE		SPI_NUM_DEVICES 

/* MCP2510 */
#define ENABLE_MCP2510()        (P5OUT &= ~BIT(0))
#define DISABLE_MCP2510()       (P5OUT |= BIT(0))

#endif
