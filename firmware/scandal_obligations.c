/* --------------------------------------------------------------------------                                 
    USBCAN Main
    File name: scandal_obligations.c
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

#include "scandal_obligations.h"
#include "scandal_error.h"
#include "scandal_devices.h"
#include "scandal_led.h"

/* Reset the node in a safe manner
	- will be called from handle_scandal */
void scandal_reset_node(void){
  /* Reset the node here */
  /* Write an invalid password to the WDT */
  WDTCTL = ~WDTPW;
}

void scandal_user_do_first_run(void){
  return;
}

u08 scandal_user_do_config(u08 param, s32 value, s32 value2){
	return NO_ERR;
}

u08 scandal_user_handle_message(can_msg* msg){
	return NO_ERR;
}

u08 scandal_user_handle_command(u08 command, u08* data){
  return NO_ERR; 
}

