/* --------------------------------------------------------------------------                                 
    USBCAN Main
    File name: scandal_timer.c
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
#include <signal.h>

#include "scandal_timer.h"

static volatile u32 ms;

/* Interrupt handler associated with internal RTC */
/* Timer A overflow interrupt */
interrupt (TIMERA0_VECTOR) timera_int(void) {
  ms += 1000;
}

void sc_init_timer(void){
  /* Set ms to zero */
  ms = 0;
  
  /* Use TimerA to create periodic interrupts */
  
  /* Clear counter, input divider /1, ACLK */
  TACTL = /*TAIE |*/ TACLR | ID_DIV1 | TASSEL_ACLK;

  /* Enable Capture/Compare interrupt */
  TACCTL0 = CCIE;
  TACCR0 = 32767; /* Count 1 sec at ACLK=32768Hz */
  
  /* Start timer in up to CCR0 mode */
  TACTL |= MC_UPTO_CCR0;
}

void sc_set_timer(sc_time_t time){
  TACCTL0 &= ~CCIE;
  TACCR0 = ((time % 1000) << 15) / 1000;
  ms = time / 1000;
  TACCTL0 |= CCIE;
}

sc_time_t sc_get_timer(void){
  sc_time_t 	time;
  u32           tar_copy;
  
  /* Work out what the time in ms is */

  /* Turn off the timer interrupt */ 
  TACCTL0 &= ~CCIE; 

  /* Short delay so that we're sure the interrupt is off */ 
  {
    volatile int i; 
    for(i=0; i<15; i++)
      ;
  }

  /* Copy the relevant numbers */ 
  time = ms;
  tar_copy = TAR; 

  /* Turn the timer interrupt back on */ 
  TACCTL0 |= CCIE; 

  time += ((tar_copy * 1000) >> 15);

  return time;
}
