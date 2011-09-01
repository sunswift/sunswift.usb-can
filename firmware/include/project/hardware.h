/* --------------------------------------------------------------------------                                 
    USBCAN Main
    File name: hardware.h
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

/* Hardware definitions */
#define BIT(x) (1<<x)

#define CLOCK_SPEED 7372800

/* Port 1 */

/* Port 2 */
#define CAN_INT         BIT(7)

/* Port 3 */
#define TX              BIT(4) 
#define RX              BIT(5)

/* Port 4 */

/* Port 5 */
#define CAN_CS          BIT(0) /* Note: re-defined in scandal_devices.h */
#define SIMO1           BIT(1)
#define SOMI1           BIT(2)
#define UCLK1           BIT(3)
#define YELLOWLED       BIT(6)
#define REDLED          BIT(7)

/* Port 6 / ADC */
#define MEAS_12V_PIN    (BIT(7))

/* ADC channel definitions */
#define MEAS_12V        7

#define MEAS_TEMP       8
#define MEAS_3V3        9

