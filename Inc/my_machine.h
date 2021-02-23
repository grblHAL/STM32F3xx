/*
  my_machine.h - configuration for STM32F3xx ARM processors

  Part of grblHAL

  Copyright (c) 2021 Terje Io

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

// NOTE: Only one board may be enabled!
// If none is enabled pin mappings from generic_map.h will be used.
//#define BOARD_CNC3040
//#define BOARD_CNC_BOOSTERPACK
//#define BOARD_MY_MACHINE // Add my_machine_map.h before enabling this!

// Configuration
// Uncomment to enable.
#define USB_SERIAL_CDC       1 // Serial communication via native USB.
//#define SDCARD_ENABLE        1 // Run gcode programs from SD card, requires sdcard plugin.
//#define KEYPAD_ENABLE        1 // I2C keypad for jogging etc., requires keypad plugin.
//#define ODOMETER_ENABLE      1 // Odometer plugin.
//#define EEPROM_ENABLE        2 // I2C EEPROM support. Set to 1 for 24LC16(2K), 2 for larger sizes. Requires eeprom plugin.
//#define EEPROM_IS_FRAM       1 // Uncomment when EEPROM is enabled and chip is FRAM, this to remove write delay.

/**/
