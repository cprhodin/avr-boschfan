/*
 * Copyright 2013-2023 Chris Rhodin <chris@notav8.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef _TM1638_H_
#define _TM1638_H_

#include <stdint.h>

#include "spi.h"

/*
 * LSb first, Master, Data changes on falling edge and latches
 * on rising edge, CPU clock/16
 */
#define TM1638_SPCR ( (SPI_MSTR_LSB | SPI_MODE3 | SPI_DIV16)       & 0xFF)
#define TM1638_SPSR (((SPI_MSTR_LSB | SPI_MODE3 | SPI_DIV16) >> 8) & 0xFF)

/*
 * Main Settings
 */
#define TM1638_MAX_BRIGHTNESS   7
#define TM1638_MAX_DIGIT        9
#define TM1638_MAX_VALUE        15

/*
 * keys buffer for keyboard
 */
struct tm1638_keypad
{
    uint32_t filter;    // + 0
    uint32_t last;      // + 4
    uint32_t up;        // + 8
    uint32_t down;      // + 12
};

/*
 * Initialize TM1638 LED controller
 */
extern void TM1638_init(uint8_t const keys_update_ms);

/*
 * Configure display
 */
extern void TM1638_led_enable(uint8_t const enable);
extern void TM1638_brightness(uint8_t const brightness);

/*
 * Write segment buffer
 */
extern void TM1638_write_segments(void);

/*
 * Read keys
 */
extern void TM1638_read_keypad(void);
extern int TM1638_get_key(void);


/*
 * Display digit ('0'..'F')
 */
extern void TM1638_write_digit(uint8_t const position, int8_t const value);

#endif /* !_TM1638_H_ */

