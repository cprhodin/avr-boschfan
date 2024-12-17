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
#include "project.h"

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "timer.h"
#include "pinmap.h"
#include "tm1638.h"


#define TM1638_DELAY_US          (1)

/* TM1638 commands                  */
#define TM1638_CMD_DATA         0x40
#define TM1638_CMD_ADDRESS      0xC0
#define TM1638_CMD_DISPLAY      0x80

/* TM1638 data command bitfields    */
#define TM1638_DATA_WRITE       0x00
#define TM1638_DATA_READ        0x02
#define TM1638_DATA_INCR        0x00
#define TM1638_DATA_FIXED       0x04

/* TM1638 address command bitfields */
#define TM1638_ADDRESS_MASK     0x0F

/* TM1638 display command bitfields */
#define TM1638_DISPLAY_BRIGHT   0x07
#define TM1638_DISPLAY_ON       0x08


/* command identifier */
#define TM1638_IDLE              (0)
#define TM1638_WRITE_CONFIG   _BV(0)
#define TM1638_READ_KEYS      _BV(1)
#define TM1638_WRITE_SEGMENTS _BV(2)


/*
 * segments buffer for LED display
 */
static uint8_t segments_buffer[16];

/*
 * keys buffer for keyboard
 */
static uint32_t keys_buffer = 0;

/*
 * default to display off at 1/2 maximum brightness
 */
static uint8_t _config = TM1638_CMD_DISPLAY
                       | (TM1638_MAX_BRIGHTNESS / 2);

/*
 * command state variables
 */
static uint8_t pending_command = TM1638_IDLE;
static uint8_t active_command = TM1638_IDLE;
static uint8_t state;
static uint8_t * data;


static uint8_t   spi_action;
static uint8_t   nxt_spi_action;
static uint8_t   spi_data_cnt;
static uint8_t * spi_data_ptr;


extern void TM1638_command_dispatch(void);

static void spi_command_dispatch(void)
{
    TM1638_command_dispatch();
}


extern void TM1638_spi_isr(void);

ISR(SPI_STC_vect)
{
    TM1638_spi_isr();
}

