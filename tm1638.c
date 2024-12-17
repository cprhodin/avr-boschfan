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

#include "spi.h"
#include "timer.h"
#include "pinmap.h"
#include "tm1638.h"
#include "librb.h"

/*
 * keypad ring-buffer.
 */
#ifndef TM1638_KEY_BUF_SIZE
#define TM1638_KEY_BUF_SIZE (16)
#endif
static uint8_t key_buffer[TM1638_KEY_BUF_SIZE];
static struct ring_buffer tm1638_key_rb;


/* strobe delay                     */
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
#define TM1638_READ_KEYPAD    _BV(1)
#define TM1638_WRITE_SEGMENTS _BV(2)


/*
 * segments buffer for LED display
 */
static uint8_t segments_buffer[16];

/*
 * keypad state
 */
static struct tm1638_keypad keypad = { 0 };

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


static void TM1638_command_dispatch(void)
{
    if (!(GPIOR0 & SPI_EV_BUSY))
    {
        /* update pending and active commands */
        active_command = pending_command & -pending_command;
        pending_command &= ~active_command;
        state = 0;

        if (TM1638_IDLE != active_command)
        {
            GPIOR0 |= SPI_EV_BUSY;

            _delay_us(TM1638_DELAY_US);
            TM1638_STB_LOW();
            _delay_us(TM1638_DELAY_US);

            switch (active_command)
            {
            case TM1638_WRITE_CONFIG:
                /* write the first byte */
                SPDR = _config;
                break;

            case TM1638_READ_KEYPAD:
                /* write the first byte */
                SPDR = TM1638_CMD_DATA | TM1638_DATA_READ | TM1638_DATA_INCR;
                break;

            case TM1638_WRITE_SEGMENTS:
                /* write the first byte */
                SPDR = TM1638_CMD_DATA | TM1638_DATA_WRITE | TM1638_DATA_INCR;

                /* set the command data */
                data = &segments_buffer[0];
                break;
            }

            /* enable SPI interrupt */
            SPCR |= _BV(SPIE);
        }
    }
}


static void read_keypad(struct tm1638_keypad * const keypad, uint8_t const state)
{
    /* translate state into index*/
    uint8_t const index = state - 1;

    // get pointer into the data structure
    uint8_t * const data = (uint8_t *) keypad + index;

    // get and filter latest keypad
    uint8_t const filter = *data;
    uint8_t const spdr = SPDR & 0xEE;
    *data = spdr;

    // unstable keys
    uint8_t const unstable = spdr ^ filter;

    // update filtered key
    uint8_t const old_last = *(data + 4);
    uint8_t const last = (old_last & unstable) | (filter & ~unstable);
    *(data + 4) = last;

    // changed keys
    uint8_t const changed = old_last ^ last;

    // keys changed to up
    uint8_t const up = (changed & ~last) | *(data + 8);

#ifndef XXX
    *(data + 8) = up;
#else
    uint8_t key_up = up & -up;

    if (0 != key_up)
    {
        *(data + 8) = up & ~key_up;

        key_up--;
        key_up -= ((key_up >> 1) & 0x55);
        key_up  = ((key_up >> 2) & 0x33) + (key_up & 0x33);
        key_up  = (__builtin_avr_swap(key_up) + key_up) & 0x07;

        key_up += (index << 4) + 0x80;

        rb_put(&tm1638_key_rb, (uint8_t *) &key_up);
    }
#endif

    // keys changed to down
    uint8_t const down = (changed & last) | *(data + 12);

#ifndef XXX
    *(data + 12) = down;
#else
    uint8_t key_down = down & -down;

    if (0 != key_down)
    {
        *(data + 12) = down & ~key_down;

        key_down--;
        key_down -= ((key_down >> 1) & 0x55);
        key_down  = ((key_down >> 2) & 0x33) + (key_down & 0x33);
        key_down  = (__builtin_avr_swap(key_down) + key_down) & 0x07;

        key_down += index << 4;

        rb_put(&tm1638_key_rb, (uint8_t *) &key_down);
    }
#endif
}


ISR(SPI_STC_vect)
{
    switch (active_command)
    {
    case TM1638_WRITE_CONFIG:
        GPIOR0 &= ~SPI_EV_BUSY;
        break;

    case TM1638_READ_KEYPAD:
        if      (0 == state)  /* command state 0 */
        {
            pinmap_dir(PINMAP_MOSI, 0);
            SPDR = 0xFF;
        }
        else if (4 > state)   /* command state 1 thru 3 */
        {
            read_keypad(&keypad, state);
            SPDR = 0xFF;
        }
        else                  /* command state 4 */
        {
            read_keypad(&keypad, state);
            pinmap_dir(0, PINMAP_MOSI);

            GPIOR0 &= ~SPI_EV_BUSY;
        }
        break;

    case TM1638_WRITE_SEGMENTS:
        if      (0 == state)  /* command state 0 */
        {
            _delay_us(TM1638_DELAY_US);
            TM1638_STB_HIGH();
            _delay_us(TM1638_DELAY_US);
            TM1638_STB_LOW();
            _delay_us(TM1638_DELAY_US);
            SPDR = TM1638_CMD_ADDRESS | (0 & TM1638_ADDRESS_MASK);

            /* set the command data */
            data = &segments_buffer[0];
        }
        else if (16 >= state) /* command state 1 thru 16 */
        {
            SPDR = *data++;
        }
        else                  /* command state 17 */
        {
            GPIOR0 &= ~SPI_EV_BUSY;
        }
        break;

    default:
        GPIOR0 &= ~SPI_EV_BUSY;
        break;
    }

    state++;

    if (!(GPIOR0 & SPI_EV_BUSY))
    {
        _delay_us(TM1638_DELAY_US);
        TM1638_STB_HIGH();

        SPCR &= ~_BV(SPIE);

        TM1638_command_dispatch();
    }
}


static void TM1638_write_config(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        pending_command |= TM1638_WRITE_CONFIG;
        TM1638_command_dispatch();
    }
}

void TM1638_read_keypad(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        pending_command |= TM1638_READ_KEYPAD;
        TM1638_command_dispatch();
    }
}

void TM1638_write_segments(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        pending_command |= TM1638_WRITE_SEGMENTS;
        TM1638_command_dispatch();
    }
}


/*
 * TM1638_get_key, this call can be blocking or non-blocking
 */
int TM1638_get_key(void)
{
    char c;

    if (-1 == rb_get(&tm1638_key_rb, (uint8_t *) &c))
    {
        return -1;
    }

    return c;
}


void TM1638_get_keys(struct tm1638_keypad * keys)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        *keys = keypad;
        keypad.up = 0;
        keypad.down = 0;
    }
}

void TM1638_enable(uint8_t const enable)
{
    _config = (_config & ~TM1638_DISPLAY_ON)
            | (enable ? TM1638_DISPLAY_ON : 0);

    TM1638_write_config();
}

void TM1638_brightness(uint8_t const brightness)
{
    _config = (_config & ~TM1638_DISPLAY_BRIGHT)
            | (brightness & TM1638_DISPLAY_BRIGHT);

    TM1638_write_config();
}


/*
 * timer events for periodic key scanning
 */
tbtick_t keys_update_interval;

static int8_t keys_update_handler(struct timer_event * this_timer_event)
{
    /* set pending command bit */
    pending_command |= TM1638_READ_KEYPAD;

    TM1638_command_dispatch();

    /* advance this timer */
    this_timer_event->tbtick += keys_update_interval;

    /* reschedule this timer */
    return 1;
}

static struct timer_event keys_update_event = {
    .next = &keys_update_event,
    .handler = keys_update_handler,
};


void TM1638_init(uint8_t const keys_update_ms)
{
    /**/
    rb_init(&tm1638_key_rb, key_buffer, sizeof(key_buffer));

    /* initialize SPI interface */
    pinmap_set(PINMAP_MISO | PINMAP_SCK | PINMAP_MOSI | PINMAP_SS | TM1638_STB);
    pinmap_dir(PINMAP_MISO, PINMAP_SCK | PINMAP_MOSI | PINMAP_SS | TM1638_STB);

    /*
     * LSb first, Master, Data changes on falling edge and latches
     * on rising edge, CPU clock/32
     */
    SPCR = TM1638_SPCR;
    SPSR = TM1638_SPSR;

    /* initialize variables */
    GPIOR0 &= ~SPI_EV_BUSY;
    pending_command = TM1638_IDLE;
    active_command = TM1638_IDLE;

    TM1638_write_config();

    for (uint8_t i = 0; i < ARRAY_SIZE(segments_buffer); i++)
    {
        segments_buffer[i] = 0x00;
    }

    TM1638_write_segments();

    keys_update_interval = TBTICKS_FROM_MS(keys_update_ms);

    /*
     * schedule key scan
     */
    if (0 != keys_update_ms)
    {
        keys_update_event.tbtick = keys_update_interval;
        schedule_timer_event(&keys_update_event, NULL);
    }
}


/*
 * Display segments
 *
 *      --a--
 *     |     |
 *     f     b
 *     |     |
 *      --g--
 *     |     |
 *     e     c
 *     |     |
 *      --d-- * dp
 *
 *   Display Bits
 *
 *    a:  _BV(0)
 *    b:  _BV(1)
 *    c:  _BV(2)
 *    d:  _BV(3)
 *    e:  _BV(4)
 *    f:  _BV(5)
 *    g:  _BV(6)
 *    dp: _BV(7)
 *
 * Example segment configurations:
 * - for character 'H', segments=bcefg, bits=0b01110110
 * - for character '-', segments=g,     bits=0b01000000
 * - etc.
 */
static const uint8_t PROGMEM _digit_segments[] =
{
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F, // 9
    0x77, // A
    0x7C, // b
    0x39, // C
    0x5E, // d
    0x79, // E
    0x71, // F
};

void TM1638_write_digit(uint8_t const digit, int8_t const value)
{
    if (digit <= TM1638_MAX_DIGIT)
    {
        uint16_t const digit_mask = 0x0001 << digit;
        uint8_t segments;

        if ((value < 0) || (value > TM1638_MAX_VALUE))
        {
            segments = 0x00;
        }
        else
        {
            segments = pgm_read_word(&_digit_segments[value]);
        }

        for (uint8_t i = 0; i < ARRAY_SIZE(segments_buffer); i += 2)
        {
            uint16_t * segment_word = (uint16_t *) &segments_buffer[i];

            if (segments & 0x01)
            {
                *segment_word |= digit_mask;
            }
            else
            {
                *segment_word &= ~digit_mask;
            }

            segments >>= 1;
        }
    }

    /* schedule segment update */
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        pending_command |= TM1638_WRITE_SEGMENTS;
    }
}

