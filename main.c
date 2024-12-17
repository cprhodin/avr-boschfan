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
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include <util/atomic.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "timer.h"
#include "tick.h"
#include "tm1638.h"
#include "bibase.h"

#define BUILD_DATE __DATE__ ", " __TIME__

#define PWM_FREQ    100U  // in Hertz
#define MIN_FAN_RPM 840U  // in RPM
#define MAX_FAN_RPM 2200U // in RPM

#define PWM_COUNTS  20000U  // 100 Hertz at 20000 timer clock
#define MIN_COUNTS  1900UL  // 10%
#define MAX_COUNTS  14900UL // 75%

/* 840 slow, 2200 fast */
static uint16_t rpm = MAX_FAN_RPM;

static uint8_t brightness = TM1638_MAX_BRIGHTNESS / 2;


static struct tm1638_keypad keys = { 0 };


void set_rpm(uint16_t rpm)
{
    uint16_t pulse_counts;

    if (0U == rpm)
    {
        pulse_counts = 0U;
    }
    else
    {
        /* limit RPM to supported range */
        if      (rpm < MIN_FAN_RPM)
        {
            rpm = MIN_FAN_RPM;
        }
        else if (rpm > MAX_FAN_RPM)
        {
            rpm = MAX_FAN_RPM;
        }

	    /* interpolate pulse width */
	    pulse_counts = (uint16_t) (MIN_COUNTS + (((uint32_t) (rpm - MIN_FAN_RPM)
	                               * (uint32_t) (MAX_COUNTS - MIN_COUNTS))
	                               / (uint32_t) (MAX_FAN_RPM - MIN_FAN_RPM)));
    }

    OCR1A = (PWM_COUNTS - 1) - pulse_counts;
}


static void update_rpm(void)
{
    /* process button pushes */
    int const k = TM1638_get_key();
    int delta_rpm = 0;

    switch (k)
    {
    case 0x02:
        /* ON */
        TM1638_enable(1);
        break;

    case 0x22:
        /* OFF */
        TM1638_enable(0);
        break;

    case 0x16:
        /* brighter */
        if (brightness < TM1638_MAX_BRIGHTNESS)
        {
            brightness++;
        }

        TM1638_brightness(brightness);
        break;

    case 0x36:
        /* dimmer */
        if (brightness > 0)
        {
            brightness--;
        }

        TM1638_brightness(brightness);
        break;

    case 0x01:
        delta_rpm = 1000;
        break;

    case 0x05:
        delta_rpm = 100;
        break;

    case 0x11:
        delta_rpm = 10;
        break;

    case 0x15:
        delta_rpm = 1;
        break;

    case 0x21:
        delta_rpm = -1000;
        break;

    case 0x25:
        delta_rpm = -100;
        break;

    case 0x31:
        delta_rpm = -10;
        break;

    case 0x35:
        delta_rpm = -1;
        break;
    }

    if      (0 > delta_rpm)
    {
        if      (rpm <= MIN_FAN_RPM)
        {
            rpm = 0;
        }
        else if ((rpm - MIN_FAN_RPM) < -delta_rpm)
        {
            rpm = MIN_FAN_RPM;
        }
        else
        {
            rpm += delta_rpm;
        }

        set_rpm(rpm);
    }
    else if (0 < delta_rpm)
    {
        if (0 == rpm)
        {
            rpm = MIN_FAN_RPM;
        }
        else
        {
            rpm += delta_rpm;

            if (rpm > MAX_FAN_RPM)
            {
                rpm = MAX_FAN_RPM;
            }
        }

        set_rpm(rpm);
    }

    uint8_t dec[4] = { 0, 0, 0, 0 };

    uint8_t n_digit = bibase(0, rpm >> 8, dec, 246);
    n_digit = bibase(n_digit, rpm, dec, 246);

    TM1638_write_digit(3, (n_digit > 3) ? dec[3] : -1);
    TM1638_write_digit(2, (n_digit > 2) ? dec[2] : -1);
    TM1638_write_digit(1, (n_digit > 1) ? dec[1] : -1);
    TM1638_write_digit(0, dec[0]);
}


void fan_init(void)
{
    /* initialize fan output pin */
    pinmap_clear(FAN_OUT);
    pinmap_dir(0, FAN_OUT);

    // Timer 1, Fast PWM mode 14, WGM = 1:1:1:0, clk/8,
    TCCR1A = 0xC2;  // COM1A1 = 1, COM1A0 = 0, WGM11 = 1, WGM10 = 0
    TCCR1B = 0x1A;  // WGM13 = 1, WGM12 = 1, CS = 2
    TCCR1C = 0x00;
    ICR1 = PWM_COUNTS - 1;
    OCR1A = PWM_COUNTS - 1;
}


void main(void)
{
    /*
     * initialize
     */
    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        tbtick_init();
        tick_init();
        fan_init();
    }
    /* interrupts are enabled */

    printf("\033[2J\033[H");
    printf("avr-boschfan, Bosch radiator fan controller.\n");
    printf("Build date: " BUILD_DATE "\n\n");

    /* initialize and enable the TM1638 */
    TM1638_init(10);
    TM1638_enable(1);

    for (;;)
    {
        /* read keys and update rpm */
        update_rpm();
    }
}

