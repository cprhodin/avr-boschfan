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

#include "pinmap.h"


void TM1638_spi_isr(void);

ISR(SPI_STC_vect)
{
    TM1638_spi_isr();
}


uint8_t spi_capture(void)
{
    uint8_t rc = 1;

    /* capture the SPI interface */
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        if (0 == (GPIOR0 & SPI_EV_BUSY))
        {
            GPIOR0 |= SPI_EV_BUSY;

            rc = 0;
        }
    }

    return rc;
}

void spi_release(void)
{
    /* disable SPI interrupt */
    SPCR &= ~_BV(SPIE);

    /* disable SPI interface */
    SPCR &= ~_BV(SPE);

    GPIOR0 &= ~SPI_EV_BUSY;
}


void spi_init(void)
{
    /*
     * initialize SPI interface pins
     */
    pinmap_set(PINMAP_MISO | MCP2515_INT | PINMAP_SCK | PINMAP_MOSI | TM1638_STB | MCP2515_CS);
    pinmap_dir(PINMAP_MISO | MCP2515_INT, PINMAP_SCK | PINMAP_MOSI | TM1638_STB | MCP2515_CS);

    /*
     * initialize busy bit
     */
    GPIOR0 &= ~SPI_EV_BUSY;
}

