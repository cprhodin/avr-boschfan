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
#include "mcp2515.h"


/*
 * command state variables
 */


static uint8_t MCP2515_capture_spi(void)
{
    uint8_t rc = 1;

    /* capture the SPI interface */
    rc = spi_capture();

    if (0 == rc)
    {
        /*
         * LSb first, Master, Data changes on falling edge and latches
         * on rising edge, CPU clock/2
         */
        SPCR = MCP2515_SPCR;
        SPSR = MCP2515_SPSR;

        /* select the MCP2515 */
        MCP2515_CS_LOW();
    }

    return rc;
}

static inline void MCP2515_release_spi(void)
{
    /* deselect the MCP2515 */
    MCP2515_CS_HIGH();

    /* disable SPI interface */
    spi_release();
}

static void MCP2515_send_byte(uint8_t byte)
{
    SPDR = byte;

    while (!(SPSR & _BV(SPIE)));
}


uint8_t MCP2515_reset(void)
{
    uint8_t rc;

    rc = MCP2515_capture_spi();

    if (0 == rc)
    {
        MCP2515_send_byte(MCP2515_CMD_RESET);

        /* release the SPI interface */
        MCP2515_release_spi();
    }

    return rc;
}

uint8_t MCP2515_rts(uint8_t const tx_buffer)
{
    uint8_t rc;

    rc = MCP2515_capture_spi();

    if (0 == rc)
    {
        MCP2515_send_byte(MCP2515_CMD_RTS | (tx_buffer & 0x07));

        /* release the SPI interface */
        MCP2515_release_spi();
    }

    return rc;
}

uint8_t MCP2515_read_status(uint8_t * data)
{
    uint8_t rc;

    rc = MCP2515_capture_spi();

    if (0 == rc)
    {
        MCP2515_send_byte(MCP2515_CMD_READ_STATUS);

        MCP2515_send_byte(0xFF);
        *data = SPDR;

        /* release the SPI interface */
        MCP2515_release_spi();
    }

    return rc;
}

uint8_t MCP2515_rx_status(uint8_t * data)
{
    uint8_t rc;

    rc = MCP2515_capture_spi();

    if (0 == rc)
    {
        MCP2515_send_byte(MCP2515_CMD_RX_STATUS);

        MCP2515_send_byte(0xFF);
        *data = SPDR;

        /* release the SPI interface */
        MCP2515_release_spi();
    }

    return rc;
}


uint8_t MCP2515_bit_modify(uint8_t const addr, uint8_t const mask, uint8_t const data)
{
    uint8_t rc;

    rc = MCP2515_capture_spi();

    if (0 == rc)
    {
        MCP2515_send_byte(MCP2515_CMD_BIT_MODIFY);
        MCP2515_send_byte(addr);
        MCP2515_send_byte(mask);
        MCP2515_send_byte(data);

        /* release the SPI interface */
        MCP2515_release_spi();
    }

    return rc;
}

uint8_t MCP2515_read(uint8_t const addr, uint8_t * data, uint8_t size)
{
    uint8_t rc;

    rc = MCP2515_capture_spi();

    if (0 == rc)
    {
        MCP2515_send_byte(MCP2515_CMD_READ);
        MCP2515_send_byte(addr);

        while (0 != size--)
        {
            MCP2515_send_byte(0xFF);
            *data++ = SPDR;
        }

        /* release the SPI interface */
        MCP2515_release_spi();
    }

    return rc;
}

uint8_t MCP2515_write(uint8_t const addr, uint8_t const * data, uint8_t size)
{
    uint8_t rc;

    rc = MCP2515_capture_spi();

    if (0 == rc)
    {
        MCP2515_send_byte(MCP2515_CMD_WRITE);
        MCP2515_send_byte(addr);

        while (0 != size--)
        {
            MCP2515_send_byte(*data++);
        }

        /* release the SPI interface */
        MCP2515_release_spi();
    }

    return rc;
}

uint8_t MCP2515_read_rx_buffer(uint8_t const rx_addr, uint8_t * data, uint8_t size)
{
    uint8_t rc;

    rc = MCP2515_capture_spi();

    if (0 == rc)
    {
        MCP2515_send_byte(MCP2515_CMD_READ_RX_BUFFER | (rx_addr & 0x06));

        while (0 != size--)
        {
            MCP2515_send_byte(0xFF);
            *data++ = SPDR;
        }

        /* release the SPI interface */
        MCP2515_release_spi();
    }

    return rc;
}

uint8_t MCP2515_load_tx_buffer(uint8_t const tx_addr, uint8_t const * data, uint8_t size)
{
    uint8_t rc;

    rc = MCP2515_capture_spi();

    if (0 == rc)
    {
        MCP2515_send_byte(MCP2515_CMD_LOAD_TX_BUFFER | (tx_addr & 0x07));

        while (0 != size--)
        {
            MCP2515_send_byte(*data++);
        }

        /* release the SPI interface */
        MCP2515_release_spi();
    }

    return rc;
}


void MCP2515_init(void)
{
    uint8_t rc;

    /* reset MCP2515 */
    do
    {
        rc = MCP2515_reset();
    }
    while (0 != rc);
}

