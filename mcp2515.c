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


/* MCP2515 commands                     */
#define MCP2515_CMD_RESET            0xC0
#define MCP2515_CMD_READ             0x03
#define MCP2515_CMD_READ_RX_BUFFER   0x90
#define  MCP2515_RX_BUFFER_RXB0SIDH  0x00
#define  MCP2515_RX_BUFFER_RXB0D0    0x02
#define  MCP2515_RX_BUFFER_RXB1SIDH  0x04
#define  MCP2515_RX_BUFFER_RXB1D0    0x06
#define MCP2515_CMD_WRITE            0x02
#define MCP2515_CMD_LOAD_TX_BUFFER   0x40
#define  MCP2515_TX_BUFFER_TXB0SIDH  0x00
#define  MCP2515_TX_BUFFER_TXB0D0    0x01
#define  MCP2515_TX_BUFFER_TXB1SIDH  0x02
#define  MCP2515_TX_BUFFER_TXB1D0    0x03
#define  MCP2515_TX_BUFFER_TXB2SIDH  0x04
#define  MCP2515_TX_BUFFER_TXB2D0    0x05
#define MCP2515_CMD_RTS              0x80
#define  MCP2515_TXB0                0x01
#define  MCP2515_TXB1                0x02
#define  MCP2515_TXB2                0x04
#define MCP2515_CMD_READ_STATUS      0xA0
#define MCP2515_CMD_RX_STATUS        0xB0
#define MCP2515_CMD_BIT_MODIFY       0x05


/* command identifier                   */
#define MCP2515_IDLE                   (0)
#define MCP2515_RESET               _BV(0)
#define MCP2515_READ                _BV(1)
#define MCP2515_READ_RX_BUFFER      _BV(2)
#define MCP2515_WRITE               _BV(3)
#define MCP2515_LOAD_TX_BUFFER      _BV(4)
#define MCP2515_RTS                 _BV(5)
#define MCP2515_READ_STATUS         _BV(6)
#define MCP2515_RX_STATUS           _BV(7)
#define MCP2515_BIT_MODIFY          _BV(8)


/*
 * command state variables
 */
static uint8_t pending_command = MCP2515_IDLE;
static uint8_t active_command = MCP2515_IDLE;


static void MCP2515_command_dispatch(void)
{
    if (!(GPIOR0 & SPI_EV_BUSY))
    {
        /* update pending and active commands */
        active_command = pending_command & -pending_command;
        pending_command &= ~active_command;
        state = 0;

        if (MCP2515_IDLE != active_command)
        {
            GPIOR0 |= SPI_EV_BUSY;

            _delay_us(MCP2515_DELAY_US);
            MCP2515_STB_LOW();
            _delay_us(MCP2515_DELAY_US);

            switch (active_command)
            {
            case MCP2515_WRITE_CONFIG:
                /* write the first byte */
                SPDR = _config;
                break;

            case MCP2515_READ_KEYPAD:
                /* write the first byte */
                SPDR = MCP2515_CMD_DATA | MCP2515_DATA_READ | MCP2515_DATA_INCR;
                break;

            case MCP2515_WRITE_SEGMENTS:
                /* write the first byte */
                SPDR = MCP2515_CMD_DATA | MCP2515_DATA_WRITE | MCP2515_DATA_INCR;
                break;
            }

            /* enable SPI interrupt */
            SPCR |= _BV(SPIE);
        }
    }
}


static void MCP2515_write_config(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        pending_command |= MCP2515_WRITE_CONFIG;
        MCP2515_command_dispatch();
    }
}

void MCP2515_read_keypad(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        pending_command |= MCP2515_READ_KEYPAD;
        MCP2515_command_dispatch();
    }
}

void MCP2515_write_segments(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        pending_command |= MCP2515_WRITE_SEGMENTS;
        MCP2515_command_dispatch();
    }
}


