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
#ifndef _MCP2515_H_
#define _MCP2515_H_

#include "spi.h"

/*
 * LSb first, Master, Data changes on falling edge and latches
 * on rising edge, CPU clock/2
 */
#define MCP2515_SPCR ( (SPI_MSTR_LSB | SPI_MODE3 | SPI_DIV2)       & 0xFF)
#define MCP2515_SPSR (((SPI_MSTR_LSB | SPI_MODE3 | SPI_DIV2) >> 8) & 0xFF)


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

extern void MCP2515_init(void);

#endif /* !_MCP2515_H_ */

