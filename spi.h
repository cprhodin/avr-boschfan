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
#ifndef _SPI_H_
#define _SPI_H_

#define SPI_MODE0 (0         | 0        )
#define SPI_MODE1 (0         | _BV(CPHA))
#define SPI_MODE2 (_BV(CPOL) | 0        )
#define SPI_MODE3 (_BV(CPOL) | _BV(CPHA))

#define SPI_DIV2   (_BV(SPI2X+8) | 0         | 0        )
#define SPI_DIV4   (0            | 0         | 0        )
#define SPI_DIV8   (_BV(SPI2X+8) | 0         | _BV(SPR0))
#define SPI_DIV16  (0            | 0         | _BV(SPR0))
#define SPI_DIV32  (_BV(SPI2X+8) | _BV(SPR1) | 0        )
#define SPI_DIV64  (0            | _BV(SPR1) | 0        )
#define SPI_DIV128 (0            | _BV(SPR1) | _BV(SPR0))

#define SPI_MSTR_LSB (_BV(SPE) | _BV(DORD) | _BV(MSTR))
#define SPI_MSTR_MSB (_BV(SPE) | 0         | _BV(MSTR))

#endif /* !_SPI_H_ */

