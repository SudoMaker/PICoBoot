/*
    This file is part of PotatoPi PICo24 SDK.

    Copyright (C) 2021 ReimuNotMoe <reimu@sudomaker.com>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#pragma once

// What's your problem, Microchip?

#if defined(__CLION_IDE_) || defined(__INTELLISENSE__)
#define __eds__
#define __prog__

#undef uint8_t
#undef int8_t
#undef uint16_t
#undef int16_t
#undef uint32_t
#undef int32_t
#undef uint64_t
#undef int64_t
#endif