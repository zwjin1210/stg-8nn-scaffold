/*
Copyright (C) 2019 Adolfo E. García

This file is part of STG-8nn-Scaffold.

STG-8nn-Scaffold is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

STG-8nn-Scaffold is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with STG-8nn-Scaffold.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "uavcan/protocol/GetNodeInfo.h"
#include "bsp.h"

/* TODO: replace with actual implementation */
void BSP_readUniqueID(uint8_t* outUid)
{
    for (uint8_t i = 0; i < UAVCAN_PROTOCOL_HARDWAREVERSION_UNIQUE_ID_LENGTH; ++i) {
        outUid[i] = i;
    }
}