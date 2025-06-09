/*
 * common.c - shared code for Trinamic drivers
 *
 * v0.0.9 / 2025-06-08
 */

/*

Copyright (c) 2021-2025, Terje Io
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may
be used to endorse or promote products derived from this software without
specific prior written permission..

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "common.h"

static uint8_t tmc_motors = 0;

// 1 - 256 in steps of 2^value is valid
bool tmc_microsteps_validate (uint16_t microsteps)
{
    uint_fast8_t i = 8, count = 0;

    if(microsteps <= 256) do {
        if(microsteps & 0x01)
            count++;
        microsteps >>= 1;
    } while(i--);

    return count == 1;
}

uint8_t tmc_microsteps_to_mres (uint16_t microsteps)
{
    uint8_t value = 0;

    microsteps = microsteps == 0 ? 1 : microsteps;

    while((microsteps & 0x01) == 0) {
        value++;
        microsteps >>= 1;
    }

    return 8 - (value > 8 ? 8 : value);
}

uint32_t tmc_calc_tstep (trinamic_config_t *config, float mm_sec, float steps_mm)
{
    uint32_t den = (uint32_t)(256.0f * mm_sec * steps_mm);

    return den ? (config->microsteps * config->f_clk) / den : 0;
}

float tmc_calc_tstep_inv (trinamic_config_t *config, uint32_t tstep, float steps_mm)
{
    return tstep == 0 ? 0.0f : (float)(config->f_clk * config->microsteps) / (256.0f * (float)tstep * steps_mm);
}

void tmc_motors_set (uint8_t motors)
{
    tmc_motors = motors;
}

uint8_t tmc_motors_get (void)
{
    return tmc_motors;
}

void tmc_crc8 (uint8_t *datagram, uint8_t datagramLength)
{
    int i,j;
    uint8_t *crc = datagram + (datagramLength - 1); // CRC located in last byte of message
    uint8_t currentByte;
    *crc = 0;
    for (i = 0; i < (datagramLength - 1); i++) {    // Execute for all bytes of a message
        currentByte = datagram[i];                  // Retrieve a byte to be sent from Array
        for (j = 0; j < 8; j++) {
            if ((*crc >> 7) ^ (currentByte & 0x01)) // update CRC based result of XOR operation
                *crc = (*crc << 1) ^ 0x07;
            else
                *crc = (*crc << 1);
            currentByte = currentByte >> 1;
        } // for CRC bit
    }
}
