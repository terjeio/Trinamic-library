/*
 * common.h - shared code for Trinamic drivers
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

#pragma once

#include <stdint.h>
#include <stdbool.h>

#define TMC_N_MOTORS_MAX    6  // max number of Trinamic drivers
#define TMC_THRESHOLD_MIN   0
#define TMC_THRESHOLD_MAX   ((1<<20) - 1)

typedef union {
    uint32_t value;
    struct {
        uint32_t
        cs :5,
        seup: 2,
        sedn: 2,
        semax: 4,
        semin: 4,
        seimin :1;
    };
} trinamic_coolconf_t;

typedef union {
    uint32_t value;
    struct {
        uint32_t
        toff            :4,
        hstrt           :3,
        hend            :4,
        hdec            :2,
        rndtf           :1,
        chm             :1,
        tbl             :2,
        tfd             :4,
        intpol          :1;
    };
} trinamic_chopconf_t;

typedef uint32_t trinamic_drvconf_t;

typedef struct {
    trinamic_drvconf_t drvconf;
    trinamic_coolconf_t coolconf;
    trinamic_chopconf_t chopconf;
} trinamic_cfg_t;

typedef struct {
     float vsense[2]; // mV
     trinamic_cfg_t cap;
     trinamic_cfg_t dflt;
} trinamic_cfg_params_t;

typedef enum {
    TMC2209 = 0,
    TMC2130,
    TMC5160,
    TMC2660,
    TMC2240,
    TMCNULL
} trinamic_driver_t;

typedef enum {
   TMCMode_StealthChop = 0,
   TMCMode_CoolStep,
   TMCMode_StallGuard,
} trinamic_mode_t;

typedef enum {
   TMCCurrent_Min = 0,
   TMCCurrent_Max,
   TMCCurrent_Actual,
   TMCCurrent_Hold,
//   TMCCurrent_Momentary,
} trinamic_current_t;

#pragma pack(push, 1)

typedef struct {
    uint8_t id;         // motor id
    uint8_t axis;       // axis index
    uint8_t address;    // UART address
    uint8_t seq;        // optional motor sequence number (for chained SPI drivers)
    void *cs_pin;       // optional CS pin data for the stepper driver
} trinamic_motor_t;

typedef struct {
    uint32_t f_clk;
    uint16_t microsteps;
    uint16_t r_sense;           // mOhm
    uint16_t current;           // mA
    uint8_t hold_current_pct;   // percent
    trinamic_mode_t mode;
    trinamic_motor_t motor;
    const trinamic_cfg_params_t *cfg_params;
} trinamic_config_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t
        idx   :7,
        write :1;
    };
} TMC_addr_t;

typedef union {
    uint32_t value;
    uint8_t data[4];
} TMC_payload_t;

typedef struct {
    TMC_addr_t addr;
    TMC_payload_t payload;
} TMC_spi_datagram_t;

typedef union {
    uint32_t value;
    uint8_t data[3];
    struct {
        uint32_t
        payload :20;
    };
} TMC_spi20_datagram_t;

typedef union {
    uint8_t data[8];
    struct {
        uint8_t sync;
        uint8_t slave;
        TMC_addr_t addr;
        TMC_payload_t payload;
        uint8_t crc;
    } msg;
} TMC_uart_write_datagram_t;

typedef union {
    uint8_t data[4];
    struct {
        uint8_t sync;
        uint8_t slave;
        TMC_addr_t addr;
        uint8_t crc;
    } msg;
} TMC_uart_read_datagram_t;

// Custom registers used by I2C <> SPI bridge
typedef enum {
    TMC_I2CReg_MON_STATE = 0x7D,
    TMC_I2CReg_ENABLE = 0x7E
} TMC_i2c_registers_t;

#pragma pack(pop)

typedef uint8_t TMC_spi_status_t;

bool tmc_microsteps_validate (uint16_t microsteps);
uint8_t tmc_microsteps_to_mres (uint16_t microsteps);
uint32_t tmc_calc_tstep (trinamic_config_t *config, float mm_sec, float steps_mm);
float tmc_calc_tstep_inv (trinamic_config_t *config, uint32_t tstep, float steps_mm);
void tmc_crc8 (uint8_t *datagram, uint8_t datagramLength);
void tmc_motors_set (uint8_t motors);
uint8_t tmc_motors_get (void);

static inline void tmc_byteswap (uint8_t data[4])
{
    uint8_t tmp;

    tmp = data[0];
    data[0] = data[3];
    data[3] = tmp;
    tmp = data[1];
    data[1] = data[2];
    data[2] = tmp;
}

extern TMC_spi_status_t tmc_spi_write (trinamic_motor_t driver, TMC_spi_datagram_t *datagram);
extern TMC_spi_status_t tmc_spi_read (trinamic_motor_t driver, TMC_spi_datagram_t *datagram);

extern TMC_spi20_datagram_t tmc_spi20_write (trinamic_motor_t driver, TMC_spi20_datagram_t *datagram);
extern TMC_spi20_datagram_t tmc_spi20_read (trinamic_motor_t driver, TMC_spi20_datagram_t *datagram);

extern void tmc_uart_write (trinamic_motor_t driver, TMC_uart_write_datagram_t *datagram);
extern TMC_uart_write_datagram_t *tmc_uart_read (trinamic_motor_t driver, TMC_uart_read_datagram_t *datagram);
