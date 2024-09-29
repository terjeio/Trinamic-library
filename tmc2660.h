/*
 * tmc2660.h - register and message (datagram) descriptors for Trinamic TMC2660 stepper driver
 *
 * v0.0.2 / 2024-09-28
 */

/*

Copyright (c) 2023-2024, Expatria Technologies,
Copyright (c) 2024, Terje Io
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
specific prior written permission.

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

#ifndef _TRINAMIC2660_H_
#define _TRINAMIC2660_H_

#include "common.h"
//#include "tmc26x.h"

//#define TMC2660_COMPLETE // comment out for minimum set of registers

#pragma pack(push, 1)

typedef enum {
    TMC2660_Microsteps_1 = 1,
    TMC2660_Microsteps_2 = 2,
    TMC2660_Microsteps_4 = 4,
    TMC2660_Microsteps_8 = 8,
    TMC2660_Microsteps_16 = 16,
    TMC2660_Microsteps_32 = 32,
    TMC2660_Microsteps_64 = 64,
    TMC2660_Microsteps_128 = 128,
    TMC2660_Microsteps_256 = 256
} tmc2660_microsteps_t;


// default values

// General
#define TMC2660_F_CLK               15000000UL              // factory tuned to 15MHz - see datasheet for calibration procedure if required
#define TMC2660_MODE                TMCMode_CoolStep        // 0 = TMCMode_StealthChop - not supported on 2660, 1 = TMCMode_CoolStep, 3 = TMCMode_StallGuard
#define TMC2660_MICROSTEPS          TMC2660_Microsteps_8    // Default 8x microsteps -- this gets overwritten upon motor add by defaults from trinamic.h
#define TMC2660_R_SENSE             50                      // mOhm -- this gets overwritten upon motor add by defaults from trinamic.h
#define TMC2660_CURRENT             500                     // mA RMS -- this gets overwritten upon motor add by defaults from trinamic.h
#define TMC2660_HOLD_CURRENT_PCT    50                      // holding current percent -- this gets overwritten upon motor add by defaults from trinamic.h

// CHOPCONF
#define TMC2660_TOFF                1   // Off time: 1 - 15, 0 = MOSFET disable
#define TMC2660_TBL                 1   // Blanking time: 0 = 16, 1 = 24, 2 = 36, 3 = 54 clocks
#define TMC2660_CHM                 0   // Chopper mode: 0 = spreadCycle, 1 = constant off time
#define TMC2660_HSTRT               3   // Hysteresis start: 1 - 8
#define TMC2660_HEND                5   // Hysteresis end: -3 - 12
#define TMC2660_HMAX               15   // HSTRT + HEND
#define TMC2660_HDEC                0   // Hysteresis decrement: 0 = 16, 1 = 32, 2 = 48, 3 = 64 clocks
#define TMC2660_RNDTF               1   // Random TOFF time

//SGCSCONF
#define TMC2660_CURRENT_SCALE       10  // Current scale default (conservative)
#define TMC2660_SG_THRESH           22  // Stallguard threshold
#define TMC2660_SG_FILTER           1   // Enable Stallguard Filter

//DRVCONF
#define TMC2660_DRVCONF             0xA31F  //0xA33F extra debug//0xA31F   // DRVCONF Register defaults (likely don't need to change)  All protections enabled.

//DRVCTRL
#define TMC2660_DEDGE               0   // Double edge step pulses
#define TMC2660_INTPOL              1   // Step interpolation

//SMARTEN
#define TMC2660_SEMIN               7   // 0 = Coolstep disabled
#define TMC2660_SEUP                3   // 0 - 3 (1 - 8)
#define TMC2660_SEMAX               0   // 0 - 15
#define TMC2660_SEDN                3   // 0 - 15
#define TMC2660_SEIMIN              0   // 0 = 1/2 of CS, 1 = 1/4 of CS

// end of default values

typedef uint8_t tmc2660_regaddr_t;

//adresses are 3 bits.
enum tmc2660_regaddr_t {
    TMC2660Reg_DRVCTRL  = 0b000, // only ever going to use step/dir mode.
    TMC2660Reg_CHOPCONF = 0b100,
    TMC2660Reg_SMARTEN  = 0b101, // Coolstep control register.
    TMC2660Reg_SGCSCONF = 0b110, // Stallguard2 control register.
    TMC2660Reg_DRVCONF  = 0b111  // Driver Control Register.
};

typedef enum {
    TMC2660_RDSEL0 = 0,
    TMC2660_RDSEL1,
    TMC2660_RDSEL2,
    TMC2660_RDSEL3
} tmc2660_rdsel_t;

// --- register definitions ---

typedef union {
    uint32_t value;
    struct {
        uint32_t
        payload         :17,
        addr            :3;
    };
} TMC2660_datagram_t;

// DRVCTRL : RW
typedef union {
    TMC2660_datagram_t dgr;
    struct {
        uint32_t
        mres            :4,
        reserved1       :4,
        dedge           :1,
        intpol          :1,
        reserved2       :7;
    };
} TMC2660_drvctrl_reg_t;

// CHOPCONF : RW
typedef union {
    TMC2660_datagram_t dgr;
    struct {
        uint32_t
        toff            :4,
        hstrt           :3,
        hend            :4,
        hdec            :2,
        rndtf           :1,
        chm             :1,
        tbl             :2;
    };
} TMC2660_chopconf_reg_t;

// SMARTEN (Coolstep) : RW
typedef union {
    TMC2660_datagram_t dgr;
    struct {
        uint32_t
        semin           :4,
        reserved1       :1,
        seup            :2,
        reserved2       :1,
        semax           :4,
        reserved3       :1,
        sedn            :2,
        seimin          :1,
        reserved4       :1;
    };
} TMC2660_smarten_reg_t;

// SGCSCONF (Stallguard) : RW
typedef union {
    TMC2660_datagram_t dgr;
    struct {
        uint32_t
        cs              :5,
        reserved1       :3,
        sgt             :7,
        reserved2       :1,
        sfilt           :1;
    };
} TMC2660_sgcsconf_reg_t;

// DRVCONF (Driver control) : RW
typedef union {
    TMC2660_datagram_t dgr;
    struct {
        uint32_t
        en_s2vs         :1,
        en_pfd          :1,
        shrtsens        :1,
        otsens          :1,
        rdsel           :2,
        vsense          :1,
        sdoff           :1,
        ts2g            :2,
        diss2g          :1,
        reserved1       :1,
        slpl            :2,
        slph            :2,
        tst             :1;
    };
} TMC2660_drvconf_reg_t;

// DRV_STATUS : R  // this is the status return

typedef union {
    uint32_t value;
    struct {
        uint32_t
        sg         :1,
        ot         :1, //over temp shutdown
        otpw       :1, //over temp warning
        shorta     :1,
        shortb     :1,
        ola        :1,
        olb        :1,
        stst       :1,  //standstill indicator.
        chip_rev   :2,
        mstep_90   :10;
    };
} TMC2660_drvstatus0_t;

typedef union {
    uint32_t value;
    struct {
        uint32_t
        sg         :1,
        ot         :1, //over temp shutdown
        otpw       :1, //over temp warning
        shorta     :1,
        shortb     :1,
        ola        :1,
        olb        :1,
        stst       :1,  //standstill indicator.
        chip_rev   :2,
        sg_90      :10;
    };
} TMC2660_drvstatus1_t;

typedef union {
    uint32_t value;
    struct {
        uint32_t
        sg         :1,
        ot         :1, //over temp shutdown
        otpw       :1, //over temp warning
        shorta     :1,
        shortb     :1,
        ola        :1,
        olb        :1,
        stst       :1,  //standstill indicator.
        chip_rev   :2,
        se_40      :5,
        sg_59      :5;
    };
} TMC2660_drvstatus2_t;

typedef union {
    uint32_t value;
    struct {
        uint32_t
        sg         :1,
        ot         :1, //over temp shutdown
        otpw       :1, //over temp warning
        shorta     :1,
        shortb     :1,
        ola        :1,
        olb        :1,
        stst       :1,  //standstill indicator.
        chip_rev   :2,
        ot100      :1,
        ot120      :1,
        ot136      :1,
        ot150      :1,
        s2ga       :1,
        s2vsa      :1,
        s2gb       :1,
        s2vsb      :1,
        enn        :1,
        uv_7v      :1;
    };
} TMC2660_drvstatus3_t;

typedef union {
    TMC2660_datagram_t response;
    TMC2660_drvstatus0_t rdsel0;
    TMC2660_drvstatus1_t rdsel1;
    TMC2660_drvstatus2_t rdsel2;
    TMC2660_drvstatus3_t rdsel3;
} TMC2660_drvstatus_t;

// --- end of register definitions ---

// --- datagrams ---

typedef TMC2660_drvctrl_reg_t TMC2660_drvctrl_dgr_t;

typedef TMC2660_chopconf_reg_t TMC2660_chopconf_dgt_t;

typedef TMC2660_smarten_reg_t TMC2660_smarten_dgr_t;

typedef TMC2660_sgcsconf_reg_t TMC2660_sgcsconf_dgr_t;

typedef TMC2660_drvconf_reg_t TMC2660_drvconf_dgr_t;

typedef TMC2660_drvstatus_t TMC2660_drvstatus_dgr_t;

// -- end of datagrams

typedef union {
    uint32_t value;
    uint8_t data[3];
    TMC2660_drvctrl_reg_t drvctrl;
    TMC2660_chopconf_reg_t chopconf;
    TMC2660_smarten_reg_t smarten;
    TMC2660_sgcsconf_reg_t sgcsconf;
    TMC2660_drvconf_reg_t drvconf;
    TMC2660_drvstatus_t drvstatus;
} TMC2660_payload;

typedef struct {
    // driver registers
    TMC2660_drvctrl_dgr_t drvctrl;
    TMC2660_chopconf_dgt_t chopconf;
    TMC2660_smarten_dgr_t smarten;
    TMC2660_sgcsconf_dgr_t sgcsconf;
    TMC2660_drvconf_dgr_t drvconf;
    trinamic_config_t config;
} TMC2660_t;

#pragma pack(pop)

bool TMC2660_Init(TMC2660_t *driver);
void TMC2660_SetDefaults (TMC2660_t *driver);
const trinamic_cfg_params_t *TMC2660_GetConfigDefaults (void);
void TMC2660_SetCurrent (TMC2660_t *driver, uint16_t mA, uint8_t hold_pct);
uint16_t TMC2660_GetCurrent (TMC2660_t *driver, trinamic_current_t type);
bool TMC2660_MicrostepsIsValid (uint16_t usteps);
void TMC2660_SetMicrosteps(TMC2660_t *driver, tmc2660_microsteps_t usteps);

//stallguard functions

void TMC2660_SetConstantOffTimeChopper(TMC2660_t *driver, uint8_t constant_off_time, uint8_t blank_time, uint8_t fast_decay_time, int8_t sine_wave_offset, bool use_current_comparator);
TMC2660_datagram_t *TMC2660_GetRegPtr (TMC2660_t *driver, tmc2660_regaddr_t reg);
TMC2660_drvstatus_dgr_t TMC2660_WriteRegister (TMC2660_t *driver, TMC2660_datagram_t *reg);
TMC2660_drvstatus_dgr_t TMC2660_ReadRegister (TMC2660_t *driver, TMC2660_datagram_t *reg, uint8_t rdsel);

#endif
