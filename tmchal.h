/*
 * tmchal.h - HAL interface for Trinamic stepper drivers
 *
 * v0.0.5 / 2022-12-20 / (c) Io Engineering / Terje
 */

/*

Copyright (c) 2021-2022, Terje Io
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

#include "common.h"

// CHOPCONF : RW
typedef struct {
    uint32_t
    toff      :4,
    hstrt     :3,
    hend      :4,
    tbl       :2,
    mres      :4;
} TMC_chopconf_t;

// IHOLD_IRUN : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        ihold      :5,
        irun       :5,
        iholddelay :4;
    };
} TMC_ihold_irun_t;

// DRV_STATUS : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        sg_result  :10,
        s2vsa      :1,
        s2vsb      :1,
        stealth    :1,
        fsactive   :1,
        cs_actual  :5,
        stallguard :1,
        ot         :1,
        otpw       :1,
        s2ga       :1,
        s2gb       :1,
        ola        :1,
        olb        :1,
        stst       :1,
        driver_error : 1;
    };
} TMC_drv_status_t;


typedef struct {
    uint8_t semin;
    int8_t semax;
    uint8_t sedn;
} TMC_coolconf_t;

typedef struct {
    uint8_t hstrt;
    int8_t hend;
    uint8_t tbl;
    uint8_t toff;
} TMC_chopper_timing_t;

typedef trinamic_config_t *(*tmc_get_config)(uint8_t motor);

typedef bool (*tmc_microsteps_isvalid)(uint8_t motor, uint16_t microsteps);
typedef void (*tmc_set_microsteps)(uint8_t motor, uint16_t microsteps);
typedef void (*tmc_set_current)(uint8_t motor, uint16_t mA, uint8_t hold_pct);
typedef uint16_t (*tmc_get_current)(uint8_t motor);
typedef TMC_chopconf_t (*tmc_get_chopconf)(uint8_t motor);
typedef uint32_t (*tmc_get_tstep)(uint8_t motor);
typedef TMC_drv_status_t (*tmc_get_drv_status)(uint8_t motor);
typedef uint32_t (*tmc_get_drv_status_raw)(uint8_t motor);
typedef void (*tmc_set_tcoolthrs)(uint8_t motor, float mm_sec, float steps_mm);
typedef void (*tmc_set_tcoolthrs_raw)(uint8_t motor, uint32_t value);
typedef void (*tmc_set_thigh)(uint8_t motor, float mm_sec, float steps_mm);
typedef void (*tmc_set_thigh_raw)(uint8_t motor, uint32_t value);
typedef void (*tmc_stallguard_enable)(uint8_t motor, float feed_rate, float steps_mm, int16_t sensitivity);
typedef void (*tmc_stealthchop_enable)(uint8_t motor);
typedef uint32_t (*tmc_get_sg_result)(uint8_t motor);
typedef void (*tmc_coolstep_enable)(uint8_t motor);
typedef uint32_t (*tmc_get_tpwmthrs_raw)(uint8_t motor);
typedef float (*tmc_get_tpwmthrs)(uint8_t motor, float steps_mm);
typedef void (*tmc_set_tpwmthrs)(uint8_t motor, float mm_sec, float steps_mm);
typedef uint8_t (*tmc_get_global_scaler)(uint8_t motor);
typedef bool (*tmc_get_en_pwm_mode)(uint8_t motor);
typedef TMC_ihold_irun_t (*tmc_get_ihold_irun)(uint8_t motor);
typedef void (*tmc_stealthChop)(uint8_t motor, bool on);
typedef void (*tmc_sg_filter)(uint8_t motor, bool on);
typedef void (*tmc_sg_stall_value)(uint8_t motor, int16_t val);
typedef int16_t (*tmc_get_sg_stall_value)(uint8_t motor);
typedef uint8_t (*tmc_pwm_scale)(uint8_t motor);
typedef bool (*tmc_vsense)(uint8_t motor);
typedef void (*tmc_coolconf)(uint8_t motor, TMC_coolconf_t coolconf);
typedef void (*tmc_chopper_timing)(uint8_t motor, TMC_chopper_timing_t timing);
typedef bool (*tmc_read_register)(uint8_t motor, uint8_t addr, uint32_t *val);
typedef bool (*tmc_write_register)(uint8_t motor, uint8_t addr, uint32_t val);
typedef void *(*tmc_get_register_addr)(uint8_t motor, uint8_t addr);

typedef struct {
    const char *name;
    trinamic_driver_t driver;

    tmc_get_config get_config;

    tmc_microsteps_isvalid microsteps_isvalid;
    tmc_set_microsteps set_microsteps;
    tmc_set_current set_current;
    tmc_get_current get_current;
    tmc_get_chopconf get_chopconf;
    tmc_get_tstep get_tstep;
    tmc_get_drv_status get_drv_status;
    tmc_get_drv_status_raw get_drv_status_raw;
    tmc_set_tcoolthrs set_tcoolthrs;
    tmc_set_tcoolthrs_raw set_tcoolthrs_raw;
    tmc_set_thigh set_thigh;
    tmc_set_thigh_raw set_thigh_raw;
    tmc_get_sg_result get_sg_result;
    tmc_stallguard_enable stallguard_enable;
    tmc_stealthchop_enable stealthchop_enable;
    tmc_coolstep_enable coolstep_enable;
    tmc_get_tpwmthrs_raw get_tpwmthrs_raw;
    tmc_get_tpwmthrs get_tpwmthrs;
    tmc_set_tpwmthrs set_tpwmthrs;
    tmc_get_global_scaler get_global_scaler;
    tmc_get_en_pwm_mode get_en_pwm_mode;
    tmc_get_ihold_irun get_ihold_irun;

    tmc_stealthChop stealthChop;
    tmc_sg_filter sg_filter;
    tmc_sg_stall_value sg_stall_value;
    tmc_get_sg_stall_value get_sg_stall_value;
    tmc_vsense vsense;
    tmc_coolconf coolconf;
    tmc_pwm_scale pwm_scale;
    tmc_chopper_timing chopper_timing;
    tmc_get_register_addr get_register_addr;
    tmc_read_register read_register;
    tmc_write_register write_register;
} tmchal_t;
