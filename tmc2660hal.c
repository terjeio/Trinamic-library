/*
 * tmc2660hal.c - interface for Trinamic TMC2660 stepper driver
 *
 * v0.0.4 / 2025-01-17
 */

/*

Copyright (c) 2023-2024, Expatria Technologies,
Copyright (c) 2024-2025, Terje Io
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

#include <stdlib.h>
#include <string.h>

#include "grbl/hal.h"

#include "tmc2660.h"
#include "tmchal.h"

static TMC2660_t *tmcdriver[6];

static trinamic_config_t *getConfig (uint8_t motor)
{
    return &tmcdriver[motor]->config;
}

static bool isValidMicrosteps (uint8_t motor, uint16_t msteps)
{
    return tmc_microsteps_validate(msteps);
}

static void setMicrosteps (uint8_t motor, uint16_t msteps)
{
   TMC2660_SetMicrosteps(tmcdriver[motor], (tmc2660_microsteps_t)msteps);
   tmcdriver[motor]->config.microsteps = (tmc2660_microsteps_t)msteps;
}

static void setCurrent (uint8_t motor, uint16_t mA, uint8_t hold_pct)
{
    TMC2660_SetCurrent(tmcdriver[motor], mA, hold_pct);
}

static uint16_t getCurrent (uint8_t motor, trinamic_current_t type)
{
    return TMC2660_GetCurrent(tmcdriver[motor], type);
}

static TMC_chopconf_t getChopconf (uint8_t motor)
{
    TMC_chopconf_t chopconf;
    TMC2660_t *driver = tmcdriver[motor];

    chopconf.mres = driver->drvctrl.mres;
    chopconf.toff = driver->chopconf.toff;
    chopconf.tbl = driver->chopconf.tbl;
    chopconf.hend = driver->chopconf.hend;
    chopconf.hstrt = driver->chopconf.hstrt;

    return chopconf;
}

static uint32_t getStallGuardResult (uint8_t motor)
{
    TMC2660_drvstatus_dgr_t drvstatus = TMC2660_ReadRegister(tmcdriver[motor], NULL, TMC2660_RDSEL1);

    return (uint32_t)drvstatus.rdsel1.sg_90;
}

static TMC_drv_status_t getDriverStatus (uint8_t motor)
{
    TMC_drv_status_t drv_status = {0};
    TMC2660_drvstatus_dgr_t status = TMC2660_ReadRegister(tmcdriver[motor], NULL, TMC2660_RDSEL1);

//    drv_status.driver_error = status.value == 0 || status.value = 0xFFFFFF;
    drv_status.sg_result = status.rdsel1.sg_90;
    drv_status.ot = status.rdsel1.ot;
    drv_status.otpw = status.rdsel1.otpw;
    drv_status.cs_actual = tmcdriver[motor]->sgcsconf.cs;
    drv_status.stst = status.rdsel1.stst;
    drv_status.ola = status.rdsel1.ola;
    drv_status.olb = status.rdsel1.olb;
    drv_status.s2ga = status.rdsel1.shorta;
    drv_status.s2gb = status.rdsel1.shortb;
    drv_status.stallguard = status.rdsel1.sg;

    return drv_status;
}

static TMC_ihold_irun_t getIholdIrun (uint8_t motor)
{
    TMC_ihold_irun_t ihold_irun;

    ihold_irun.ihold = tmcdriver[motor]->config.hold_current_pct * tmcdriver[motor]->sgcsconf.cs / 100;
    ihold_irun.irun = tmcdriver[motor]->sgcsconf.cs;
    ihold_irun.iholddelay = 2; //standstill delay is fixed with 2660.

    return ihold_irun;
}

static uint32_t getDriverStatusRaw (uint8_t motor)  //only used for reporting
{
    TMC2660_drvstatus_dgr_t drvstatus = TMC2660_ReadRegister(tmcdriver[motor], NULL, (tmc2660_rdsel_t)tmcdriver[motor]->drvconf.rdsel);

    return drvstatus.response.value;
}

static void stallGuardEnable (uint8_t motor, float feed_rate, float steps_mm, int16_t sensitivity)
{
    TMC2660_t *driver = tmcdriver[motor];

    //sg_tst pin is always enabled.
    //driver->sgcsconf.sgt = sensitivity & 0x7F; // 7-bits signed value
    tmc_spi20_write(tmcdriver[motor]->config.motor, (TMC_spi20_datagram_t *)&driver->sgcsconf);
}

// stallguard filter
static void sg_filter (uint8_t motor, bool val)
{
    tmcdriver[motor]->sgcsconf.sfilt = val;
    tmc_spi20_write(tmcdriver[motor]->config.motor, (TMC_spi20_datagram_t *)&tmcdriver[motor]->sgcsconf);
}

static void sg_stall_value (uint8_t motor, int16_t val)
{
    tmcdriver[motor]->sgcsconf.sgt = val & 0x7F; // 7-bits signed value
    tmc_spi20_write(tmcdriver[motor]->config.motor, (TMC_spi20_datagram_t *)&tmcdriver[motor]->sgcsconf);
}

static int16_t get_sg_stall_value (uint8_t motor)
{
    return (int16_t)tmcdriver[motor]->sgcsconf.sgt;
}

static void coolconf (uint8_t motor, trinamic_coolconf_t coolconf)
{
    TMC2660_t *driver = tmcdriver[motor];

    driver->smarten.semin = coolconf.semin;
    driver->smarten.semax = coolconf.semax;
    driver->smarten.sedn = coolconf.sedn;
    driver->smarten.seimin = coolconf.seimin;
    driver->smarten.seup = coolconf.seup;

    tmc_spi20_write(tmcdriver[motor]->config.motor, (TMC_spi20_datagram_t *)&driver->smarten);
}

static void coolStepEnable (uint8_t motor)
{
    //coolstep is always enabled.
}
// chopconf

static void chopper_timing (uint8_t motor, trinamic_chopconf_t chopconf)
{
    TMC2660_t *driver = tmcdriver[motor];

    driver->chopconf.chm = chopconf.chm;
    driver->chopconf.hstrt = chopconf.hstrt;
    driver->chopconf.hend = chopconf.hend;
    driver->chopconf.tbl = chopconf.tbl;
    driver->chopconf.toff = chopconf.toff;

    tmc_spi20_write(driver->config.motor, (TMC_spi20_datagram_t *)&driver->chopconf);
}

static bool vsense (uint8_t motor)
{
    return tmcdriver[motor]->drvconf.vsense;
}

static bool read_register (uint8_t motor, uint8_t addr, uint32_t *val)
{
    TMC2660_datagram_t *reg;

    if((reg = TMC2660_GetRegPtr(tmcdriver[motor], addr)))
        *val = reg->payload;

    return reg != NULL;
}

static bool write_register (uint8_t motor, uint8_t addr, uint32_t val)
{
    TMC2660_datagram_t *reg;

    if((reg = TMC2660_GetRegPtr(tmcdriver[motor], addr))) {
        reg->payload = val;
        TMC2660_WriteRegister(tmcdriver[motor], reg);
    }

    return reg != NULL;
}

static void *get_register_addr (uint8_t motor, uint8_t addr)
{
    return TMC2660_GetRegPtr(tmcdriver[motor], (tmc2660_regaddr_t)addr);
}

static const tmchal_t tmchal = {
    .driver = TMC2660,
    .name = "TMC2660",
    .drvconf_address = TMC2660Reg_DRVCONF,

    .get_config = getConfig,

    .microsteps_isvalid = isValidMicrosteps,
    .set_microsteps = setMicrosteps,
    .set_current = setCurrent,
    .get_current = getCurrent,
    .get_chopconf = getChopconf,
    .get_drv_status = getDriverStatus,
    .get_drv_status_raw = getDriverStatusRaw,
    .stallguard_enable = stallGuardEnable,
    .coolstep_enable = coolStepEnable,
    .get_sg_result = getStallGuardResult,
    .get_ihold_irun = getIholdIrun,
    .sg_filter = sg_filter,
    .sg_stall_value = sg_stall_value,
    .get_sg_stall_value = get_sg_stall_value,
    .coolconf = coolconf,
    .vsense = vsense,
    .chopper_timing = chopper_timing,

    .get_register_addr = get_register_addr,
    .read_register = read_register,
    .write_register = write_register
};

const tmchal_t *TMC2660_AddMotor (motor_map_t motor, uint16_t current, uint8_t microsteps, uint8_t r_sense)
{
    bool ok = !!tmcdriver[motor.id];

    if(ok || (ok = (tmcdriver[motor.id] = malloc(sizeof(TMC2660_t))) != NULL)) {
        TMC2660_SetDefaults(tmcdriver[motor.id]);
        tmcdriver[motor.id]->config.motor.id = motor.id;
        tmcdriver[motor.id]->config.motor.axis = motor.axis;
        tmcdriver[motor.id]->config.current = current;
        tmcdriver[motor.id]->config.microsteps = microsteps;
        tmcdriver[motor.id]->config.r_sense = r_sense;
    }

    if(ok && !(ok = TMC2660_Init(tmcdriver[motor.id]))) {
        free(tmcdriver[motor.id]);
        tmcdriver[motor.id] = NULL;
    }

    return ok ? &tmchal : NULL;
}

const tmchal_t *TMC2660_AddNULLMotor (motor_map_t motor)
{
    return !!tmcdriver[motor.id] || !!(tmcdriver[motor.id] = malloc(sizeof(TMC2660_t))) ? &tmc_null_driver : NULL;
}
