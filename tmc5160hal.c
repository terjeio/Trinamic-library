/*
 * tmc5160hal.c - interface for Trinamic TMC5160 stepper driver
 *
 * v0.0.5 / 2022-12-22 / (c) Io Engineering / Terje
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

#include <stdlib.h>
#include <string.h>

#include "grbl/hal.h"

#include "tmc5160.h"
#include "tmchal.h"

static TMC5160_t *tmcdriver[6];

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
   TMC5160_SetMicrosteps(tmcdriver[motor], (tmc5160_microsteps_t)msteps);
}

static void setCurrent (uint8_t motor, uint16_t mA, uint8_t hold_pct)
{
    TMC5160_SetCurrent(tmcdriver[motor], mA, hold_pct);
}

static uint16_t getCurrent (uint8_t motor)
{
    return TMC5160_GetCurrent(tmcdriver[motor]);
}

static TMC_chopconf_t getChopconf (uint8_t motor)
{
    TMC_chopconf_t chopconf;
    TMC5160_t *driver = tmcdriver[motor];

    tmc_spi_read(tmcdriver[motor]->config.motor, (TMC_spi_datagram_t *)&tmcdriver[motor]->chopconf);

    chopconf.mres = driver->chopconf.reg.mres;
    chopconf.toff = driver->chopconf.reg.toff;
    chopconf.tbl = driver->chopconf.reg.tbl;
    chopconf.hend = driver->chopconf.reg.hend;
    chopconf.hstrt = driver->chopconf.reg.hstrt;

    return chopconf;
}

static uint32_t getStallGuardResult (uint8_t motor)
{
    tmc_spi_read(tmcdriver[motor]->config.motor, (TMC_spi_datagram_t *)&tmcdriver[motor]->drv_status);

    return (uint32_t)tmcdriver[motor]->drv_status.reg.sg_result;
}

static TMC_drv_status_t getDriverStatus (uint8_t motor)
{
    TMC_drv_status_t drv_status;
    TMC5160_status_t status;

    status.value = tmc_spi_read(tmcdriver[motor]->config.motor, (TMC_spi_datagram_t *)&tmcdriver[motor]->drv_status);

    drv_status.driver_error = status.driver_error;
    drv_status.sg_result = tmcdriver[motor]->drv_status.reg.sg_result;
    drv_status.ot = tmcdriver[motor]->drv_status.reg.ot;
    drv_status.otpw = tmcdriver[motor]->drv_status.reg.otpw;
    drv_status.cs_actual = tmcdriver[motor]->drv_status.reg.cs_actual;
    drv_status.stst = tmcdriver[motor]->drv_status.reg.stst;
    drv_status.fsactive = tmcdriver[motor]->drv_status.reg.fsactive;
    drv_status.ola = tmcdriver[motor]->drv_status.reg.ola;
    drv_status.olb = tmcdriver[motor]->drv_status.reg.olb;
    drv_status.s2ga = tmcdriver[motor]->drv_status.reg.s2ga;
    drv_status.s2gb = tmcdriver[motor]->drv_status.reg.s2gb;

    return drv_status;
}

static TMC_ihold_irun_t getIholdIrun (uint8_t motor)
{
    TMC_ihold_irun_t ihold_irun;

    ihold_irun.ihold = tmcdriver[motor]->ihold_irun.reg.ihold;
    ihold_irun.irun = tmcdriver[motor]->ihold_irun.reg.irun;
    ihold_irun.iholddelay = tmcdriver[motor]->ihold_irun.reg.iholddelay;

    return ihold_irun;
}

static uint32_t getDriverStatusRaw (uint8_t motor)
{
    tmc_spi_read(tmcdriver[motor]->config.motor, (TMC_spi_datagram_t *)&tmcdriver[motor]->drv_status);

    return tmcdriver[motor]->drv_status.reg.value;
}

static uint32_t getTStep (uint8_t motor)
{
    tmc_spi_read(tmcdriver[motor]->config.motor, (TMC_spi_datagram_t *)&tmcdriver[motor]->tstep);

    return (uint32_t)tmcdriver[motor]->tstep.reg.tstep;
}

static void setTHigh (uint8_t motor, float mm_sec, float steps_mm)
{
    TMC5160_SetTHIGH(tmcdriver[motor], mm_sec, steps_mm);
}

static void setTHighRaw (uint8_t motor, uint32_t value)
{
    tmcdriver[motor]->thigh.reg.thigh = value;
    tmc_spi_write(tmcdriver[motor]->config.motor, (TMC_spi_datagram_t *)&tmcdriver[motor]->thigh);
}


static void setTCoolThrs (uint8_t motor, float mm_sec, float steps_mm)
{
    TMC5160_SetTCOOLTHRS(tmcdriver[motor], mm_sec, steps_mm);
}

static void setTCoolThrsRaw (uint8_t motor, uint32_t value)
{
    tmcdriver[motor]->tcoolthrs.reg.tcoolthrs = value;
    tmc_spi_write(tmcdriver[motor]->config.motor, (TMC_spi_datagram_t *)&tmcdriver[motor]->tcoolthrs);
}

static void stallGuardEnable (uint8_t motor, float feed_rate, float steps_mm, int16_t sensitivity)
{
    TMC5160_t *driver = tmcdriver[motor];

    driver->gconf.reg.diag1_stall = true;
    driver->gconf.reg.en_pwm_mode = false; // stealthChop
    tmc_spi_write(driver->config.motor, (TMC_spi_datagram_t *)&driver->gconf);

    driver->pwmconf.reg.pwm_autoscale = false;
    tmc_spi_write(driver->config.motor, (TMC_spi_datagram_t *)&driver->pwmconf);

    TMC5160_SetTCOOLTHRS(driver, feed_rate / (60.0f * 1.5f), steps_mm);
    TMC5160_SetTHIGH(driver, feed_rate / 60.0f * 0.6f, steps_mm);

    driver->coolconf.reg.sgt = sensitivity & 0x7F; // 7-bits signed value
    tmc_spi_write(driver->config.motor, (TMC_spi_datagram_t *)&driver->coolconf);
}

static void stealthChopEnable (uint8_t motor)
{
    TMC5160_t *driver = tmcdriver[motor];

    driver->gconf.reg.diag1_stall = false;
    driver->gconf.reg.en_pwm_mode = true; // stealthChop
    tmc_spi_write(driver->config.motor, (TMC_spi_datagram_t *)&driver->gconf);

    driver->pwmconf.reg.pwm_autoscale = true;
    tmc_spi_write(driver->config.motor, (TMC_spi_datagram_t *)&driver->pwmconf);

    setTCoolThrsRaw(motor, 0);
}

static void coolStepEnable (uint8_t motor)
{
    TMC5160_t *driver = tmcdriver[motor];

    driver->gconf.reg.en_pwm_mode = false; // stealthChop
    tmc_spi_write(driver->config.motor, (TMC_spi_datagram_t *)&driver->gconf);

    driver->pwmconf.reg.pwm_autoscale = false;
    tmc_spi_write(driver->config.motor, (TMC_spi_datagram_t *)&driver->pwmconf);

    setTCoolThrsRaw(motor, 0);
}

static float getTPWMThrs (uint8_t motor, float steps_mm)
{
    return TMC5160_GetTPWMTHRS(tmcdriver[motor], steps_mm);
}

static uint32_t getTPWMThrsRaw (uint8_t motor)
{
    return tmcdriver[motor]->tpwmthrs.reg.tpwmthrs;
}

static void setTPWMThrs (uint8_t motor, float mm_sec, float steps_mm)
{
    TMC5160_SetTPWMTHRS(tmcdriver[motor], mm_sec, steps_mm);
}

static uint8_t getGlobalScaler (uint8_t motor)
{
    return tmcdriver[motor]->global_scaler.reg.scaler;
}

static void stealthChop (uint8_t motor, bool on)
{
    tmcdriver[motor]->config.mode = on ? TMCMode_StealthChop : TMCMode_CoolStep;

    if(on)
        stealthChopEnable(motor);
    else
        coolStepEnable(motor);
}

static bool stealthChopGet (uint8_t motor)
{
    return tmcdriver[motor]->gconf.reg.en_pwm_mode;
}

// coolconf

static void sg_filter (uint8_t motor, bool val)
{
    tmcdriver[motor]->coolconf.reg.sfilt = val;
    tmc_spi_write(tmcdriver[motor]->config.motor, (TMC_spi_datagram_t *)&tmcdriver[motor]->coolconf);
}

static void sg_stall_value (uint8_t motor, int16_t val)
{
    tmcdriver[motor]->coolconf.reg.sgt = val & 0x7F; // 7-bits signed value
    tmc_spi_write(tmcdriver[motor]->config.motor, (TMC_spi_datagram_t *)&tmcdriver[motor]->coolconf);
}

static int16_t get_sg_stall_value (uint8_t motor)
{
    return (int16_t)(tmcdriver[motor]->coolconf.reg.sgt & 0x40 ? tmcdriver[motor]->coolconf.reg.sgt | 0xFF80 : tmcdriver[motor]->coolconf.reg.sgt);
}

static void coolconf (uint8_t motor, TMC_coolconf_t coolconf)
{
    TMC5160_t *driver = tmcdriver[motor];

    driver->coolconf.reg.semin = coolconf.semin;
    driver->coolconf.reg.semax = coolconf.semax;
    driver->coolconf.reg.sedn = coolconf.sedn;
    tmc_spi_write(tmcdriver[motor]->config.motor, (TMC_spi_datagram_t *)&driver->coolconf);
}

// chopconf

static void chopper_timing (uint8_t motor, TMC_chopper_timing_t timing)
{
    TMC5160_t *driver = tmcdriver[motor];

    driver->chopconf.reg.chm = 0;
    driver->chopconf.reg.hstrt = timing.hstrt + 1;
    driver->chopconf.reg.hend = timing.hend + 3;
    driver->chopconf.reg.tbl = timing.tbl;
    driver->chopconf.reg.toff = timing.toff;
    tmc_spi_write(driver->config.motor, (TMC_spi_datagram_t *)&driver->chopconf);
}

static uint8_t pwm_scale (uint8_t motor)
{
    tmc_spi_read(tmcdriver[motor]->config.motor, (TMC_spi_datagram_t *)&tmcdriver[motor]->pwm_scale);

    return tmcdriver[motor]->pwm_scale.reg.pwm_scale_sum;
}

static bool read_register (uint8_t motor, uint8_t addr, uint32_t *val)
{
    TMC5160_datagram_t reg;
    reg.addr.reg = (tmc5160_regaddr_t)addr;
    reg.addr.write = Off;

    TMC5160_ReadRegister(tmcdriver[motor], &reg);

    *val = reg.payload.value;

    return true;
}

static bool write_register (uint8_t motor, uint8_t addr, uint32_t val)
{
    TMC5160_datagram_t reg;
    reg.addr.reg = (tmc5160_regaddr_t)addr;
    reg.addr.write = On;
    reg.payload.value = val;

    TMC5160_WriteRegister(tmcdriver[motor], &reg);

    return true;
}

static void *get_register_addr (uint8_t motor, uint8_t addr)
{
    return TMC5160_GetRegPtr(tmcdriver[motor], (tmc5160_regaddr_t)addr);
}

static const tmchal_t tmchal = {
    .driver = TMC5160,
    .name = "TMC5160",
    .get_config = getConfig,

    .microsteps_isvalid = isValidMicrosteps,
    .set_microsteps = setMicrosteps,
    .set_current = setCurrent,
    .get_current = getCurrent,
    .get_chopconf = getChopconf,
    .get_tstep = getTStep,
    .get_drv_status = getDriverStatus,
    .get_drv_status_raw = getDriverStatusRaw,
    .set_tcoolthrs = setTCoolThrs,
    .set_tcoolthrs_raw = setTCoolThrsRaw,
    .set_thigh = setTHigh,
    .set_thigh_raw = setTHighRaw,
    .stallguard_enable = stallGuardEnable,
    .stealthchop_enable = stealthChopEnable,
    .coolstep_enable = coolStepEnable,
    .get_sg_result = getStallGuardResult,
    .get_tpwmthrs = getTPWMThrs,
    .get_tpwmthrs_raw = getTPWMThrsRaw,
    .set_tpwmthrs = setTPWMThrs,
    .get_global_scaler = getGlobalScaler,
    .get_en_pwm_mode = stealthChopGet,
    .get_ihold_irun = getIholdIrun,

    .stealthChop = stealthChop,
    .sg_filter = sg_filter,
    .sg_stall_value = sg_stall_value,
    .get_sg_stall_value = get_sg_stall_value,
    .coolconf = coolconf,
    .pwm_scale = pwm_scale,
    .chopper_timing = chopper_timing,

    .get_register_addr = get_register_addr,
    .read_register = read_register,
    .write_register = write_register
};

const tmchal_t *TMC5160_AddMotor (motor_map_t motor, uint16_t current, uint8_t microsteps, uint8_t r_sense)
{
    bool ok = !!tmcdriver[motor.id];

    if(ok || (ok = (tmcdriver[motor.id] = malloc(sizeof(TMC5160_t))) != NULL)) {
        TMC5160_SetDefaults(tmcdriver[motor.id]);
        tmcdriver[motor.id]->config.motor.id = motor.id;
        tmcdriver[motor.id]->config.motor.axis = motor.axis;
        tmcdriver[motor.id]->config.current = current;
        tmcdriver[motor.id]->config.microsteps = microsteps;
        tmcdriver[motor.id]->config.r_sense = r_sense;
        tmcdriver[motor.id]->chopconf.reg.mres = tmc_microsteps_to_mres(microsteps);
    }

    if(ok && !(ok = TMC5160_Init(tmcdriver[motor.id]))) {
        free(tmcdriver[motor.id]);
        tmcdriver[motor.id] = NULL;
    }

    return ok ? &tmchal : NULL;
}
