/*
 * tmc2240hal.c - interface for Trinamic TMC2240 stepper driver
 *
 * v0.0.1 / 2025-06-08
 */

/*

Copyright (c) 2025, Terje Io
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

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "grbl/hal.h"

#include "tmc2240.h"
#include "tmchal.h"

static TMC2240_t *tmcdriver[6];

static trinamic_config_t *getConfig (uint8_t motor)
{
    return &tmcdriver[motor]->config;
}

static float getTemp (uint8_t motor)
{
    tmc2240_read(tmcdriver[motor], adc_temp);

    return (float)(tmcdriver[motor]->adc_temp.reg.adc_temp - 2038) / 7.7f;
}

static bool isValidMicrosteps (uint8_t motor, uint16_t msteps)
{
    return tmc_microsteps_validate(msteps);
}

static void setMicrosteps (uint8_t motor, uint16_t msteps)
{
   TMC2240_SetMicrosteps(tmcdriver[motor], (tmc2240_microsteps_t)msteps);
}

static void setCurrent (uint8_t motor, uint16_t mA, uint8_t hold_pct)
{
    TMC2240_SetCurrent(tmcdriver[motor], mA, hold_pct);
}

static uint16_t getCurrent (uint8_t motor, trinamic_current_t type)
{
    return TMC2240_GetCurrent(tmcdriver[motor], type);
}

static TMC_chopconf_t getChopconf (uint8_t motor)
{
    TMC_chopconf_t chopconf;
    TMC2240_t *driver = tmcdriver[motor];

    tmc2240_read(tmcdriver[motor], chopconf);

    chopconf.mres = driver->chopconf.reg.mres;
    chopconf.toff = driver->chopconf.reg.toff;
    chopconf.tbl = driver->chopconf.reg.tbl;
    chopconf.hend = driver->chopconf.reg.hend_offset;
    chopconf.hstrt = driver->chopconf.reg.hstrt_tfd;

    return chopconf;
}

static uint32_t getStallGuardResult (uint8_t motor)
{
    tmc2240_read(tmcdriver[motor], sg4_result);

    return (uint32_t)tmcdriver[motor]->sg4_result.reg.sg4_result;
}

static TMC_drv_status_t getDriverStatus (uint8_t motor)
{
    TMC_drv_status_t drv_status = {};

#ifdef TMC_UART
    drv_status.driver_error = !tmc2240_read(tmcdriver[motor], drv_status);
#else
    TMC2240_status_t status;
    status.value = tmc2240_read(tmcdriver[motor], drv_status);
    drv_status.driver_error = status.driver_error;
#endif

    drv_status.ot = tmcdriver[motor]->drv_status.reg.ot;
    drv_status.otpw = tmcdriver[motor]->drv_status.reg.otpw;
    drv_status.cs_actual = tmcdriver[motor]->drv_status.reg.cs_actual;
    drv_status.stst = tmcdriver[motor]->drv_status.reg.stst;
    drv_status.fsactive = tmcdriver[motor]->drv_status.reg.fsactive;
    drv_status.ola = tmcdriver[motor]->drv_status.reg.ola;
    drv_status.olb = tmcdriver[motor]->drv_status.reg.olb;
    drv_status.s2ga = tmcdriver[motor]->drv_status.reg.s2ga;
    drv_status.s2gb = tmcdriver[motor]->drv_status.reg.s2gb;

    tmc2240_read(tmcdriver[motor], sg4_result);
    drv_status.sg_result = tmcdriver[motor]->sg4_result.reg.sg4_result;

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
    tmc2240_read(tmcdriver[motor], drv_status);

    return tmcdriver[motor]->drv_status.reg.value;
}

static uint32_t getTStep (uint8_t motor)
{
    tmc2240_read(tmcdriver[motor], tstep);

    return (uint32_t)tmcdriver[motor]->tstep.reg.tstep;
}

static void setTHigh (uint8_t motor, float mm_sec, float steps_mm)
{
    TMC2240_SetTHIGH(tmcdriver[motor], mm_sec, steps_mm);
}

static void setTHighRaw (uint8_t motor, uint32_t value)
{
    tmcdriver[motor]->thigh.reg.thigh = value;
    tmc2240_write(tmcdriver[motor], thigh);
}


static void setTCoolThrs (uint8_t motor, float mm_sec, float steps_mm)
{
    TMC2240_SetTCOOLTHRS(tmcdriver[motor], mm_sec, steps_mm);
}

static void setTCoolThrsRaw (uint8_t motor, uint32_t value)
{
    tmcdriver[motor]->tcoolthrs.reg.tcoolthrs = value;
    tmc2240_write(tmcdriver[motor], tcoolthrs);
}

static void stallGuardEnable (uint8_t motor, float feed_rate, float steps_mm, int16_t sensitivity)
{
    TMC2240_t *driver = tmcdriver[motor];

    driver->gconf.reg.diag1_stall = true;
    driver->gconf.reg.en_pwm_mode = true; // stealthChop on
    tmc2240_write(driver, gconf);

    driver->pwmconf.reg.pwm_autoscale = true;
    tmc2240_write(driver, pwmconf);

    TMC2240_SetTCOOLTHRS(driver, feed_rate / (60.0f * 1.5f), steps_mm);
//    TMC2240_SetTHIGH(driver, feed_rate / 60.0f * 0.6f, steps_mm);

    driver->sg4_thrs.reg.sg4_thrs = sensitivity & 0xFF;
    tmc2240_write(driver, sg4_thrs);
}

static void stealthChopEnable (uint8_t motor)
{
    TMC2240_t *driver = tmcdriver[motor];

    driver->gconf.reg.diag1_stall = false;
    driver->gconf.reg.en_pwm_mode = true; // stealthChop on
    tmc2240_write(driver, gconf);

    driver->pwmconf.reg.pwm_autoscale = true;
    tmc2240_write(driver, pwmconf);

    setTCoolThrsRaw(motor, 0);
}

static void coolStepEnable (uint8_t motor)
{
    TMC2240_t *driver = tmcdriver[motor];

    driver->gconf.reg.diag1_stall = false;
    driver->gconf.reg.en_pwm_mode = false; // stealthChop off
    tmc2240_write(driver, gconf);

    driver->pwmconf.reg.pwm_autoscale = false;
    tmc2240_write(driver, pwmconf);

    setTCoolThrsRaw(motor, 0);
}

static float getTPWMThrs (uint8_t motor, float steps_mm)
{
    return TMC2240_GetTPWMTHRS(tmcdriver[motor], steps_mm);
}

static uint32_t getTPWMThrsRaw (uint8_t motor)
{
    return tmcdriver[motor]->tpwmthrs.reg.tpwmthrs;
}

static void setTPWMThrs (uint8_t motor, float mm_sec, float steps_mm)
{
    TMC2240_SetTPWMTHRS(tmcdriver[motor], mm_sec, steps_mm);
}

static uint8_t getGlobalScaler (uint8_t motor)
{
    return tmcdriver[motor]->global_scaler.reg.globalscaler == 0 ? 256 : tmcdriver[motor]->global_scaler.reg.globalscaler;
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
    tmcdriver[motor]->sg4_thrs.reg.sg4_fil_en = val;
    tmc2240_write(tmcdriver[motor], sg4_thrs);
}

static void sg_stall_value (uint8_t motor, int16_t val)
{
    tmcdriver[motor]->sg4_thrs.reg.sg4_thrs = val & 0xFF;
    tmc2240_write(tmcdriver[motor], sg4_thrs);
}

static int16_t get_sg_stall_value (uint8_t motor)
{
    return (int16_t)(tmcdriver[motor]->coolconf.reg.sgt & 0x40 ? tmcdriver[motor]->coolconf.reg.sgt | 0xFF80 : tmcdriver[motor]->coolconf.reg.sgt);
}

static void coolconf (uint8_t motor, trinamic_coolconf_t coolconf)
{
    TMC2240_t *driver = tmcdriver[motor];

    driver->coolconf.reg.semin = coolconf.semin;
    driver->coolconf.reg.semax = coolconf.semax;
    driver->coolconf.reg.sedn = coolconf.sedn;
    driver->coolconf.reg.seimin = coolconf.seimin;
    driver->coolconf.reg.seup = coolconf.seup;
    tmc_spi_write(tmcdriver[motor]->config.motor, (TMC_spi_datagram_t *)&driver->coolconf);
}

// chopconf

static void chopper_timing (uint8_t motor, trinamic_chopconf_t chopconf)
{
    TMC2240_t *driver = tmcdriver[motor];

    driver->chopconf.reg.chm = chopconf.chm;
    driver->chopconf.reg.hstrt_tfd = chopconf.hstrt;
    driver->chopconf.reg.hend_offset = chopconf.hend;
    driver->chopconf.reg.tbl = chopconf.tbl;
    driver->chopconf.reg.toff = chopconf.toff;
    tmc2240_write(driver, chopconf);
}

static uint8_t pwm_scale (uint8_t motor)
{
    tmc2240_read(tmcdriver[motor], pwm_scale);

    return tmcdriver[motor]->pwm_scale.reg.pwm_scale_sum;
}

static bool read_register (uint8_t motor, uint8_t addr, uint32_t *val)
{
    TMC2240_datagram_t reg;
    reg.addr.reg = (tmc2240_regaddr_t)addr;
    reg.addr.write = Off;

    TMC2240_ReadRegister(tmcdriver[motor], &reg);

    *val = reg.payload.value;

    return true;
}

static bool write_register (uint8_t motor, uint8_t addr, uint32_t val)
{
    TMC2240_datagram_t reg;
    reg.addr.reg = (tmc2240_regaddr_t)addr;
    reg.addr.write = On;
    reg.payload.value = val;

    TMC2240_WriteRegister(tmcdriver[motor], &reg);

    return true;
}

static void *get_register_addr (uint8_t motor, uint8_t addr)
{
    return TMC2240_GetRegPtr(tmcdriver[motor], (tmc2240_regaddr_t)addr);
}

static const tmchal_t tmchal = {
    .driver = TMC2240,
    .name = "TMC2240",
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
    .get_temp = getTemp,
    .chopper_timing = chopper_timing,

    .get_register_addr = get_register_addr,
    .read_register = read_register,
    .write_register = write_register
};

#ifdef TMC_UART

const tmchal_t *TMC2240_AddMotor (motor_map_t motor, uint8_t address, uint16_t current, uint8_t microsteps, uint8_t r_sense)
{
    bool ok = !!tmcdriver[motor.id];

    if(ok || (ok = (tmcdriver[motor.id] = malloc(sizeof(TMC2240_t))) != NULL)) {
        TMC2240_SetDefaults(tmcdriver[motor.id]);
        tmcdriver[motor.id]->config.motor.id = motor.id;
        tmcdriver[motor.id]->config.motor.address = address;
        tmcdriver[motor.id]->config.motor.axis = motor.axis;
        tmcdriver[motor.id]->config.current = current;
        tmcdriver[motor.id]->config.microsteps = microsteps;
        tmcdriver[motor.id]->config.r_sense = r_sense;
        tmcdriver[motor.id]->chopconf.reg.mres = tmc_microsteps_to_mres(microsteps);
    }

    if(ok && !(ok = TMC2240_Init(tmcdriver[motor.id]))) {
        free(tmcdriver[motor.id]);
        tmcdriver[motor.id] = NULL;
    }

    return ok ? &tmchal : NULL;
}

#else

const tmchal_t *TMC2240_AddMotor (motor_map_t motor, uint16_t current, uint8_t microsteps, uint8_t r_sense)
{
    bool ok = !!tmcdriver[motor.id];

    if(ok || (ok = (tmcdriver[motor.id] = malloc(sizeof(TMC2240_t))) != NULL)) {
        TMC2240_SetDefaults(tmcdriver[motor.id]);
        tmcdriver[motor.id]->config.motor.id = motor.id;
        tmcdriver[motor.id]->config.motor.axis = motor.axis;
        tmcdriver[motor.id]->config.current = current;
        tmcdriver[motor.id]->config.microsteps = microsteps;
        tmcdriver[motor.id]->config.r_sense = r_sense;
        tmcdriver[motor.id]->chopconf.reg.mres = tmc_microsteps_to_mres(microsteps);
    }

    if(ok && !(ok = TMC2240_Init(tmcdriver[motor.id]))) {
        free(tmcdriver[motor.id]);
        tmcdriver[motor.id] = NULL;
    }

    return ok ? &tmchal : NULL;
}

#endif
