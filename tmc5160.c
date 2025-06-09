/*
 * tmc5160.c - interface for Trinamic TMC5160 stepper driver
 *
 * v0.0.7 / 2024-12-05
 */

/*

Copyright (c) 2021-2024, Terje Io
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

/*
 * Reference for calculations:
 * https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC5160_Calculations.xlsx
 *
 */

#include <string.h>

#include "tmc5160.h"

static const trinamic_cfg_params_t cfg_params = {

    .cap.drvconf = 0,

    .cap.coolconf.seup = 0b11,
    .cap.coolconf.sedn = 0b11,
    .cap.coolconf.semax = 0b1111,
    .cap.coolconf.semin = 0b1111,
    .cap.coolconf.seimin = 1,

    .cap.chopconf.toff = 0b1111,
    .cap.chopconf.hstrt = 0b111,
    .cap.chopconf.hend = 0b1111,
    .cap.chopconf.rndtf = 1,
    .cap.chopconf.intpol = 1,
    .cap.chopconf.tbl = 0b11,

    .dflt.drvconf = 0,

    .dflt.coolconf.seup = TMC5160_SEUP,
    .dflt.coolconf.sedn = TMC5160_SEDN,
    .dflt.coolconf.semax = TMC5160_SEMAX,
    .dflt.coolconf.semin = TMC5160_SEMIN,
    .dflt.coolconf.seimin = TMC5160_SEIMIN,

    .dflt.chopconf.toff = TMC5160_TOFF,
    .dflt.chopconf.hstrt = TMC5160_HSTRT - 1,
    .dflt.chopconf.hend = TMC5160_HEND + 3,
    .dflt.chopconf.intpol = TMC5160_INTPOL,
    .dflt.chopconf.tfd = TMC5160_TFD,
    .dflt.chopconf.tbl = TMC5160_TBL
};

static const TMC5160_t tmc5160_defaults = {
    .config.f_clk = TMC5160_F_CLK,
    .config.mode = TMC5160_MODE,
    .config.r_sense = TMC5160_R_SENSE,
    .config.current = TMC5160_CURRENT,
    .config.hold_current_pct = TMC5160_HOLD_CURRENT_PCT,
    .config.microsteps = TMC5160_MICROSTEPS,

    // register adresses
    .gconf.addr.reg = TMC5160Reg_GCONF,
    .gconf.reg.en_pwm_mode = TMC5160_EN_PWM_MODE,
    .gstat.addr.reg = TMC5160Reg_GSTAT,
    .ioin.addr.reg = TMC5160Reg_IOIN,
    .global_scaler.addr.reg = TMC5160Reg_GLOBAL_SCALER,
    .ihold_irun.addr.reg = TMC5160Reg_IHOLD_IRUN,
    .ihold_irun.reg.iholddelay = TMC5160_IHOLDDELAY,
    .tpowerdown.addr.reg = TMC5160Reg_TPOWERDOWN,
    .tpowerdown.reg.tpowerdown = TMC5160_TPOWERDOWN,
    .tstep.addr.reg = TMC5160Reg_TSTEP,
    .tpwmthrs.addr.reg = TMC5160Reg_TPWMTHRS,
    .tpwmthrs.reg.tpwmthrs = TMC5160_TPWM_THRS,
    .tcoolthrs.addr.reg = TMC5160Reg_TCOOLTHRS,
    .tcoolthrs.reg.tcoolthrs = TMC5160_COOLSTEP_THRS,
    .thigh.addr.reg = TMC5160Reg_THIGH,
    .vdcmin.addr.reg = TMC5160Reg_VDCMIN,
    .mscnt.addr.reg = TMC5160Reg_MSCNT,
    .mscuract.addr.reg = TMC5160Reg_MSCURACT,
    .chopconf.addr.reg = TMC5160Reg_CHOPCONF,
    .chopconf.reg.intpol = TMC5160_INTPOL,
    .chopconf.reg.toff = TMC5160_TOFF,
    .chopconf.reg.chm = TMC5160_CHM,
    .chopconf.reg.tbl = TMC5160_TBL,
    .chopconf.reg.hend = TMC5160_HEND + 3,
#if TMC5160_CHM == 0
    .chopconf.reg.hstrt = TMC5160_HSTRT - 1,
#else
    .chopconf.reg.fd3 = (TMC5160_TFD & 0x08) >> 3,
    .chopconf.reg.hstrt = TMC5160_TFD & 0x07,
#endif
    .coolconf.addr.reg = TMC5160Reg_COOLCONF,
    .coolconf.reg.semin = TMC5160_SEMIN,
    .coolconf.reg.seup = TMC5160_SEUP,
    .coolconf.reg.semax = TMC5160_SEMAX,
    .coolconf.reg.sedn = TMC5160_SEDN,
    .coolconf.reg.seimin = TMC5160_SEIMIN,
    .dcctrl.addr.reg = TMC5160Reg_DCCTRL,
    .drv_status.addr.reg = TMC5160Reg_DRV_STATUS,
    .pwmconf.addr.reg = TMC5160Reg_PWMCONF,
    .pwmconf.reg.pwm_autoscale = TMC5160_PWM_AUTOSCALE,
    .pwmconf.reg.pwm_lim = TMC5160_PWM_LIM,
    .pwmconf.reg.pwm_reg = TMC5160_PWM_REG,
    .pwmconf.reg.pwm_autograd = TMC5160_PWM_AUTOGRAD,
    .pwmconf.reg.pwm_freq = TMC5160_PWM_FREQ,
    .pwmconf.reg.pwm_grad = TMC5160_PWM_GRAD,
    .pwmconf.reg.pwm_ofs = TMC5160_PWM_OFS,
    .pwm_scale.addr.reg = TMC5160Reg_PWM_SCALE,
    .lost_steps.addr.reg = TMC5160Reg_LOST_STEPS,
#ifdef TMC5160_COMPLETE
    .xdirect.addr.reg = TMC5160Reg_XDIRECT,
    .mslut[0].addr.reg = TMC5160Reg_MSLUT_BASE,
    .mslut[1].addr.reg = (tmc5160_regaddr_t)(TMC5160Reg_MSLUT_BASE + 1),
    .mslut[2].addr.reg = (tmc5160_regaddr_t)(TMC5160Reg_MSLUT_BASE + 2),
    .mslut[3].addr.reg = (tmc5160_regaddr_t)(TMC5160Reg_MSLUT_BASE + 3),
    .mslut[4].addr.reg = (tmc5160_regaddr_t)(TMC5160Reg_MSLUT_BASE + 4),
    .mslut[5].addr.reg = (tmc5160_regaddr_t)(TMC5160Reg_MSLUT_BASE + 5),
    .mslut[6].addr.reg = (tmc5160_regaddr_t)(TMC5160Reg_MSLUT_BASE + 6),
    .mslut[7].addr.reg = (tmc5160_regaddr_t)(TMC5160Reg_MSLUT_BASE + 7),
    .mslutsel.addr.reg = TMC5160Reg_MSLUTSEL,
    .mslutstart.addr.reg = TMC5160Reg_MSLUTSTART,
    .encm_ctrl.addr.reg = TMC5160Reg_ENCM_CTRL,
#endif

};

static void _set_rms_current (TMC5160_t *driver)
{
    const uint32_t V_fs = 325; // 0.325 * 1000
    uint_fast8_t CS = 31;
    uint32_t scaler = 0; // = 256

    uint16_t RS_scaled = ((float)driver->config.r_sense / 1000.f) * 0xFFFF; // Scale to 16b
    uint32_t numerator = 11585; // 32 * 256 * sqrt(2)
    numerator *= RS_scaled;
    numerator >>= 8;
    numerator *= driver->config.current;

    do {
        uint32_t denominator = V_fs * 0xFFFF >> 8;
        denominator *= CS + 1;
        scaler = numerator / denominator;
        if (scaler > 255)
            scaler = 0; // Maximum
        else if (scaler < 128)
            CS--;  // Try again with smaller CS
    } while(scaler && scaler < 128);

    driver->global_scaler.reg.scaler = scaler;
    driver->ihold_irun.reg.irun = CS;
    driver->ihold_irun.reg.ihold = (driver->ihold_irun.reg.irun * driver->config.hold_current_pct) / 100;
}

const trinamic_cfg_params_t *TMC5160_GetConfigDefaults (void)
{
    return &cfg_params;
}

void TMC5160_SetDefaults (TMC5160_t *driver)
{
    memcpy(driver, &tmc5160_defaults, sizeof(TMC5160_t));

    _set_rms_current(driver);

    driver->chopconf.reg.mres = tmc_microsteps_to_mres(driver->config.microsteps);
}

bool TMC5160_Init (TMC5160_t *driver)
{
    // Read drv_status to check if driver is online
    tmc_spi_read(driver->config.motor, (TMC_spi_datagram_t *)&driver->drv_status);
    if(driver->drv_status.reg.value == 0 || driver->drv_status.reg.value == 0xFFFFFFFF)
        return false;

    // Perform a status register read to clear reset flag
    tmc_spi_read(driver->config.motor, (TMC_spi_datagram_t *)&driver->gstat);

    driver->chopconf.reg.mres = tmc_microsteps_to_mres(driver->config.microsteps);
    tmc_spi_write(driver->config.motor, (TMC_spi_datagram_t *)&driver->gconf);
    tmc_spi_write(driver->config.motor, (TMC_spi_datagram_t *)&driver->chopconf);
    tmc_spi_write(driver->config.motor, (TMC_spi_datagram_t *)&driver->coolconf);
    tmc_spi_write(driver->config.motor, (TMC_spi_datagram_t *)&driver->pwmconf);
    tmc_spi_write(driver->config.motor, (TMC_spi_datagram_t *)&driver->tpowerdown);
    tmc_spi_write(driver->config.motor, (TMC_spi_datagram_t *)&driver->tpwmthrs);

    TMC5160_SetCurrent(driver, driver->config.current, driver->config.hold_current_pct);

    //set to a conservative start value
    //TMC5160_SetConstantOffTimeChopper(driver->config.motor, 5, 24, 13, 12, true); // move to default values

    // Read back chopconf to check if driver is online
    uint32_t chopconf = driver->chopconf.reg.value;
    tmc_spi_read(driver->config.motor, (TMC_spi_datagram_t *)&driver->chopconf);

    return driver->chopconf.reg.value == chopconf;
}

static uint_fast16_t cs2rms (TMC5160_t *driver, uint8_t CS, uint8_t global_scaler)
{
    uint32_t numerator = (global_scaler ? global_scaler : 256) * (CS + 1);
    numerator *= 325;
    numerator >>= (8 + 5); // Divide by 256 and 32
    numerator *= 1000000;
    uint32_t denominator = driver->config.r_sense;
    denominator *= 1414;

    return numerator / denominator;
}

uint16_t TMC5160_GetCurrent (TMC5160_t *driver, trinamic_current_t type)
{
    uint8_t cs, global_scaler;

    switch(type) {
        case TMCCurrent_Max:
            cs = 31;
            global_scaler = 255;
            break;
        case TMCCurrent_Actual:
            cs = driver->ihold_irun.reg.irun;
            global_scaler = driver->global_scaler.reg.scaler;
            break;
        case TMCCurrent_Hold:
            cs = driver->ihold_irun.reg.ihold;
            global_scaler = driver->global_scaler.reg.scaler;
            break;
        default: // TMCCurrent_Min:
            cs = 0;
            global_scaler = 1;
            break;
    }

    return (uint16_t)cs2rms(driver, cs, global_scaler);
}

// r_sense = mOhm, Vsense = mV, current = mA (RMS)
void TMC5160_SetCurrent (TMC5160_t *driver, uint16_t mA, uint8_t hold_pct)
{
    driver->config.current = mA;
    driver->config.hold_current_pct = hold_pct;

    _set_rms_current(driver);

    tmc_spi_write(driver->config.motor, (TMC_spi_datagram_t *)&driver->global_scaler);
    tmc_spi_write(driver->config.motor, (TMC_spi_datagram_t *)&driver->ihold_irun);
}

float TMC5160_GetTPWMTHRS (TMC5160_t *driver, float steps_mm)
{
    return tmc_calc_tstep_inv(&driver->config, driver->tpwmthrs.reg.tpwmthrs, steps_mm);
}

void TMC5160_SetTPWMTHRS (TMC5160_t *driver, float mm_sec, float steps_mm) // -> pwm threshold
{
    driver->tpwmthrs.reg.tpwmthrs = tmc_calc_tstep(&driver->config, mm_sec, steps_mm);
    tmc_spi_write(driver->config.motor, (TMC_spi_datagram_t *)&driver->tpwmthrs);
}

void TMC5160_SetTHIGH (TMC5160_t *driver, float mm_sec, float steps_mm) // -> pwm threshold
{
    driver->thigh.reg.thigh = tmc_calc_tstep(&driver->config, mm_sec, steps_mm);
    tmc_spi_write(driver->config.motor, (TMC_spi_datagram_t *)&driver->thigh);
}

void TMC5160_SetTCOOLTHRS (TMC5160_t *driver, float mm_sec, float steps_mm) // -> pwm threshold
{
    driver->tcoolthrs.reg.tcoolthrs = tmc_calc_tstep(&driver->config, mm_sec, steps_mm);
    tmc_spi_write(driver->config.motor, (TMC_spi_datagram_t *)&driver->tcoolthrs);
}

// 1 - 256 in steps of 2^value is valid for TMC5160
bool TMC5160_MicrostepsIsValid (uint16_t usteps)
{
    return tmc_microsteps_validate(usteps);
}

void TMC5160_SetMicrosteps (TMC5160_t *driver, tmc5160_microsteps_t msteps)
{
    driver->chopconf.reg.mres = tmc_microsteps_to_mres(msteps);
    driver->config.microsteps = (tmc5160_microsteps_t)(1 << (8 - driver->chopconf.reg.mres));
// TODO: recalc and set hybrid threshold if enabled?
    tmc_spi_write(driver->config.motor, (TMC_spi_datagram_t *)&driver->chopconf);
}

void TMC5160_SetConstantOffTimeChopper (TMC5160_t *driver, uint8_t constant_off_time, uint8_t blank_time, uint8_t fast_decay_time, int8_t sine_wave_offset, bool use_current_comparator)
{
    //calculate the value acc to the clock cycles
    if (blank_time >= 54)
        blank_time = 3;
    else if (blank_time >= 36)
        blank_time = 2;
    else if (blank_time >= 24)
        blank_time = 1;
    else
        blank_time = 0;

    if (fast_decay_time > 15)
        fast_decay_time = 15;

    if(driver->chopconf.reg.chm)
        driver->chopconf.reg.fd3 = (fast_decay_time & 0x8) >> 3;

    driver->chopconf.reg.tbl = blank_time;
    driver->chopconf.reg.toff = constant_off_time < 2 ? 2 : (constant_off_time > 15 ? 15 : constant_off_time);
    driver->chopconf.reg.hstrt = fast_decay_time & 0x7;
    driver->chopconf.reg.hend = (sine_wave_offset < -3 ? -3 : (sine_wave_offset > 12 ? 12 : sine_wave_offset)) + 3;

    tmc_spi_write(driver->config.motor, (TMC_spi_datagram_t *)&driver->chopconf);
}

TMC5160_status_t TMC5160_WriteRegister (TMC5160_t *driver, TMC5160_datagram_t *reg)
{
    TMC5160_status_t status;

    status.value = tmc_spi_write(driver->config.motor, (TMC_spi_datagram_t *)reg);

    return status;
}

TMC5160_status_t TMC5160_ReadRegister (TMC5160_t *driver, TMC5160_datagram_t *reg)
{
    TMC5160_status_t status;

    status.value = tmc_spi_read(driver->config.motor, (TMC_spi_datagram_t *)reg);

    return status;
}

// Returns pointer to shadow register or NULL if not found
TMC5160_datagram_t *TMC5160_GetRegPtr (TMC5160_t *driver, tmc5160_regaddr_t reg)
{
    TMC5160_datagram_t *ptr = (TMC5160_datagram_t *)driver;

    while(ptr && ptr->addr.reg != reg) {
        ptr++;
        if(ptr->addr.reg == TMC5160Reg_LOST_STEPS && ptr->addr.reg != reg)
            ptr = NULL;
    }

    return ptr;
}

