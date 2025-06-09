/*
 * tmc2240.c - interface for Trinamic tmc2240 stepper driver
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

/*
 * Reference for calculations:
 * https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/tmc2240_Calculations.xlsx
 *
 */

#include <math.h>
#include <string.h>

#include "tmc2240.h"

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

    .dflt.coolconf.seup = TMC2240_SEUP,
    .dflt.coolconf.sedn = TMC2240_SEDN,
    .dflt.coolconf.semax = TMC2240_SEMAX,
    .dflt.coolconf.semin = TMC2240_SEMIN,
    .dflt.coolconf.seimin = TMC2240_SEIMIN,

    .dflt.chopconf.toff = TMC2240_TOFF,
    .dflt.chopconf.hstrt = TMC2240_HSTRT - 1,
    .dflt.chopconf.hend = TMC2240_HEND + 3,
    .dflt.chopconf.intpol = TMC2240_INTPOL,
    .dflt.chopconf.tfd = TMC2240_TFD,
    .dflt.chopconf.tbl = TMC2240_TBL
};

static const TMC2240_t tmc2240_defaults = {
    .config.f_clk = TMC2240_F_CLK,
    .config.mode = TMC2240_MODE,
    .config.r_sense = TMC2240_R_REF,
    .config.current = TMC2240_CURRENT,
    .config.hold_current_pct = TMC2240_HOLD_CURRENT_PCT,
    .config.microsteps = TMC2240_MICROSTEPS,

    // register adresses
    .gconf.addr.reg = TMC2240Reg_GCONF,
//    .gconf.reg.en_pwm_mode = TMC2240_EN_PWM_MODE,
    .gstat.addr.reg = TMC2240Reg_GSTAT,
    .ioin.addr.reg = TMC2240Reg_IOIN,
    .ifcnt.addr.reg = TMC2240Reg_IFCNT,
    .nodeconf.addr.reg = TMC2240Reg_NODECONF,
    .drv_conf.addr.reg = TMC2240Reg_DRV_CONF,
    .drv_conf.reg.current_range = TMC2240_CURRENT_RANGE,
    .drv_conf.reg.slope_control = TMC2240_SLOPE_CONTROL,
    .global_scaler.addr.reg = TMC2240Reg_GLOBAL_SCALER,
    .ihold_irun.addr.reg = TMC2240Reg_IHOLD_IRUN,
    .ihold_irun.reg.irundelay = TMC2240_IRUNDELAY,
    .ihold_irun.reg.iholddelay = TMC2240_IHOLDDELAY,
    .tpowerdown.addr.reg = TMC2240Reg_TPOWERDOWN,
    .tpowerdown.reg.tpowerdown = TMC2240_TPOWERDOWN,
    .tstep.addr.reg = TMC2240Reg_TSTEP,
    .tpwmthrs.addr.reg = TMC2240Reg_TPWMTHRS,
    .tpwmthrs.reg.tpwmthrs = TMC2240_TPWM_THRS,
    .tcoolthrs.addr.reg = TMC2240Reg_TCOOLTHRS,
    .tcoolthrs.reg.tcoolthrs = TMC2240_COOLSTEP_THRS,
    .thigh.addr.reg = TMC2240Reg_THIGH,
    .direct_mode.addr.reg = TMC2240Reg_DIRECT_MODE,
    .encmode.addr.reg = TMC2240Reg_ENCMODE,
    .x_enc.addr.reg = TMC2240Reg_X_ENC,
    .enc_const.addr.reg = TMC2240Reg_ENC_CONST,
    .enc_status.addr.reg = TMC2240Reg_ENC_STATUS,
    .enc_latch.addr.reg = TMC2240Reg_ENC_LATCH,
    .adc_vsupply_ain.addr.reg = TMC2240Reg_ADC_VSUPPLY_AIN,
    .adc_temp.addr.reg = TMC2240Reg_ADC_TEMP,
    .otw_ov_vth.addr.reg = TMC2240Reg_OTW_OV_VTH,
    .chopconf.addr.reg = TMC2240Reg_CHOPCONF,
    .chopconf.reg.intpol = TMC2240_INTPOL,
    .chopconf.reg.toff = TMC2240_TOFF,
    .chopconf.reg.chm = TMC2240_CHM,
    .chopconf.reg.tbl = TMC2240_TBL,
    .chopconf.reg.hend_offset = TMC2240_HEND + 3,
#if TMC2240_CHM == 0
    .chopconf.reg.hend_offset = TMC2240_HSTRT - 1,
#else
    .chopconf.reg.fd3 = (TMC2240_TFD & 0x08) >> 3,
    .chopconf.reg.hstrt_tfd = TMC2240_TFD & 0x07,
#endif
    .coolconf.addr.reg = TMC2240Reg_COOLCONF,
    .coolconf.reg.semin = TMC2240_SEMIN,
    .coolconf.reg.seup = TMC2240_SEUP,
    .coolconf.reg.semax = TMC2240_SEMAX,
    .coolconf.reg.sedn = TMC2240_SEDN,
    .coolconf.reg.seimin = TMC2240_SEIMIN,
    .drv_status.addr.reg = TMC2240Reg_DRV_STATUS,
    .pwmconf.addr.reg = TMC2240Reg_PWMCONF,
    .pwmconf.reg.pwm_autoscale = TMC2240_PWM_AUTOSCALE,
    .pwmconf.reg.pwm_lim = TMC2240_PWM_LIM,
    .pwmconf.reg.pwm_reg = TMC2240_PWM_REG,
    .pwmconf.reg.pwm_autoscale =TMC2240_PWM_AUTOSCALE,
    .pwmconf.reg.pwm_autograd = TMC2240_PWM_AUTOGRAD,
    .pwmconf.reg.pwm_freq = TMC2240_PWM_FREQ,
    .pwmconf.reg.pwm_grad = TMC2240_PWM_GRAD,
    .pwmconf.reg.pwm_ofs = TMC2240_PWM_OFS,
    .pwm_scale.addr.reg = TMC2240Reg_PWM_SCALE,
    .pwm_auto.addr.reg = TMC2240Reg_PWM_AUTO,
    .sg4_thrs.addr.reg = TMC2240Reg_SG4_THRS,
    .sg4_result.addr.reg = TMC2240Reg_SG4_RESULT,
    .sg4_ind.addr.reg = TMC2240Reg_SG4_IND,
#ifdef TMC2240_COMPLETE
    .mslut[0].addr.reg = TMC2240Reg_MSLUT_BASE,
    .mslut[1].addr.reg = (TMC2240_regaddr_t)(TMC2240Reg_MSLUT_BASE + 1),
    .mslut[2].addr.reg = (TMC2240_regaddr_t)(TMC2240Reg_MSLUT_BASE + 2),
    .mslut[3].addr.reg = (TMC2240_regaddr_t)(TMC2240Reg_MSLUT_BASE + 3),
    .mslut[4].addr.reg = (TMC2240_regaddr_t)(TMC2240Reg_MSLUT_BASE + 4),
    .mslut[5].addr.reg = (TMC2240_regaddr_t)(TMC2240Reg_MSLUT_BASE + 5),
    .mslut[6].addr.reg = (TMC2240_regaddr_t)(TMC2240Reg_MSLUT_BASE + 6),
    .mslut[7].addr.reg = (TMC2240_regaddr_t)(TMC2240Reg_MSLUT_BASE + 7),
    .mslutsel.addr.reg = TMC2240Reg_MSLUTSEL,
    .mslutstart.addr.reg = TMC2240Reg_MSLUTSTART,
    .mscnt.addr.reg = TMC2240Reg_MSCNT,
    .mscuract.addr.reg = TMC2240Reg_MSCURACT,
#endif

};

static void _set_rms_current (TMC2240_t *driver)
{
    uint_fast16_t CS = 31, scaler;
    float K_ifs = driver->drv_conf.reg.current_range >= 2 ? 36000.0f : (driver->drv_conf.reg.current_range == 0 ? 11750.0f : 24000.0f),
          x = (float)driver->config.current * (32.0f * 256.0f) / (K_ifs / (float)driver->config.r_sense * 0.7071f);

    do {
        scaler = (uint_fast16_t)roundf(x / (float)(CS + 1));
        if(scaler > 255)
            scaler = 0; // Maximum
        else if(scaler < 128)
            CS--;  // Try again with smaller CS
    } while(scaler && scaler < 128);

    // scaler: 1...31 not allowed

    driver->global_scaler.reg.globalscaler = scaler;
    driver->ihold_irun.reg.irun = CS;
    driver->ihold_irun.reg.ihold = (driver->ihold_irun.reg.irun * driver->config.hold_current_pct) / 100;
}

const trinamic_cfg_params_t *TMC2240_GetConfigDefaults (void)
{
    return &cfg_params;
}

void TMC2240_SetDefaults (TMC2240_t *driver)
{
    memcpy(driver, &tmc2240_defaults, sizeof(TMC2240_t));

    _set_rms_current(driver);

    driver->chopconf.reg.mres = tmc_microsteps_to_mres(driver->config.microsteps);
}

#ifdef TMC_UART

bool TMC2240_Init (TMC2240_t *driver)
{
    // Perform a status register read/write to clear status flags.
    // If no or bad response from driver return with error.
    if(!tmc2240_read(driver, gstat))
        return false;

    tmc2240_write(driver, gstat);

    tmc2240_read(driver, gconf);
    tmc2240_read(driver, ifcnt);

    uint8_t ifcnt = driver->ifcnt.reg.count;

    driver->chopconf.reg.mres = tmc_microsteps_to_mres(driver->config.microsteps);

    tmc2240_write(driver, gconf);
    tmc2240_write(driver, drv_conf);
    tmc2240_write(driver, tpowerdown);
    tmc2240_write(driver, pwmconf);
    tmc2240_write(driver, tpwmthrs);
    tmc2240_write(driver, tcoolthrs);
    TMC2240_SetCurrent(driver, driver->config.current, driver->config.hold_current_pct);

    tmc2240_read(driver, ifcnt);

    return (((uint8_t)driver->ifcnt.reg.count - ifcnt) & 0xFF) == 8;
}

#else

bool TMC2240_Init (TMC2240_t *driver)
{
    // Read drv_status to check if driver is online
    tmc2240_read(driver, drv_status);
    if(driver->drv_status.reg.value == 0 || driver->drv_status.reg.value == 0xFFFFFFFF)
        return false;

    // Perform a status register read to clear reset flag
    tmc2240_read(driver, gstat);

    driver->chopconf.reg.mres = tmc_microsteps_to_mres(driver->config.microsteps);
    tmc2240_write(driver, gconf);
    tmc2240_write(driver, drv_conf);
    tmc2240_write(driver, chopconf);
    tmc2240_write(driver, coolconf);
    tmc2240_write(driver, pwmconf);
    tmc2240_write(driver, tpowerdown);
    tmc2240_write(driver, tpwmthrs);

    TMC2240_SetCurrent(driver, driver->config.current, driver->config.hold_current_pct);

    //set to a conservative start value
    //TMC2240_SetConstantOffTimeChopper(driver->config.motor, 5, 24, 13, 12, true); // move to default values

    // Read back chopconf to check if driver is online
    uint32_t chopconf = driver->chopconf.reg.value;
    tmc2240_read(driver, chopconf);

    return driver->chopconf.reg.value == chopconf;
}

#endif

static uint_fast16_t cs2rms (TMC2240_t *driver, uint8_t CS, uint8_t scaler)
{
    float K_ifs = driver->drv_conf.reg.current_range >= 2 ? 36000.0f : (driver->drv_conf.reg.current_range == 0 ? 11750.0f : 24000.0f);

    return (uint_fast16_t)((K_ifs / (float)driver->config.r_sense * 0.7071f) * ((float)(CS + 1) * (scaler ? (float)scaler : 256.0f)) / (32.0f * 256.0f));
}

uint16_t TMC2240_GetCurrent (TMC2240_t *driver, trinamic_current_t type)
{
    uint8_t cs, global_scaler;

    switch(type) {
        case TMCCurrent_Max:
            cs = 31;
            global_scaler = 0;
            break;
        case TMCCurrent_Actual:
            cs = driver->ihold_irun.reg.irun;
            global_scaler = driver->global_scaler.reg.globalscaler;
            break;
        case TMCCurrent_Hold:
            cs = driver->ihold_irun.reg.ihold;
            global_scaler = driver->global_scaler.reg.globalscaler;
            break;
        default: // TMCCurrent_Min:
            cs = 0;
            global_scaler = 32;
            break;
    }

    return (uint16_t)cs2rms(driver, cs, global_scaler);
}

// r_sense = mOhm, Vsense = mV, current = mA (RMS)
void TMC2240_SetCurrent (TMC2240_t *driver, uint16_t mA, uint8_t hold_pct)
{
    driver->config.current = mA;
    driver->config.hold_current_pct = hold_pct;

    _set_rms_current(driver);

    tmc2240_write(driver, global_scaler);
    tmc2240_write(driver, ihold_irun);
}

float TMC2240_GetTPWMTHRS (TMC2240_t *driver, float steps_mm)
{
    return tmc_calc_tstep_inv(&driver->config, driver->tpwmthrs.reg.tpwmthrs, steps_mm);
}

void TMC2240_SetTPWMTHRS (TMC2240_t *driver, float mm_sec, float steps_mm) // -> pwm threshold
{
    driver->tpwmthrs.reg.tpwmthrs = tmc_calc_tstep(&driver->config, mm_sec, steps_mm);
    tmc2240_write(driver, tpwmthrs);
}

void TMC2240_SetTHIGH (TMC2240_t *driver, float mm_sec, float steps_mm) // -> pwm threshold
{
    driver->thigh.reg.thigh = tmc_calc_tstep(&driver->config, mm_sec, steps_mm);
    tmc2240_write(driver, thigh);
}

void TMC2240_SetTCOOLTHRS (TMC2240_t *driver, float mm_sec, float steps_mm) // -> pwm threshold
{
    driver->tcoolthrs.reg.tcoolthrs = tmc_calc_tstep(&driver->config, mm_sec, steps_mm);
    tmc2240_write(driver, tcoolthrs);
}

// 1 - 256 in steps of 2^value is valid for tmc2240
bool TMC2240_MicrostepsIsValid (uint16_t usteps)
{
    return tmc_microsteps_validate(usteps);
}

void TMC2240_SetMicrosteps (TMC2240_t *driver, tmc2240_microsteps_t msteps)
{
    driver->chopconf.reg.mres = tmc_microsteps_to_mres(msteps);
    driver->config.microsteps = (tmc2240_microsteps_t)(1 << (8 - driver->chopconf.reg.mres));
// TODO: recalc and set hybrid threshold if enabled?
    tmc2240_write(driver, chopconf);
}

void TMC2240_SetConstantOffTimeChopper (TMC2240_t *driver, uint8_t constant_off_time, uint8_t blank_time, uint8_t fast_decay_time, int8_t sine_wave_offset, bool use_current_comparator)
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
    driver->chopconf.reg.hstrt_tfd = fast_decay_time & 0x7;
    driver->chopconf.reg.hend_offset = (sine_wave_offset < -3 ? -3 : (sine_wave_offset > 12 ? 12 : sine_wave_offset)) + 3;

    tmc2240_write(driver, chopconf);
}


// Returns pointer to shadow register or NULL if not found
TMC2240_datagram_t *TMC2240_GetRegPtr (TMC2240_t *driver, tmc2240_regaddr_t reg)
{
    TMC2240_datagram_t *ptr = (TMC2240_datagram_t *)driver;

    while(ptr && ptr->addr.reg != reg) {
        ptr++;
        if(ptr->addr.reg == TMC2240Reg_SG4_IND && ptr->addr.reg != reg)
            ptr = NULL;
    }

    return ptr;
}

#ifdef TMC_UART

bool TMC2240_WriteRegister (TMC2240_t *driver, TMC2240_datagram_t *reg)
{
    TMC_uart_write_datagram_t datagram;

    datagram.msg.sync = 0x05;
    datagram.msg.slave = driver->config.motor.address;
    datagram.msg.addr.value = reg->addr.value;
    datagram.msg.addr.write = 1;
    datagram.msg.payload.value = reg->payload.value;

    tmc_byteswap(datagram.msg.payload.data);

    tmc_crc8(datagram.data, sizeof(TMC_uart_write_datagram_t));

    tmc_uart_write(driver->config.motor, &datagram);

// TODO: add check for ok'ed?

    return true;
}

bool TMC2240_ReadRegister (TMC2240_t *driver, TMC2240_datagram_t *reg)
{
    bool ok = false;
    TMC_uart_read_datagram_t datagram;
    TMC_uart_write_datagram_t *res;

    datagram.msg.sync = 0x05;
    datagram.msg.slave = driver->config.motor.address;
    datagram.msg.addr.value = reg->addr.value;
    datagram.msg.addr.write = 0;
    tmc_crc8(datagram.data, sizeof(TMC_uart_read_datagram_t));

    res = tmc_uart_read(driver->config.motor, &datagram);

    if(res->msg.slave == 0xFF && res->msg.addr.value == datagram.msg.addr.value) {
        uint8_t crc = res->msg.crc;
        tmc_crc8(res->data, sizeof(TMC_uart_write_datagram_t));
        if((ok = crc == res->msg.crc)) {
            reg->payload.value = res->msg.payload.value;
            tmc_byteswap(reg->payload.data);
        }
    }

    return ok;
}

#else

TMC2240_status_t TMC2240_WriteRegister (TMC2240_t *driver, TMC2240_datagram_t *reg)
{
    TMC2240_status_t status;

    status.value = tmc_spi_write(driver->config.motor, (TMC_spi_datagram_t *)reg);

    return status;
}

TMC2240_status_t TMC2240_ReadRegister (TMC2240_t *driver, TMC2240_datagram_t *reg)
{
    TMC2240_status_t status;

    status.value = tmc_spi_read(driver->config.motor, (TMC_spi_datagram_t *)reg);

    return status;
}

#endif
