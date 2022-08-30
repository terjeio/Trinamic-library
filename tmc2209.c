/*
 * tmc2209.c - interface for Trinamic TMC2209 stepper driver
 *
 * v0.0.5 / 2022-08-24 / (c) Io Engineering / Terje
 */

/*

Copyright (c) 2020-2022, Terje Io
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

/*
 * Reference for calculations:
 * https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC5130_TMC2209_TMC2100_Calculations.xlsx
 *
 */

#include <string.h>

#include "tmc2209.h"


static const TMC2209_t tmc2209_defaults = {
    .config.f_clk = TMC2209_F_CLK,
    .config.mode = TMC2209_MODE,
    .config.r_sense = TMC2209_R_SENSE,
    .config.current = TMC2209_CURRENT,
    .config.hold_current_pct = TMC2209_HOLD_CURRENT_PCT,
    .config.microsteps = TMC2209_MICROSTEPS,

    // register adresses
    .gconf.addr.reg = TMC2209Reg_GCONF,
    .gconf.reg.en_spreadcycle = TMC2209_SPREADCYCLE,
    .gstat.addr.reg = TMC2209Reg_GSTAT,
    .ifcnt.addr.reg = TMC2209Reg_IFCNT,
    .slaveconf.addr.reg = TMC2209Reg_SLAVECONF,
    .otp_prog.addr.reg = TMC2209Reg_OTP_PROG,
    .otp_read.addr.reg = TMC2209Reg_OTP_READ,
    .ioin.addr.reg = TMC2209Reg_IOIN,
    .factory_conf.addr.reg = TMC2209Reg_FACTORY_CONF,
    .ihold_irun.addr.reg = TMC2209Reg_IHOLD_IRUN,
    .ihold_irun.reg.iholddelay = TMC2209_IHOLDDELAY,
    .tpowerdown.addr.reg = TMC2209Reg_TPOWERDOWN,
    .tpowerdown.reg.tpowerdown = TMC2209_TPOWERDOWN,
    .tstep.addr.reg = TMC2209Reg_TSTEP,
    .tpwmthrs.addr.reg = TMC2209Reg_TPWMTHRS,
    .vactual.addr.reg = TMC2209Reg_VACTUAL,
    .tcoolthrs.addr.reg = TMC2209Reg_TCOOLTHRS,
    .tcoolthrs.reg.tcoolthrs = TMC2209_COOLSTEP_THRS,
    .sgthrs.addr.reg = TMC2209Reg_SGTHRS,
    .sg_result.addr.reg = TMC2209Reg_SG_RESULT,
    .coolconf.addr.reg = TMC2209Reg_COOLCONF,
    .coolconf.reg.semin = TMC2209_SEMIN,
    .coolconf.reg.seup = TMC2209_SEUP,
    .coolconf.reg.semax = TMC2209_SEMAX,
    .coolconf.reg.sedn = TMC2209_SEDN,
    .coolconf.reg.seimin = TMC2209_SEIMIN,
    .mscnt.addr.reg = TMC2209Reg_MSCNT,
    .mscuract.addr.reg = TMC2209Reg_MSCURACT,
    .chopconf.addr.reg = TMC2209Reg_CHOPCONF,
    .chopconf.reg.tbl = TMC2209_TBL,
    .chopconf.reg.toff = TMC2209_TOFF,          // 0 = driver disable, 1 - with TBL >= 2 only, 2...15
    .chopconf.reg.hstrt = TMC2209_HSTRT - 1,    // 0...7 -> 1...8
    .chopconf.reg.hend = TMC2209_HEND + 3,      // 0...15 -> -3...12
    .chopconf.reg.intpol = TMC2209_INTPOL,
    .drv_status.addr.reg = TMC2209Reg_DRV_STATUS,
    .pwmconf.addr.reg = TMC2209Reg_PWMCONF,
    .pwmconf.reg.pwm_lim = TMC2209_PWM_LIM,
    .pwmconf.reg.pwm_reg = TMC2209_PWM_REG,
    .pwmconf.reg.pwm_autograd = TMC2209_PWM_AUTOGRAD,
    .pwmconf.reg.pwm_freq = TMC2209_PWM_FREQ,
    .pwmconf.reg.pwm_grad = TMC2209_PWM_GRAD,
    .pwmconf.reg.pwm_ofs = TMC2209_PWM_OFS,
    .pwmconf.reg.pwm_autoscale = TMC2209_PWM_AUTOSCALE,
    .pwm_scale.addr.reg = TMC2209Reg_PWM_SCALE,
    .pwm_auto.addr.reg = TMC2209Reg_PWM_AUTO
};

static void _set_rms_current (TMC2209_t *driver)
{
    float maxv = (((float)(driver->config.r_sense + 20)) * (float)(32UL * driver->config.current)) * 1.41421f / 1000.0f;

    uint8_t current_scaling = (uint8_t)(maxv / 325.0f) - 1;

    // If the current scaling is too low set the vsense bit and recalculate the current setting
    if ((driver->chopconf.reg.vsense = (current_scaling < 16)))
        current_scaling = (uint8_t)(maxv / 180.0f) - 1;

    driver->ihold_irun.reg.irun = current_scaling > 31 ? 31 : current_scaling;
    driver->ihold_irun.reg.ihold = (driver->ihold_irun.reg.irun * driver->config.hold_current_pct) / 100;

//?    driver->coolconf.reg.seimin = driver->ihold_irun.reg.irun >= 20;
}

void TMC2209_SetDefaults (TMC2209_t *driver)
{
    memcpy(driver, &tmc2209_defaults, sizeof(TMC2209_t));

    _set_rms_current(driver);

    driver->chopconf.reg.mres = tmc_microsteps_to_mres(driver->config.microsteps);
}

bool TMC2209_Init (TMC2209_t *driver)
{
    // Perform a status register read/write to clear status flags.
    // If no or bad response from driver return with error.
    if(!TMC2209_ReadRegister(driver, (TMC2209_datagram_t *)&driver->gstat))
        return false;

    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->gstat);

    TMC2209_ReadRegister(driver, (TMC2209_datagram_t *)&driver->gconf);
    driver->gconf.reg.pdn_disable = 1;
    driver->gconf.reg.mstep_reg_select = 1;

// Use default settings (from OTP) for these:
//  driver->gconf.reg.I_scale_analog = 1;
//  driver->gconf.reg.internal_Rsense = 0;
//  driver->gconf.reg.en_spreadcycle = 0;
//  driver->gconf.reg.multistep_filt = 1;

    TMC2209_ReadRegister(driver, (TMC2209_datagram_t *)&driver->ifcnt);

    uint8_t ifcnt = driver->ifcnt.reg.count;

    driver->chopconf.reg.mres = tmc_microsteps_to_mres(driver->config.microsteps);

    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->gconf);
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->tpowerdown);
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->pwmconf);
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->tpwmthrs);
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->tcoolthrs);
    TMC2209_SetCurrent(driver, driver->config.current, driver->config.hold_current_pct);

    TMC2209_ReadRegister(driver, (TMC2209_datagram_t *)&driver->ifcnt);

    return (((uint8_t)driver->ifcnt.reg.count - ifcnt) & 0xFF) == 7;
}

uint16_t TMC2209_GetCurrent (TMC2209_t *driver)
{
    return (uint16_t)((float)(driver->ihold_irun.reg.irun + 1) / 32.0f * (driver->chopconf.reg.vsense ? 180.0f : 325.0f) / (float)(driver->config.r_sense + 20) / 1.41421f * 1000.0f);
}

// r_sense = mOhm, Vsense = mV, current = mA (RMS)
void TMC2209_SetCurrent (TMC2209_t *driver, uint16_t mA, uint8_t hold_pct)
{
    driver->config.current = mA;
    driver->config.hold_current_pct = hold_pct;

    _set_rms_current(driver);

    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->chopconf);
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->ihold_irun);
}

float TMC2209_GetTPWMTHRS (TMC2209_t *driver, float steps_mm)
{
    return tmc_calc_tstep_inv(&driver->config, driver->tpwmthrs.reg.tpwmthrs, steps_mm);
}

void TMC2209_SetTPWMTHRS (TMC2209_t *driver, float mm_sec, float steps_mm)
{
    driver->tpwmthrs.reg.tpwmthrs = tmc_calc_tstep(&driver->config, mm_sec, steps_mm);
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->tpwmthrs);
}

void TMC2209_SetTCOOLTHRS (TMC2209_t *driver, float mm_sec, float steps_mm) // -> pwm threshold
{
    driver->tcoolthrs.reg.tcoolthrs = tmc_calc_tstep(&driver->config, mm_sec, steps_mm);
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->tcoolthrs);
}

// 1 - 256 in steps of 2^value is valid for TMC2209
bool TMC2209_MicrostepsIsValid (uint16_t usteps)
{
    return tmc_microsteps_validate(usteps);
}

void TMC2209_SetMicrosteps (TMC2209_t *driver, tmc2209_microsteps_t msteps)
{
    driver->chopconf.reg.mres = tmc_microsteps_to_mres(msteps);
    driver->config.microsteps = (tmc2209_microsteps_t)(1 << (8 - driver->chopconf.reg.mres));
// TODO: recalc and set hybrid threshold if enabled?
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->chopconf);
}

void TMC2209_SetConstantOffTimeChopper (TMC2209_t *driver, uint8_t constant_off_time, uint8_t blank_time, uint8_t fast_decay_time, int8_t sine_wave_offset, bool use_current_comparator)
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

    driver->chopconf.reg.tbl = blank_time;
    driver->chopconf.reg.toff = constant_off_time < 2 ? 2 : (constant_off_time > 15 ? 15 : constant_off_time);
    driver->chopconf.reg.hstrt = fast_decay_time & 0x7;
    driver->chopconf.reg.hend = (sine_wave_offset < -3 ? -3 : (sine_wave_offset > 12 ? 12 : sine_wave_offset)) + 3;
//!    driver->chopconf.reg.rndtf = !use_current_comparator;

    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->chopconf);
}

static void calcCRC (uint8_t *datagram, uint8_t datagramLength)
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

static void byteswap (uint8_t data[4])
{
    uint8_t tmp;

    tmp = data[0];
    data[0] = data[3];
    data[3] = tmp;
    tmp = data[1];
    data[1] = data[2];
    data[2] = tmp;
}

bool TMC2209_WriteRegister (TMC2209_t *driver, TMC2209_datagram_t *reg)
{
    TMC_uart_write_datagram_t datagram;

    datagram.msg.sync = 0x05;
    datagram.msg.slave = driver->config.motor.address;
    datagram.msg.addr.value = reg->addr.value;
    datagram.msg.addr.write = 1;
    datagram.msg.payload.value = reg->payload.value;

    byteswap(datagram.msg.payload.data);

    calcCRC(datagram.data, sizeof(TMC_uart_write_datagram_t));

    tmc_uart_write(driver->config.motor, &datagram);

// TODO: add check for ok'ed?

    return true;
}


bool TMC2209_ReadRegister (TMC2209_t *driver, TMC2209_datagram_t *reg)
{
    bool ok = false;
    TMC_uart_read_datagram_t datagram;
    TMC_uart_write_datagram_t *res;

    datagram.msg.sync = 0x05;
    datagram.msg.slave = driver->config.motor.address;
    datagram.msg.addr.value = reg->addr.value;
    datagram.msg.addr.write = 0;
    calcCRC(datagram.data, sizeof(TMC_uart_read_datagram_t));

    res = tmc_uart_read(driver->config.motor, &datagram);

    if(res->msg.slave == 0xFF && res->msg.addr.value == datagram.msg.addr.value) {
        uint8_t crc = res->msg.crc;
        calcCRC(res->data, sizeof(TMC_uart_write_datagram_t));
        if((ok = crc == res->msg.crc)) {
            reg->payload.value = res->msg.payload.value;
            byteswap(reg->payload.data);
        }
    }

    return ok;
}

// Returns pointer to shadow register or NULL if not found
TMC2209_datagram_t *TMC2209_GetRegPtr (TMC2209_t *driver, tmc2209_regaddr_t reg)
{
    TMC2209_datagram_t *ptr = (TMC2209_datagram_t *)driver;

    while(ptr && ptr->addr.reg != reg) {
        ptr++;
        if(ptr->addr.reg == TMC2209Reg_LAST_ADDR && ptr->addr.reg != reg)
            ptr = NULL;
    }

    return ptr;
}

