/*
 * tmc2660.c - interface for Trinamic TMC2660 stepper driver
 *
 * v0.0.4 / 2024-11-17
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
 * https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC2660_Calculations.xlsx
 *
 */

#include <string.h>
#include <math.h>
#include "tmc2660.h"
#include "driver.h"

static const trinamic_cfg_params_t TMC2660_cfg_params = {

    .vsense[0] = 325.0f,
    .vsense[1] = 173.0f,

    .cap.drvconf = 0x1FFFF,

    .cap.coolconf.seup = 0b11,
    .cap.coolconf.sedn = 0b11,
    .cap.coolconf.semax = 0b1111,
    .cap.coolconf.semin = 0b1111,
    .cap.coolconf.seimin = 1,

    .cap.chopconf.toff = 0b1111,
    .cap.chopconf.hstrt = 0b111,
    .cap.chopconf.hend = 0b1111,
    .cap.chopconf.hdec = 0b11,
    .cap.chopconf.rndtf = 1,
    .cap.chopconf.chm = 1,
    .cap.chopconf.tbl = 0b11,

    .dflt.drvconf = TMC2660_DRVCONF,

    .dflt.coolconf.seup = TMC2660_SEUP,
    .dflt.coolconf.sedn = TMC2660_SEDN,
    .dflt.coolconf.semax = TMC2660_SEMAX,
    .dflt.coolconf.semin = TMC2660_SEMIN,
    .dflt.coolconf.seimin = TMC2660_SEIMIN,

    .dflt.chopconf.toff = TMC2660_TOFF,
    .dflt.chopconf.hstrt = TMC2660_HSTRT - 1,
    .dflt.chopconf.hend = TMC2660_HEND + 3,
    .dflt.chopconf.hdec = TMC2660_HDEC,
    .dflt.chopconf.rndtf = TMC2660_RNDTF,
    .dflt.chopconf.chm = TMC2660_CHM,
    .dflt.chopconf.tbl = TMC2660_TBL,
};

static const TMC2660_t tmc2660_defaults = {
    .config.f_clk = TMC2660_F_CLK,
    .config.mode = TMC2660_MODE,
    .config.r_sense = TMC2660_R_SENSE,
    .config.current = TMC2660_CURRENT,
    .config.hold_current_pct = TMC2660_HOLD_CURRENT_PCT,
    .config.microsteps = TMC2660_MICROSTEPS,
    .config.cfg_params = &TMC2660_cfg_params,

    // register defaults

    .drvconf.dgr.addr = TMC2660Reg_DRVCONF,
    .drvconf.dgr.payload = TMC2660_DRVCONF,

    .drvctrl.dedge = TMC2660_DEDGE,
    .drvctrl.intpol = TMC2660_INTPOL,

    .chopconf.toff = TMC2660_TOFF,
    .chopconf.chm = TMC2660_CHM,
    .chopconf.tbl = TMC2660_TBL,
    .chopconf.hstrt = TMC2660_HSTRT - 1,
    .chopconf.hend = TMC2660_HEND + 3,
    .chopconf.hdec = TMC2660_HDEC,
    .chopconf.rndtf = TMC2660_RNDTF,

    .sgcsconf.cs = TMC2660_CURRENT_SCALE,
    .sgcsconf.sfilt = TMC2660_SG_FILTER,
    .sgcsconf.sgt = TMC2660_SG_THRESH,

    .smarten.sedn = TMC2660_SEDN,
    .smarten.seimin = TMC2660_SEIMIN,
    .smarten.semax = TMC2660_SEMAX,
    .smarten.seup = TMC2660_SEUP,
    .smarten.semin = TMC2660_SEMIN
};

const trinamic_cfg_params_t *TMC2660_GetConfigDefaults (void)
{
    return &TMC2660_cfg_params;
}

static void _set_rms_current (TMC2660_t *driver)
{
    float maxv = ((float)driver->config.r_sense * (float)(32UL * driver->config.current)) * 1.41421f / 1000.0f;

    int8_t current_scaling = (int8_t)(maxv / TMC2660_cfg_params.vsense[0]) - 1;

    // If the current scaling is too low set the vsense bit and recalculate the current setting
    if ((driver->drvconf.vsense = (current_scaling < 16)))
        current_scaling = (int8_t)(maxv / TMC2660_cfg_params.vsense[1]) - 1;

    driver->sgcsconf.cs = current_scaling > 31 ? 31 : current_scaling;
}

void TMC2660_SetDefaults (TMC2660_t *driver)
{
    memcpy(driver, &tmc2660_defaults, sizeof(TMC2660_t));

    driver->drvctrl.dgr.addr = TMC2660Reg_DRVCTRL;
    driver->chopconf.dgr.addr = TMC2660Reg_CHOPCONF;
    driver->sgcsconf.dgr.addr = TMC2660Reg_SGCSCONF;
    driver->drvconf.dgr.addr = TMC2660Reg_DRVCONF;
    driver->smarten.dgr.addr = TMC2660Reg_SMARTEN;

    _set_rms_current(driver);

    driver->drvctrl.mres = tmc_microsteps_to_mres(driver->config.microsteps);
}

bool TMC2660_Init (TMC2660_t *driver)
{
    TMC2660_drvstatus_dgr_t status = TMC2660_ReadRegister(driver, NULL, TMC2660_RDSEL3);
    if(status.response.value == 0 || status.response.value == 0xFFFFFF)
        return false;

    tmc_spi20_write(driver->config.motor, (TMC_spi20_datagram_t *)&driver->drvconf);
    tmc_spi20_write(driver->config.motor, (TMC_spi20_datagram_t *)&driver->drvctrl);
    tmc_spi20_write(driver->config.motor, (TMC_spi20_datagram_t *)&driver->chopconf);
    tmc_spi20_write(driver->config.motor, (TMC_spi20_datagram_t *)&driver->smarten);
    tmc_spi20_write(driver->config.motor, (TMC_spi20_datagram_t *)&driver->sgcsconf);

    TMC2660_SetMicrosteps(driver, (tmc2660_microsteps_t)driver->config.microsteps);
    TMC2660_SetCurrent(driver, driver->config.current, driver->config.hold_current_pct);

    return true;
}

uint16_t TMC2660_GetCurrent (TMC2660_t *driver, trinamic_current_t type)
{
    uint8_t cs;
    bool vsense;

    switch(type) {
        case TMCCurrent_Max:
            cs = 31;
            vsense = 0;
            break;
        case TMCCurrent_Actual:
            cs = driver->sgcsconf.cs;
            vsense = driver->drvconf.vsense;
            break;
        case TMCCurrent_Hold:
            cs = 0; // ?? return actual
            vsense = 0;
            break;
        default: // TMCCurrent_Min:
            cs = 0;
            vsense = 0;
            break;
    }

    return (uint16_t)ceilf((float)(cs + 1) / 32.0f * TMC2660_cfg_params.vsense[vsense] / (float)driver->config.r_sense / 1.41421f * 1000.0f);
}

// r_sense = mOhm, Vsense = mV, current = mA (RMS)
void TMC2660_SetCurrent (TMC2660_t *driver, uint16_t mA, uint8_t hold_pct)
{
    driver->config.current = mA;
    driver->config.hold_current_pct = hold_pct;

    _set_rms_current(driver);

    tmc_spi20_write(driver->config.motor, (TMC_spi20_datagram_t *)&driver->drvconf);
    tmc_spi20_write(driver->config.motor, (TMC_spi20_datagram_t *)&driver->sgcsconf);
}

// 1 - 256 in steps of 2^value is valid for TMC2660
bool TMC2660_MicrostepsIsValid (uint16_t usteps)
{
    return tmc_microsteps_validate(usteps);
}

void TMC2660_SetMicrosteps (TMC2660_t *driver, tmc2660_microsteps_t usteps)
{
    driver->drvctrl.mres = tmc_microsteps_to_mres(usteps);
    driver->config.microsteps = (tmc2660_microsteps_t)(1 << (8 - driver->drvctrl.mres));

    tmc_spi20_write(driver->config.motor, (TMC_spi20_datagram_t *)&driver->drvctrl);
}

void TMC2660_SetConstantOffTimeChopper (TMC2660_t *driver, uint8_t constant_off_time, uint8_t blank_time, uint8_t fast_decay_time, int8_t sine_wave_offset, bool use_current_comparator)
{
    
    #if 0
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

    //if(driver->chopconf.chm)
    //    driver->chopconf.fd3 = (fast_decay_time & 0x8) >> 3;

    driver->chopconf.tbl = blank_time;
    driver->chopconf.toff = constant_off_time < 2 ? 2 : (constant_off_time > 15 ? 15 : constant_off_time);
    driver->chopconf.hstrt = fast_decay_time & 0x7;
    driver->chopconf.hend = (sine_wave_offset < -3 ? -3 : (sine_wave_offset > 12 ? 12 : sine_wave_offset)) + 3;

    tmc_spi20_write(driver->config.motor, (TMC_spi20_datagram_t *)&driver->chopconf);
    #endif
}

TMC2660_drvstatus_dgr_t TMC2660_WriteRegister (TMC2660_t *driver, TMC2660_datagram_t *reg)
{
    TMC2660_drvstatus_t status;

    if(reg->addr == TMC2660Reg_DRVCONF) // do not overwrite vsense bit!
        ((TMC2660_drvconf_reg_t *)reg)->vsense = driver->drvconf.vsense;

    status.response.value = tmc_spi20_write(driver->config.motor, (TMC_spi20_datagram_t *)reg).value;

    return status;
}

// TMC2660 does not support register reads.  Just return the shadow register value and driver status?
TMC2660_drvstatus_dgr_t TMC2660_ReadRegister (TMC2660_t *driver, TMC2660_datagram_t *reg, tmc2660_rdsel_t rdsel)
{
    TMC2660_drvstatus_dgr_t status = {0};

    if(reg == NULL) {

        if(driver->drvconf.rdsel != rdsel) {

            TMC2660_drvconf_reg_t drvconf;

            drvconf.dgr.value = driver->drvconf.dgr.value;
            drvconf.rdsel = rdsel;

            tmc_spi20_write(driver->config.motor, (TMC_spi20_datagram_t *)&drvconf);
        }

        status.response.value = tmc_spi20_write(driver->config.motor, (TMC_spi20_datagram_t *)&driver->drvconf).value;
    }

    return status;
}

// Returns pointer to shadow register or NULL if not found
TMC2660_datagram_t *TMC2660_GetRegPtr (TMC2660_t *driver, tmc2660_regaddr_t reg)
{
    TMC2660_datagram_t *ptr = (TMC2660_datagram_t *)driver;

    while(ptr && ptr->addr != reg) {
        if(ptr->addr == TMC2660Reg_DRVCONF) {
            if(ptr->addr == reg)
                break;
            ptr = NULL;
        }
        ptr++;
    }

    return ptr;
}
