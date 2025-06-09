/*
 * tmc2240.h - register and message (datagram) descriptors for Trinamic TMC2240 stepper driver
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

#ifndef _TRINAMIC2240_H_
#define _TRINAMIC2240_H_

#include "common.h"

//#define TMC2240_COMPLETE // comment out for minimum set of registers

#pragma pack(push, 1)

typedef enum {
    TMC2240_Microsteps_1 = 1,
    TMC2240_Microsteps_2 = 2,
    TMC2240_Microsteps_4 = 4,
    TMC2240_Microsteps_8 = 8,
    TMC2240_Microsteps_16 = 16,
    TMC2240_Microsteps_32 = 32,
    TMC2240_Microsteps_64 = 64,
    TMC2240_Microsteps_128 = 128,
    TMC2240_Microsteps_256 = 256
} tmc2240_microsteps_t;

// default values

// General
#define TMC2240_F_CLK               12500000UL          // factory tuned to 12.5MHz - see datasheet for calibration procedure if required
#ifndef TMC2240_MODE
#define TMC2240_MODE                TMCMode_StealthChop // 0 = TMCMode_StealthChop, 1 = TMCMode_CoolStep, 3 = TMCMode_StallGuard
#endif
#ifndef TMC2240_MICROSTEPS
#define TMC2240_MICROSTEPS          TMC2240_Microsteps_4
#endif
#ifndef TMC2240_R_REF
#define TMC2240_R_REF               12  // Kohm
#endif
#ifndef TMC2240_CURRENT
#define TMC2240_CURRENT             500 // mA RMS
#endif
#ifndef TMC2240_HOLD_CURRENT_PCT
#define TMC2240_HOLD_CURRENT_PCT    50
#endif

// DRV_CONF
#ifndef TMC2240_CURRENT_RANGE
#define TMC2240_CURRENT_RANGE       3   // 0 - 1A, 1 - 2A, 2 - 3A, 3 - 3A
#endif
#ifndef TMC2240_SLOPE_CONTROL
#define TMC2240_SLOPE_CONTROL       0   // 0 - 100V/us, 1 - 200V/us, 2 - 400V/us, 3 - 800V/us
#endif

// CHOPCONF
#ifndef TMC2240_INTPOL
#define TMC2240_INTPOL              1   // Step interpolation: 0 = off, 1 = on
#endif
#ifndef TMC2240_TOFF
#define TMC2240_TOFF                5   // Off time: 1 - 15, 0 = MOSFET disable (8)
#endif
#ifndef TMC2240_TBL
#define TMC2240_TBL                 1   // Blanking time: 0 = 16, 1 = 24, 2 = 36, 3 = 54 clocks
#endif
#ifndef TMC2240_CHM
#define TMC2240_CHM                 0   // Chopper mode: 0 = spreadCycle, 1 = constant off time
#endif

// TMC2240_CHM 0 defaults
#ifndef TMC2240_HSTRT
#define TMC2240_HSTRT               3   // Hysteresis start: 1 - 8
#endif
#ifndef TMC2240_HEND
#define TMC2240_HEND                5   // Hysteresis end: -3 - 12
#endif
#ifndef TMC2240_HMAX
#define TMC2240_HMAX               16   // HSTRT + HEND
#endif

// TMC2240_CHM 1 defaults
#ifndef TMC2240_TFD
#define TMC2240_TFD                13  // fd3 & hstrt: 0 - 15
#endif

// IHOLD_IRUN
#ifndef TMC2240_IRUNDELAY
#define TMC2240_IRUNDELAY           6
#endif
#ifndef TMC2240_IHOLDDELAY
#define TMC2240_IHOLDDELAY          6
#endif

// TPOWERDOWN
#ifndef TMC2240_TPOWERDOWN
#define TMC2240_TPOWERDOWN          128 // 0 - ((2^8)-1) * 2^18 tCLK
#endif

// TPWMTHRS
#ifndef TMC2240_TPWM_THRS
#define TMC2240_TPWM_THRS           0   // tpwmthrs: 0 - 2^20 - 1 (20 bits)
#endif

// PWMCONF - StealthChop defaults
#ifndef TMC2240_PWM_FREQ
#define TMC2240_PWM_FREQ            1   // 0 = 1/1024, 1 = 2/683, 2 = 2/512, 3 = 2/410 fCLK
#endif
#ifndef TMC2240_PWM_AUTOGRAD
#define TMC2240_PWM_AUTOGRAD        1   // boolean (0 or 1)
#endif
#ifndef TMC2240_PWM_GRAD
#define TMC2240_PWM_GRAD            14  // 0 - 255
#endif
#ifndef TMC2240_PWM_LIM
#define TMC2240_PWM_LIM             12  // 0 - 15
#endif
#ifndef TMC2240_PWM_REG
#define TMC2240_PWM_REG             8   // 1 - 15
#endif
#ifndef TMC2240_PWM_OFS
#define TMC2240_PWM_OFS             36  // 0 - 255
#endif

// TCOOLTHRS
#ifndef TMC2240_COOLSTEP_THRS
#define TMC2240_COOLSTEP_THRS       TMC_THRESHOLD_MIN   // tpwmthrs: 0 - 2^20 - 1 (20 bits)
#endif

// COOLCONF - CoolStep defaults
#ifndef TMC2240_SEMIN
#define TMC2240_SEMIN               5   // 0 = coolStep off, 1 - 15 = coolStep on
#endif
#ifndef TMC2240_SEUP
#define TMC2240_SEUP                0   // 0 - 3 (1 - 8)
#endif
#ifndef TMC2240_SEMAX
#define TMC2240_SEMAX               2   // 0 - 15
#endif
#ifndef TMC2240_SEDN
#define TMC2240_SEDN                1   // 0 - 3
#endif
#ifndef TMC2240_SEIMIN
#define TMC2240_SEIMIN              0   // boolean (0 or 1)
#endif

// end of default values

#if TMC2240_MODE == 0   // StealthChop
#define TMC2240_PWM_AUTOSCALE 1
#define TMC2240_EN_PWM_MODE   1
#elif TMC2240_MODE == 1 // CoolStep
#define TMC2240_PWM_AUTOSCALE 0
#define TMC2240_EN_PWM_MODE   0
#else                   //StallGuard
#define TMC2240_PWM_AUTOSCALE 0
#define TMC2240_EN_PWM_MODE   0
#endif

typedef uint8_t tmc2240_regaddr_t;

enum tmc2240_regaddr_t {
    TMC2240Reg_GCONF            = 0x00,
    TMC2240Reg_GSTAT            = 0x01,
    TMC2240Reg_IFCNT            = 0x02,
    TMC2240Reg_NODECONF         = 0x03,
    TMC2240Reg_IOIN             = 0x04,

    TMC2240Reg_DRV_CONF         = 0x0A,
    TMC2240Reg_GLOBAL_SCALER    = 0x0B,

    TMC2240Reg_IHOLD_IRUN       = 0x10,
    TMC2240Reg_TPOWERDOWN       = 0x11,
    TMC2240Reg_TSTEP            = 0x12,
    TMC2240Reg_TPWMTHRS         = 0x13,
    TMC2240Reg_TCOOLTHRS        = 0x14,
    TMC2240Reg_THIGH            = 0x15,

    TMC2240Reg_DIRECT_MODE      = 0x2D,

    TMC2240Reg_ENCMODE          = 0x38,
    TMC2240Reg_X_ENC            = 0x39,
    TMC2240Reg_ENC_CONST        = 0x3A,
    TMC2240Reg_ENC_STATUS       = 0x3B,
    TMC2240Reg_ENC_LATCH        = 0x3C,

    TMC2240Reg_ADC_VSUPPLY_AIN  = 0x50,
    TMC2240Reg_ADC_TEMP         = 0x51,
    TMC2240Reg_OTW_OV_VTH       = 0x52,

    TMC2240Reg_MSLUT_BASE       = 0x60,
    TMC2240Reg_MSLUTSEL         = 0x68,
    TMC2240Reg_MSLUTSTART       = 0x69,
    TMC2240Reg_MSCNT            = 0x6A,
    TMC2240Reg_MSCURACT         = 0x6B,
    TMC2240Reg_CHOPCONF         = 0x6C,
    TMC2240Reg_COOLCONF         = 0x6D,
    TMC2240Reg_DCCTRL           = 0x6E,
    TMC2240Reg_DRV_STATUS       = 0x6F,

    TMC2240Reg_PWMCONF          = 0x70,
    TMC2240Reg_PWM_SCALE        = 0x71,
    TMC2240Reg_PWM_AUTO         = 0x72,
    TMC2240Reg_SG4_THRS         = 0x74,
    TMC2240Reg_SG4_RESULT       = 0x75,
    TMC2240Reg_SG4_IND          = 0x76,
};

typedef union {
    uint8_t value;
    struct {
        uint8_t
        reset_flag       :1,
        driver_error     :1,
        sg2              :1,
        standstill       :1,
        reserved         :4;
    };
} TMC2240_status_t;

// --- register definitions ---

// GCONF : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        reserved_0             :1,
        fast_standstill        :1,
        en_pwm_mode            :1,
        multistep_filt         :1,
        shaft                  :1,
        diag0_error            :1,
        diag0_otpw             :1,
        diag0_stall            :1,
        diag1_stall            :1,
        diag1_index            :1,
        diag1_onstate          :1,
        reserved_1             :1,
        diag0_pushpull         :1,
        diag1_pushpull         :1,
        small_hysteresis       :1,
        stop_enable            :1,
        direct_mode            :1,
        reserved               :15;
    };
} TMC2240_gconf_reg_t;

// GSTAT : R+C
typedef union {
    uint32_t value;
    struct {
        uint32_t
        reset          :1,
        drv_err        :1,
        uv_cp          :1,
        register_reset :1,
        vm_uvlo        :1,
        reserved       :27;
    };
} TMC2240_gstat_reg_t;

// IFCNT : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        count    :8,
        reserved :24;
    };
} TMC2240_ifcnt_reg_t;

// NODECONF  : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        nodeaddr  :8,
        senddelay :4,
        reserved  :20;
    };
} TMC2240_nodeconf_reg_t;

// IOIN : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        step        :1,
        dir         :1,
        enca        :1,
        encb        :1,
        drv_enn     :1,
        encn        :1,
        uart_en     :1,
        reserved_0  :1,
        comp_a      :1,
        comp_b      :1,
        comp_a1_a2  :1,
        comp_b1_b2  :1,
        output      :1,
        ext_res_det :1,
        ext_clk     :1,
        adc_err     :1,
        silicon_rev :3,
        reserved_1  :5,
        version     :8;
    };
} TMC2240_ioin_reg_t;

// DRV_CONF : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        current_range :2,
        reserved_0    :2,
        slope_control :2,
        reserved_1    :26;
    };
} TMC2240_drv_conf_reg_t;

// GLOBAL_SCALER : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        globalscaler :8,
        reserved     :24;
    };
} TMC2240_global_scaler_reg_t;

// IHOLD_IRUN : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        ihold      :5,
        reserved_0 :3,
        irun       :5,
        reserved_1 :3,
        iholddelay :4,
        reserved_2 :4,
        irundelay  :4,
        reserved_3 :4;
    };
} TMC2240_ihold_irun_reg_t;

// TPOWERDOWN : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        tpowerdown :8,
        reserved   :24;
    };
} TMC2240_tpowerdown_reg_t;

// TSTEP : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        tstep    :20,
        reserved :12;
    };
} TMC2240_tstep_reg_t;

// TPWMTHRS : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        tpwmthrs :20,
        reserved :12;
    };
} TMC2240_tpwmthrs_reg_t;

// TCOOLTHRS : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        tcoolthrs :20,
        reserved  :12;
    };
} TMC2240_tcoolthrs_reg_t;

// THIGH : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        thigh    :20,
        reserved :12;
    };
} TMC2240_thigh_reg_t;

// DIRECT_MODE : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        direct_coil_a :9,
        reserved_0    :7,
        direct_coil_b :9,
        reserved_1    :7;
    };
} TMC2240_direct_mode_reg_t;

// ENCMODE : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        pol_a           :1,
        pol_b           :1,
        pol_n           :1,
        ignore_ab       :1,
        clr_cnt         :1,
        clr_once        :1,
        pos_neg_edge    :2,
        clr_enc_x       :1,
        reserved_0      :7,
        enc_sel_decimal :1,
        reserved_1      :21;
    };
} TMC2240_encmode_reg_t;

// X_ENC : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        position :32;
    };
} TMC2240_x_enc_reg_t;

// ENC_CONST : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        integer  :16,
        fraction :16;
    };
} TMC2240_enc_const_reg_t;

// ENC_STATUS : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        n_event  :1,
        reserved :31;
    };
} TMC2240_enc_status_reg_t;

// ENC_LATCH : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        position :32;
    };
} TMC2240_enc_latch_reg_t;

// ADC_VSUPPLY_AIN : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        adc_vsupply  :13,
        reserved_0   :3,
        adc_ain      :13,
        reserved_1   :3;
    };
} TMC2240_adc_vsupply_ain_reg_t;

// ADC_TEMP : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        adc_temp     :13,
        reserved_0   :3,
        reserved     :13,
        reserved_1   :3;
    };
} TMC2240_adc_temp_reg_t;

// OTW_OV_VTH : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        overvoltage_vth        :13,
        reserved_0             :3,
        overtempprewarning_vth :13,
        reserved_1             :3;
    };
} TMC2240_otw_ov_vth_reg_t;

// MSLUTn : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        mslut_n :32;
    };
} TMC2240_mslut_n_reg_t;

// MSLUTSEL : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        w0 :2,
        w1 :2,
        w2 :2,
        w3 :2,
        x1 :8,
        x2 :8,
        x3 :8;
    };
} TMC2240_mslutsel_reg_t;

// MSLUTSTART : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        start_sin   :8,
        reserved_0  :8,
        start_sin90 :8,
        reserved_1  :8;
    };
} TMC2240_mslutstart_reg_t;

//??MSLUTSEL

// MSCNT : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        mscnt    :10,
        reserved :22;
    };
} TMC2240_mscnt_reg_t;

// MSCURACT : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        cur_b     :9,
        reserved1 :7,
        cur_a     :9,
        reserved2 :7;
    };
} TMC2240_mscuract_reg_t;

// CHOPCONF : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        toff        :4,
        hstrt_tfd   :3,
        hend_offset :4,
        fd3         :1,
        disfdcc     :1,
        reserved_0  :1,
        chm         :1,
        tbl         :2,
        reserved_1  :1,
        vhighfs     :1,
        vhighchm    :1,
        tpfd        :4,
        mres        :4,
        intpol      :1,
        dedge       :1,
        diss2g      :1,
        diss2vs     :1;
    };
} TMC2240_chopconf_reg_t;

// COOLCONF : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        semin      :4,
        reserved_0 :1,
        seup       :2,
        reserved_1 :1,
        semax      :4,
        reserved_2 :1,
        sedn       :2,
        seimin     :1,
        sgt        :7,
        reserved_3 :1,
        sfilt      :1,
        reserved_4 :7;
    };
} TMC2240_coolconf_reg_t;

// DRV_STATUS : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        sg_result   :10,
        reserved_0  :2,
        s2vsa       :1,
        s2vsb       :1,
        stealth     :1,
        fsactive    :1,
        cs_actual   :5,
        reserved_1  :3,
        stallguard  :1,
        ot          :1,
        otpw        :1,
        s2ga        :1,
        s2gb        :1,
        ola         :1,
        olb         :1,
        stst        :1;
    };
} TMC2240_drv_status_reg_t;

// PWMCONF : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        pwm_ofs            :8,
        pwm_grad           :8,
        pwm_freq           :2,
        pwm_autoscale      :1,
        pwm_autograd       :1,
        freewheel          :2,
        pwm_meas_sd_enable :1,
        pwm_dis_reg_stst   :1,
        pwm_reg            :4,
        pwm_lim            :4;
    };
} TMC2240_pwmconf_reg_t;

// PWM_SCALE : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        pwm_scale_sum  :8,
        reserved1      :8,
        pwm_scale_auto :9,
        reserved2      :7;
    };
} TMC2240_pwm_scale_reg_t;

// PWM_AUTO : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        pwm_ofs_auto   :8,
        reserved_0     :8,
        pwm_grad_auto  :8,
        reserved_1     :8;
    };
} TMC2240_pwm_auto_reg_t;

// SG4_THRS : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        sg4_thrs        :8,
        sg4_fil_en      :1,
        sg_angle_offset :1,
        reserved        :22;
    };
} TMC2240_sg4_thrs_reg_t;

// SG4_RESULT: R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        sg4_result :10,
        reserved   :22;
    };
} TMC2240_sg4_result_reg_t;

// SG4_IND: R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        sg4_ind_0 :8,
        sg4_ind_1 :8,
        sg4_ind_2 :8,
        sg4_ind_3 :8;
    };
} TMC2240_sg4_ind_t;

// --- end of register definitions ---

typedef union {
    tmc2240_regaddr_t reg;
    uint8_t value;
    struct {
        uint8_t
        idx   :7,
        write :1;
    };
} TMC2240_addr_t;

// --- datagrams ---

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_gconf_reg_t reg;
} TMC2240_gconf_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_gstat_reg_t reg;
} TMC2240_gstat_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_ifcnt_reg_t reg;
} TMC2240_ifcnt_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_nodeconf_reg_t reg;
} TMC2240_nodeconf_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_ioin_reg_t reg;
} TMC2240_ioin_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_drv_conf_reg_t reg;
} TMC2240_drv_conf_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_global_scaler_reg_t reg;
} TMC2240_global_scaler_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_ihold_irun_reg_t reg;
} TMC2240_ihold_irun_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_tpowerdown_reg_t reg;
} TMC2240_tpowerdown_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_tcoolthrs_reg_t reg;
} TMC2240_tcoolthrs_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_tstep_reg_t reg;
} TMC2240_tstep_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_tpwmthrs_reg_t reg;
} TMC2240_tpwmthrs_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_thigh_reg_t reg;
} TMC2240_thigh_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_direct_mode_reg_t reg;
} TMC2240_direct_mode_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_encmode_reg_t reg;
} TMC2240_encmode_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_x_enc_reg_t reg;
} TMC2240_x_enc_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_enc_const_reg_t reg;
} TMC2240_enc_const_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_enc_status_reg_t reg;
} TMC2240_enc_status_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_enc_latch_reg_t reg;
} TMC2240_enc_latch_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_adc_vsupply_ain_reg_t reg;
} TMC2240_adc_vsupply_ain_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_adc_temp_reg_t reg;
} TMC2240_adc_temp_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_otw_ov_vth_reg_t reg;
} TMC2240_otw_ov_vth_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_mslut_n_reg_t reg;
} TMC2240_mslut_n_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_mslutsel_reg_t reg;
} TMC2240_mslutsel_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_mslutstart_reg_t reg;
} TMC2240_mslutstart_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_mscnt_reg_t reg;
} TMC2240_mscnt_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_mscuract_reg_t reg;
} TMC2240_mscuract_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_chopconf_reg_t reg;
} TMC2240_chopconf_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_drv_status_reg_t reg;
} TMC2240_drv_status_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_coolconf_reg_t reg;
} TMC2240_coolconf_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_pwmconf_reg_t reg;
} TMC2240_pwmconf_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_pwm_scale_reg_t reg;
} TMC2240_pwm_scale_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_pwm_auto_reg_t reg;
} TMC2240_pwm_auto_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_sg4_thrs_reg_t reg;
} TMC2240_sg4_thrs_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_sg4_result_reg_t reg;
} TMC2240_sg4_result_dgr_t;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_sg4_ind_t reg;
} TMC2240_sg4_ind_dgr_t;

// -- end of datagrams

typedef union {
    uint32_t value;
    uint8_t data[4];
    TMC2240_gconf_reg_t gconf;
    TMC2240_gstat_reg_t gstat;
    TMC2240_ifcnt_reg_t ifcnt;
    TMC2240_nodeconf_reg_t nodeconf;
    TMC2240_ioin_reg_t ioin;
    TMC2240_drv_conf_reg_t drv_conf;
    TMC2240_global_scaler_reg_t global_scaler;
    TMC2240_ihold_irun_reg_t ihold_irun;
    TMC2240_tpowerdown_reg_t tpowerdown;
    TMC2240_tstep_reg_t tstep;
    TMC2240_tpwmthrs_reg_t tpwmthrs;
    TMC2240_tcoolthrs_reg_t tcoolthrs;
    TMC2240_thigh_reg_t thigh;
    TMC2240_direct_mode_reg_t direct_mode;
    TMC2240_encmode_reg_t encmode;
    TMC2240_x_enc_reg_t x_enc;
    TMC2240_enc_const_reg_t enc_const;
    TMC2240_enc_status_reg_t enc_status;
    TMC2240_enc_latch_reg_t enc_latch;
    TMC2240_adc_vsupply_ain_reg_t adc_vsupply_ain;
    TMC2240_adc_temp_reg_t adc_temp;
    TMC2240_otw_ov_vth_reg_t otw_ov_vth;
    TMC2240_mslut_n_reg_t mslut;
    TMC2240_mslutsel_reg_t mslutsel;
    TMC2240_mslutstart_reg_t mslutstart;
    TMC2240_mscnt_reg_t mscnt;
    TMC2240_mscuract_reg_t mscuract;
    TMC2240_chopconf_reg_t chopconf;
    TMC2240_coolconf_reg_t coolconf;
    TMC2240_drv_status_reg_t drv_status;
    TMC2240_pwmconf_reg_t pwmconf;
    TMC2240_pwm_scale_reg_t pwm_scale;
    TMC2240_pwm_auto_reg_t pwm_auto;
    TMC2240_sg4_thrs_reg_t sg4_thrs;
    TMC2240_sg4_result_reg_t sg4_result;
    TMC2240_sg4_ind_t sg4_ind;
} TMC2240_payload;

typedef struct {
    TMC2240_addr_t addr;
    TMC2240_payload payload;
} TMC2240_datagram_t;

typedef struct {
    // driver registers
    TMC2240_gconf_dgr_t gconf;
    TMC2240_gstat_dgr_t gstat;
    TMC2240_ifcnt_dgr_t ifcnt;
    TMC2240_nodeconf_dgr_t nodeconf;
    TMC2240_ioin_dgr_t ioin;
    TMC2240_drv_conf_dgr_t drv_conf;
    TMC2240_global_scaler_dgr_t global_scaler;
    TMC2240_ihold_irun_dgr_t ihold_irun;
    TMC2240_tpowerdown_dgr_t tpowerdown;
    TMC2240_tstep_dgr_t tstep;
    TMC2240_tpwmthrs_dgr_t tpwmthrs;
    TMC2240_tcoolthrs_dgr_t tcoolthrs;
    TMC2240_thigh_dgr_t thigh;
    TMC2240_direct_mode_dgr_t direct_mode;
    TMC2240_encmode_dgr_t encmode;
    TMC2240_x_enc_dgr_t x_enc;
    TMC2240_enc_const_dgr_t enc_const;
    TMC2240_enc_status_dgr_t enc_status;
    TMC2240_enc_latch_dgr_t enc_latch;
    TMC2240_adc_vsupply_ain_dgr_t adc_vsupply_ain;
    TMC2240_adc_temp_dgr_t adc_temp;
    TMC2240_otw_ov_vth_dgr_t otw_ov_vth;
#ifdef TMC2240_COMPLETE
    TMC2240_mslut_n_dgr_t mslut[8];
    TMC2240_mslutsel_dgr_t mslutsel;
    TMC2240_mslutstart_dgr_t mslutstart;
    TMC2240_mscnt_dgr_t mscnt;
    TMC2240_mscuract_dgr_t mscuract;
#endif
    TMC2240_chopconf_dgr_t chopconf;
    TMC2240_coolconf_dgr_t coolconf;
    TMC2240_drv_status_dgr_t drv_status;
    TMC2240_pwmconf_dgr_t pwmconf;
    TMC2240_pwm_scale_dgr_t pwm_scale;
    TMC2240_pwm_auto_dgr_t pwm_auto;
    TMC2240_sg4_thrs_dgr_t sg4_thrs;
    TMC2240_sg4_result_dgr_t sg4_result;
    TMC2240_sg4_ind_dgr_t sg4_ind;
    TMC2240_status_t driver_status;

    trinamic_config_t config;
} TMC2240_t;

#pragma pack(pop)

bool TMC2240_Init(TMC2240_t *driver);
void TMC2240_SetDefaults (TMC2240_t *driver);
const trinamic_cfg_params_t *TMC2240_GetConfigDefaults (void);
void TMC2240_SetCurrent (TMC2240_t *driver, uint16_t mA, uint8_t hold_pct);
uint16_t TMC2240_GetCurrent (TMC2240_t *driver, trinamic_current_t type);
bool TMC2240_MicrostepsIsValid (uint16_t usteps);
void TMC2240_SetMicrosteps(TMC2240_t *driver, tmc2240_microsteps_t usteps);
float TMC2240_GetTPWMTHRS (TMC2240_t *driver, float steps_mm);
void TMC2240_SetTPWMTHRS (TMC2240_t *driver, float mm_sec, float steps_mm);
void TMC2240_SetTHIGH (TMC2240_t *driver, float mm_sec, float steps_mm);
void TMC2240_SetTCOOLTHRS (TMC2240_t *driver, float mm_sec, float steps_mm);

void TMC2240_SetConstantOffTimeChopper(TMC2240_t *driver, uint8_t constant_off_time, uint8_t blank_time, uint8_t fast_decay_time, int8_t sine_wave_offset, bool use_current_comparator);
TMC2240_datagram_t *TMC2240_GetRegPtr (TMC2240_t *driver, tmc2240_regaddr_t reg);

#ifdef TMC_UART
bool TMC2240_WriteRegister (TMC2240_t *driver, TMC2240_datagram_t *reg);
bool TMC2240_ReadRegister (TMC2240_t *driver, TMC2240_datagram_t *reg);
#define tmc2240_read(driver, reg) TMC2240_ReadRegister(driver, (TMC2240_datagram_t *)&driver->reg)
#define tmc2240_write(driver, reg) TMC2240_WriteRegister(driver, (TMC2240_datagram_t *)&driver->reg)
#else
TMC2240_status_t TMC2240_WriteRegister (TMC2240_t *driver, TMC2240_datagram_t *reg);
TMC2240_status_t TMC2240_ReadRegister (TMC2240_t *driver, TMC2240_datagram_t *reg);
#define tmc2240_read(driver, reg) tmc_spi_read(driver->config.motor, (TMC_spi_datagram_t *)&driver->reg)
#define tmc2240_write(driver, reg) tmc_spi_write(driver->config.motor, (TMC_spi_datagram_t *)&driver->reg)
#endif

#endif // _TRINAMIC2240_H_
