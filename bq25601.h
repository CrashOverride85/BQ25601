/**
 * Copyright (c) 2025 CrashOverride85, https://github.com/CrashOverride85
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met: 
 * 
 *  * Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer. 
 *  * Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY 
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY 
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH 
 * DAMAGE. 
 */


#ifndef _BQ25601_H
#define _BQ25601_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define BQ25601_I2C_ADDRESS 0x6B


typedef int16_t (*i2c_write_byte)(uint8_t, uint8_t, uint8_t *, uint8_t);
typedef int16_t (*i2c_read_byte)(uint8_t, uint8_t, uint8_t *, uint8_t);

typedef struct 
{
    uint8_t BQ25601_i2c_address;
    i2c_write_byte  write_reg;
    i2c_read_byte   read_reg;
} BQ25601_ctx_t;


#define BQ25601_REG00 0x00
#define BQ25601_REG01 0x01
#define BQ25601_REG02 0x02
#define BQ25601_REG03 0x03
#define BQ25601_REG04 0x04
#define BQ25601_REG05 0x05
#define BQ25601_REG06 0x06
#define BQ25601_REG07 0x07
#define BQ25601_REG08 0x08
#define BQ25601_REG09 0x09
#define BQ25601_REG0A 0x0A
#define BQ25601_REG0B 0x0b
#define BQ25601_REG_MAX BQ25601_REG0B


typedef enum  // for BQ25601_REG01, WD_RST
{
    WD_RST_NORMAL   = 0x00,
    WD_RST_RESET    = 0x01
} watchdog_reset_enum;

typedef enum  // for BQ25601_REG01, SYS_Min
{
    SYS_MIN_2_6V   = 0x00,
    SYS_MIN_2_8V   = 0x01,
    SYS_MIN_3_0V   = 0x02,
    SYS_MIN_3_2V   = 0x03,
    SYS_MIN_3_4V   = 0x04,
    SYS_MIN_3_5V   = 0x05,
    SYS_MIN_3_6V   = 0x06,
    SYS_MIN_3_7V   = 0x07
} min_sys_volt_enum;

typedef enum  // for BQ25601_REG04, TOPOFF_TIMER
{
    TOPOFF_TIMER_DISABLED   = 0x00,
    TOPOFF_TIMER_15_MINUTES = 0x01, 
    TOPOFF_TIMER_30_MINUTES = 0x02,
    TOPOFF_TIMER_45_MINUTES = 0x03
} topoff_timer_enum;

typedef enum  // for BQ25601_REG04, VRECHG
{
    VRECHG_100mV    = 0x00,
    VRECHG_200mV    = 0x01
} rechg_thresh_enum;

typedef enum  // for BQ25601_REG05, WATCHDOG
{
    WATCHDOG_DISABLED   = 0x00,
    WATCHDOG_40S        = 0x01,
    WATCHDOG_80S        = 0x02,
    WATCHDOG_160S       = 0x03
} watchdog_enum;

typedef enum  // for BQ25601_REG05, CHG_TIMER
{
    CHG_TIMER_5_HRS   = 0x00,
    CHG_TIMER_10_HRS  = 0x01
} charge_time_enum;

typedef enum  // for BQ25601_REG05, TREG
{
    TREG_90C   = 0x00,
    TREG_110C  = 0x01
} thermal_reg_enum;

typedef enum  // for BQ25601_REG06, OVP
{
    OVP_5_5V    = 0x00,
    OVP_6_5V    = 0x01,
    OVP_10_5V   = 0x02,
    OVP_14V     = 0x03
} ovp_enum;

typedef enum  // for BQ25601_REG06, BOOSTV
{
    BOOSTV_4_85V    = 0x00,
    BOOSTV_5_00V    = 0x01,
    BOOSTV_5_15V    = 0x02,
    BOOSTV_5_30V    = 0x03
} boost_voltage_enum;

typedef enum  // for BQ25601_REG07, VDPM_BAT_TRACK
{
    VDPM_BAT_TRACK_DISABLE  = 0x00,
    VDPM_BAT_TRACK_200mV    = 0x01,
    VDPM_BAT_TRACK_250mV    = 0x02,
    VDPM_BAT_TRACK_300mV    = 0x03
} vdpm_bat_track_enum;

typedef enum  // for BQ25601_REG08, VBUS_STAT
{
    NO_INPUT        = 0x00, // No input
    USB_HOST_SDP    = 0x01, // USB Host SDP (500 mA) → PSEL HIGH
    USB_CDP         = 0x02, // 1.5A,
    USB_DSC         = 0x03, // 2.4A
    UNKNOWN_ADAPTER = 0x05, // 500 mA
    NON_STANDARD    = 0x06, // Non-Standard Adapter (1A/2A/2.1A/2.4A)
    OTG             = 0x07  // OTG
} vbus_status_enum;

typedef enum  // for BQ25601_REG08, CHRG_STAT
{
    CHRG_STAT_NOT_CHARGING    = 0x00, // Not Charging
    CHRG_STAT_PRE_CHARGE      = 0x01, // Pre-charge (< VBATLOWV)
    CHRG_STAT_FAST_CHARGING   = 0x02, // Fast Charging
    CHRG_STAT_CHARGE_TERM     = 0x03  // Charge Termination
} charge_status_enum;

typedef enum  // for BQ25601_REG09, CHRG_FAULT
{
    CHRG_FAULT_NORMAL      = 0x00, // Normal
    CHRG_FAULT_INPUT_FAULT = 0x01, // Input fault (VAC OVP or VBAT < VBUS < 3.8 V)
    CHRG_FAULT_THERM_SHDN  = 0x02, // Thermal shutdown
    CHRG_FAULT_TIMER_EXP   = 0x03  // Charge Safety Timer Expiration
} charge_fault_enum;

typedef enum  // for BQ25601_REG09, NTC_FAULT
{
    NTC_FAULT_NORMAL      = 0x00, //  
    NTC_FAULT_WARM        = 0x02, // buck mode only
    NTC_FAULT_COOL        = 0x03, // buck mode only
    NTC_FAULT_COLD        = 0x05, //
    NTC_FAULT_HOT         = 0X06  // 
} ntc_fault_enum;

typedef enum  // for BQ25601_REG0B, PN
{
    PN_BQ25601D     = 0x02
} pn_enum;


void BQ25601_init(BQ25601_ctx_t *dev);
void BQ25601_read_register(uint8_t reg);
void BQ25601_read_all_registers();

// REG00
bool                BQ25601_get_hiz();
void                BQ25601_set_hiz(bool value);
bool                BQ25601_get_stat_pin();  
void                BQ25601_set_stat_pin(bool value); // true: Enable STAT pin function, false: Disable STAT pin function (float pin) 
uint16_t            BQ25601_get_input_current_limit_mA();
uint16_t            BQ25601_set_input_current_limit_mA(uint16_t lim_mA);

// REG01
bool                BQ25601_get_pfm_enabled();                              // PFM_DIS true: PFM enabled
void                BQ25601_set_pfm_enabled(bool value);                    // true: enable PFM
watchdog_reset_enum BQ25601_get_watchdog_reset();                           // WD_RST
void                BQ25601_set_watchdog_reset(watchdog_reset_enum value); 
bool                BQ25601_get_otg_enabled();                              // OTG_CONFIG - true: OTG enabled
void                BQ25601_set_otg_enabled(bool value);
bool                BQ25601_get_charge_enabled();                           // CHG_CONFIG - true: Charge Enable. Charge is enabled when both CE pin is pulled low AND CHG_CONFIG bit is 1.
void                BQ25601_set_charge_enabled(bool value);
min_sys_volt_enum   BQ25601_get_min_sys_voltage();                          
void                BQ25601_set_min_sys_voltage(min_sys_volt_enum value);

// REG02
bool                BQ25601_get_boost_lim();                                // BOOST_LIM false: 0.5A, true: 1.2A
void                BQ25601_set_boost_lim(bool value);
bool                BQ25601_get_q1_full_on();                               // Q1_FULLON false: Use higher Q1 RDSON when programmed IINDPM < 700mA (better accuracy), true: Use lower Q1 RDSON always (better efficiency)
void                BQ25601_set_q1_full_on(bool value);
uint16_t            BQ25601_set_fast_charge_current_mA(uint16_t current_mA);
uint16_t            BQ25601_get_fast_charge_current_mA();

// REG03
uint16_t            BQ25601_get_pre_charge_current_mA();
uint16_t            BQ25601_set_pre_charge_current_mA(uint16_t current_mA);
uint16_t            BQ25601_get_charge_termination_current_mA();
uint16_t            BQ25601_set_charge_termination_current_mA(uint16_t current_mA);

// REG04
uint16_t            BQ25601_get_vreg_mV();
uint16_t            BQ25601_set_vreg_mV(uint16_t vreg);
topoff_timer_enum   BQ25601_get_topoff_timer();
void                BQ25601_set_topoff_timer(topoff_timer_enum value);
rechg_thresh_enum   BQ25601_get_recharge_thresh();
void                BQ25601_set_recharge_thresh(rechg_thresh_enum value);

// REG05
bool                BQ25601_get_enable_termination();               // EN_TERM      
void                BQ25601_set_enable_termination(bool value);
watchdog_enum       BQ25601_get_watchdog_time();
void                BQ25601_set_watchdog_time(watchdog_enum value);
bool                BQ25601_get_enable_timer();                     // EN_TIMER - true: Enable both fast charge and precharge timer      
void                BQ25601_set_enable_timer(bool value);
charge_time_enum    BQ25601_get_charge_timer();
void                BQ25601_set_charge_timer(charge_time_enum value);
thermal_reg_enum    BQ25601_get_thermal_reg_temp();
void                BQ25601_set_thermal_reg_temp(thermal_reg_enum value);

// REG06
ovp_enum            BQ25601_get_ovp();
void                BQ25601_set_ovp(ovp_enum value);
boost_voltage_enum  BQ25601_get_boost_voltage();
void                BQ25601_set_boost_voltage(boost_voltage_enum value);
uint16_t            BQ25601_get_absolute_vindpm_threshold_mV();
uint16_t            BQ25601_set_absolute_vindpm_threshold_mV(uint16_t mV);

// REG07
bool                BQ25601_get_in_current_lim_detection();             // IINDET_EN - false: Not in input current limit detection, true: Force input current limit detection when VBUS is present      
void                BQ25601_set_in_current_lim_detection(bool value);
bool                BQ25601_get_x2_slow_safety_timer();                 // TMR2X_EN - false: Disable, true: Safety timer slowed by 2X during input DPM (both V and I) or JEITA cool, or thermal regulation
void                BQ25601_set_x2_slow_safety_timer(bool value);
bool                BQ25601_get_batfet_disable();                       // BATFET_DIS - false: Allow Q4 turn on, true: Turn off Q4 with tBATFET_DLY delay time 
void                BQ25601_set_batfet_disable(bool value);
bool                BQ25601_get_batfet_delay();                         // BATFET_DLY - false: Turn off BATFET immediately when BATFET_DIS bit is set, true: Turn off BATFET after tBATFET_DLY (typ. 10 s) when BATFET_DIS bit is set
void                BQ25601_set_batfet_delay(bool value);
bool                BQ25601_get_batfet_reset_enable();                  // BATFET_RST_EN - false: Disable BATFET reset function, true: Enable BATFET reset function
void                BQ25601_set_batfet_reset_enable(bool value);
vdpm_bat_track_enum BQ25601_get_vdpm_bat_track();
void                BQ25601_set_vdpm_bat_track(vdpm_bat_track_enum value);

// REG08
vbus_status_enum    BQ25601_vbus_source();
charge_status_enum  BQ25601_charge_status();
bool                BQ25601_power_good();       // PG_STAT    - good input source
bool                BQ25601_thermal_status();   // THERM_STAT - true: internal junction temperature high (not realted to battery thermistor)
bool                BQ25601_vsys_reg();         // VSYS_STAT  - true: in VSYSMin regulation (BAT < VSYSMin)

// REG09
bool                BQ25601_watchdog_fault();   // WATCHDOG_FAULT - true: Watchdog timer expiration. Power on default is true, and means the charger is operating in default mode without host management, not actually a fault.
bool                BQ25601_boost_fault();      // BOOST_FAULT    - true: VBUS overloaded in OTG, or VBUS OVP, or battery is too low (any conditions that we cannot start boost function)
charge_fault_enum   BQ25601_charge_fault();     // CHRG_FAULT
bool                BQ25601_bat_fault();        // BAT_FAULT      - true: BATOVP
ntc_fault_enum      BQ25601_ntc_fault();        // NTC_FAULT 

// REG0A
bool                BQ25601_vbus_power_good();      // VBUS_GD         - true: VBUS Attached, false: Not VBUS attached
bool                BQ25601_in_vindpm();            // VINDPM_STAT     - VIN Dynamic Power Management. true: in VINDPM, false: Not in VINDPM
bool                BQ25601_in_iindpm();            // IINDPM_STAT     - input current regulation. true: in IINDPM, false: Not in IINDPM
bool                BQ25601_top_off_active();       // TOPOFF_ACTIVE   - true: Top off timer counting, false: Top off timer not counting
bool                BQ25601_input_over_voltage();   // ACOV_STAT       - true: Device is in ACOV, false: Device is NOT in ACOV,
bool                BQ25601_get_mask_vindpm_int();  // VINDPM_INT_MASK - true: Mask VINDPM INT pulse, false: Allow VINDPM INT pulse
void                BQ25601_set_mask_vindpm_int(bool value);
bool                BQ25601_get_mask_iindpm_int();  // IINDPM_INT_MASK - true: Mask IINDPM INT pulse, false: Allow IINDPM INT pulse
void                BQ25601_set_mask_iindpm_int(bool value);

// REG0B
bool                BQ25601_get_register_reset();   // REG_RST          - true: Reset to default register value and reset safety timer, false: Keep current register setting
void                BQ25601_set_register_reset(bool value);
pn_enum             BQ25601_pn(); // Always read as 0x0 despite what the datasheet suggests?!


#ifdef __cplusplus
}
#endif

#endif // _BQ25601_H
