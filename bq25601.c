/**
 *     Driver for: BQ25601D I2C Controlled 3-A Single-Cell Battery Charger 
 *                 With USB Charger Detection for High Input Voltage and 
 *                 Narrow Voltage
 *
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

#include <stddef.h>
#include <stdio.h>
#include "bq25601.h"

static BQ25601_ctx_t ctx = {0};

static uint8_t _register[BQ25601_REG_MAX+1] = {0};

void BQ25601_init(BQ25601_ctx_t *dev)
{
    if (dev == NULL)
        return;

    ctx.read_reg = dev->read_reg;
    ctx.write_reg = dev->write_reg;
    ctx.BQ25601_i2c_address = dev->BQ25601_i2c_address;
    BQ25601_read_all_registers();
}

void BQ25601_read_all_registers()
{
    for(uint8_t reg=0; reg < BQ25601_REG_MAX; reg++)
        BQ25601_read_register(reg);
}

void BQ25601_read_register(uint8_t reg)
{
    uint8_t data[1];
    if (ctx.read_reg(ctx.BQ25601_i2c_address, reg, data, 1))
        _register[reg] = data[0];
}

void BQ25601_set_register(uint8_t reg, uint8_t value)
{
    uint8_t data[1];
    data[0] = value;
    ctx.write_reg(ctx.BQ25601_i2c_address, reg, data, 1);
}

//////////////////////////////// REG00 ////////////////////////////////

bool BQ25601_get_hiz()
{
    return (_register[BQ25601_REG00] & 0x80) != 0;
}

void BQ25601_set_hiz(bool value)
{
    BQ25601_read_register(BQ25601_REG00);

    uint8_t new_reg00 = _register[BQ25601_REG00] & ~(0x80);
    if (value)
    {
        new_reg00 |= 0x80;
    }
    BQ25601_set_register(BQ25601_REG00, new_reg00);
    BQ25601_read_register(BQ25601_REG00);
}

bool BQ25601_get_stat_pin()
{
    // EN_ICHG_MON is two bits, but only 00 (enabled) and 11 (disabled) are used

    uint8_t en_ichg_mon = (_register[BQ25601_REG00] >> 5) & 0x03;
    return (en_ichg_mon == 0);
}

void BQ25601_set_stat_pin(bool value)
{
    // EN_ICHG_MON is two bits, but only 00 (enabled) and 11 (disabled) are used
    BQ25601_read_register(BQ25601_REG00);

    uint8_t new_reg00 = _register[BQ25601_REG00] & 0x9F;
    if (!value)
    {
        new_reg00 |= 0x60;
    }
    BQ25601_set_register(BQ25601_REG00, new_reg00);
    BQ25601_read_register(BQ25601_REG00);
}

uint16_t BQ25601_get_input_current_limit_mA()
{
    const uint16_t offset = 100;
    uint16_t iindpm = (_register[BQ25601_REG00] & 0x1F);
    return (iindpm * 100) + offset;
}

uint16_t BQ25601_set_input_current_limit_mA(uint16_t lim_mA)
{
    const uint16_t offset = 100;

    if ((lim_mA < offset) || (lim_mA > 3200))
        return 0;

    lim_mA -= offset;
    uint8_t new_iindpm = lim_mA / 100;

    BQ25601_read_register(BQ25601_REG00);
    uint8_t new_reg00 = _register[BQ25601_REG00] & 0xE0;
    new_reg00 |= new_iindpm;
    BQ25601_set_register(BQ25601_REG00, new_reg00);

    BQ25601_read_register(BQ25601_REG00);
    return BQ25601_get_input_current_limit_mA();
}

//////////////////////////////// REG01 ////////////////////////////////


bool BQ25601_get_pfm_enabled()
{
    return (_register[BQ25601_REG01] & 0x80) == 0;
}

void BQ25601_set_pfm_enabled(bool value)
{
    BQ25601_read_register(BQ25601_REG01);

    uint8_t new_reg01 = _register[BQ25601_REG01] & ~(0x80);
    /* From datasheet:
    *   0 – Enable PFM
    *   1 – Disable PFM
    */
    if (!value) 
    {
        new_reg01 |= 0x80;
    }
    BQ25601_set_register(BQ25601_REG01, new_reg01);
    BQ25601_read_register(BQ25601_REG01);
}

watchdog_reset_enum BQ25601_get_watchdog_reset()
{
    if ((_register[BQ25601_REG01] & 0x40) == 0)
        return WD_RST_NORMAL;
    else
        return WD_RST_RESET;
}

void BQ25601_set_watchdog_reset(watchdog_reset_enum value)
{
    BQ25601_read_register(BQ25601_REG01);

    uint8_t new_reg01 = _register[BQ25601_REG01] & ~(0x40);
    if (value == WD_RST_RESET) 
    {
        new_reg01 |= 0x40;
    }
    BQ25601_set_register(BQ25601_REG01, new_reg01);
    BQ25601_read_register(BQ25601_REG01);
}

bool BQ25601_get_otg_enabled()
{
    return (_register[BQ25601_REG01] & 0x20) != 0;
}

void BQ25601_set_otg_enabled(bool value)
{
    BQ25601_read_register(BQ25601_REG01);

    uint8_t new_reg01 = _register[BQ25601_REG01] & ~(0x20);
    if (value) 
    {
        new_reg01 |= 0x20;
    }
    BQ25601_set_register(BQ25601_REG01, new_reg01);
    BQ25601_read_register(BQ25601_REG01);
}

bool BQ25601_get_charge_enabled()
{
    return (_register[BQ25601_REG01] & 0x10) != 0;
}

void BQ25601_set_charge_enabled(bool value)
{
    BQ25601_read_register(BQ25601_REG01);

    uint8_t new_reg01 = _register[BQ25601_REG01] & ~(0x10);
    if (value) 
    {
        new_reg01 |= 0x10;
    }
    BQ25601_set_register(BQ25601_REG01, new_reg01);
    BQ25601_read_register(BQ25601_REG01);
}

min_sys_volt_enum BQ25601_get_min_sys_voltage()
{
    return (_register[BQ25601_REG01] & 0x0E) >> 1;
}

void BQ25601_set_min_sys_voltage(min_sys_volt_enum value)
{
    BQ25601_read_register(BQ25601_REG01);

    uint8_t new_reg01 = _register[BQ25601_REG01] & ~(0x0E);

    new_reg01 |= (value << 1);
    
    BQ25601_set_register(BQ25601_REG01, new_reg01);
    BQ25601_read_register(BQ25601_REG01);
}

//////////////////////////////// REG02 ////////////////////////////////

bool BQ25601_get_boost_lim()
{
    return (_register[BQ25601_REG02] & 0x80) != 0;
}

void BQ25601_set_boost_lim(bool value)
{
    BQ25601_read_register(BQ25601_REG02);

    uint8_t new_reg02 = _register[BQ25601_REG02] & ~(0x80);
    if (value) 
    {
        new_reg02 |= 0x80;
    }
    BQ25601_set_register(BQ25601_REG02, new_reg02);
    BQ25601_read_register(BQ25601_REG02);
}

bool BQ25601_get_q1_full_on()
{
    return (_register[BQ25601_REG02] & 0x40) != 0;
}

void BQ25601_set_q1_full_on(bool value)
{
    BQ25601_read_register(BQ25601_REG02);

    uint8_t new_reg02 = _register[BQ25601_REG02] & ~(0x40);
    if (value) 
    {
        new_reg02 |= 0x40;
    }
    BQ25601_set_register(BQ25601_REG02, new_reg02);
    BQ25601_read_register(BQ25601_REG02);
}

uint16_t BQ25601_set_fast_charge_current_mA(uint16_t current_mA)
{
    if (current_mA > 3000)
        return 0;

    uint16_t ichg = current_mA / 60;

    BQ25601_read_register(BQ25601_REG02);
    uint8_t new_reg02 = _register[BQ25601_REG02] & ~(0x3F);
    new_reg02 |= ichg;
    BQ25601_set_register(BQ25601_REG02, new_reg02);

    BQ25601_read_register(BQ25601_REG02);
    return BQ25601_get_fast_charge_current_mA();
}

uint16_t BQ25601_get_fast_charge_current_mA()
{
    return (_register[BQ25601_REG02] & 0x3F) * 60;
}

//////////////////////////////// REG03 ////////////////////////////////


uint16_t BQ25601_get_pre_charge_current_mA()
{
    const uint8_t offset = 60;
    return (((_register[BQ25601_REG03] & 0xF0) >> 4) * 60) + offset;
}

uint16_t BQ25601_set_pre_charge_current_mA(uint16_t current_mA)
{
    const uint8_t offset = 60;

    if ((current_mA < 60) || (current_mA > 780))
        return 0;

    uint16_t iprechg = (current_mA - offset) / 60;

    BQ25601_read_register(BQ25601_REG03);
    uint8_t new_reg03 = _register[BQ25601_REG03] & ~(0xF0);
    new_reg03 |= (iprechg << 4);
    BQ25601_set_register(BQ25601_REG03, new_reg03);

    BQ25601_read_register(BQ25601_REG03);
    return BQ25601_get_pre_charge_current_mA();
}

uint16_t BQ25601_get_charge_termination_current_mA()
{
    const uint8_t offset = 60;
    return ((_register[BQ25601_REG03] & 0x0F) * 60) + offset;
}

uint16_t BQ25601_set_charge_termination_current_mA(uint16_t current_mA)
{
    const uint8_t offset = 60;

    if ((current_mA < 60) || (current_mA > 780))
        return 0;

    uint16_t iterm = (current_mA - offset) / 60;

    BQ25601_read_register(BQ25601_REG03);
    uint8_t new_reg03 = _register[BQ25601_REG03] & ~(0x0F);
    new_reg03 |= iterm;
    BQ25601_set_register(BQ25601_REG03, new_reg03);

    BQ25601_read_register(BQ25601_REG03);
    return BQ25601_get_charge_termination_current_mA();
}

//////////////////////////////// REG04 ////////////////////////////////

uint16_t BQ25601_get_vreg_mV_from_reg(uint8_t reg_value)
{
    uint16_t mV = 3847;

    if (reg_value & 0x80) mV += 512;
    if (reg_value & 0x40) mV += 256;
    if (reg_value & 0x20) mV += 128;
    if (reg_value & 0x10) mV += 64;
    if (reg_value & 0x08) mV += 32;

    // From datasheet: "Special Value: (01111): 4.343 V"
    if ((reg_value >> 3) == 0x0F)
        mV = 4343;

    return mV;
}

uint16_t BQ25601_get_vreg_mV()
{
    uint8_t reg_value = _register[BQ25601_REG04];
    return BQ25601_get_vreg_mV_from_reg(reg_value);
}

uint16_t BQ25601_set_vreg_mV(uint16_t vreg_mV)
{
    if (vreg_mV < 3847 || vreg_mV > 4615)
        return 0;

    uint16_t offset = vreg_mV - 3847;
    uint16_t vreg = offset >> 5;
    uint8_t reg_value = vreg << 3;
    
    uint16_t set_to = BQ25601_get_vreg_mV_from_reg(reg_value);;

    BQ25601_read_register(BQ25601_REG04);
    uint8_t new_reg04 = _register[BQ25601_REG04] & 0x07;
    new_reg04 |= reg_value;

    BQ25601_set_register(BQ25601_REG04, new_reg04);
    BQ25601_read_register(BQ25601_REG04);
    return set_to;
}

topoff_timer_enum BQ25601_get_topoff_timer()
{
    return (_register[BQ25601_REG04] >> 1) & 0x03;
}

void BQ25601_set_topoff_timer(topoff_timer_enum value)
{
    BQ25601_read_register(BQ25601_REG04);
    uint8_t topoff_timer = value << 1;
    
    uint8_t new_reg04 = _register[BQ25601_REG04] & 0xF9;
    new_reg04 |= topoff_timer;
    BQ25601_set_register(BQ25601_REG04, new_reg04);
    BQ25601_read_register(BQ25601_REG04);
}

rechg_thresh_enum BQ25601_get_recharge_thresh()
{
    return _register[BQ25601_REG04] & 0x01;
}

void BQ25601_set_recharge_thresh(rechg_thresh_enum value)
{
    BQ25601_read_register(BQ25601_REG04);

    uint8_t new_reg04 = _register[BQ25601_REG04] & ~(0x01);
    if (value == VRECHG_200mV)
    {
        new_reg04 |= 0x01;
    }
    BQ25601_set_register(BQ25601_REG04, new_reg04);
    BQ25601_read_register(BQ25601_REG04);
}

//////////////////////////////// REG05 ////////////////////////////////


bool BQ25601_get_enable_termination()
{
    return (_register[BQ25601_REG05] & 0x80) != 0;
}

void BQ25601_set_enable_termination(bool value)
{
    BQ25601_read_register(BQ25601_REG05);

    uint8_t new_reg05 = _register[BQ25601_REG05] & ~(0x80);
    if (value) 
    {
        new_reg05 |= 0x80;
    }
    BQ25601_set_register(BQ25601_REG05, new_reg05);
    BQ25601_read_register(BQ25601_REG05);
}

watchdog_enum BQ25601_get_watchdog_time()
{
    return (_register[BQ25601_REG05] & 0x30) >> 4;
}

void BQ25601_set_watchdog_time(watchdog_enum value)
{
    BQ25601_read_register(BQ25601_REG05);

    uint8_t new_reg05 = _register[BQ25601_REG05] & ~(0x30);
    new_reg05 |= (value << 4);
    BQ25601_set_register(BQ25601_REG05, new_reg05);
    BQ25601_read_register(BQ25601_REG05);
}

bool BQ25601_get_enable_timer()
{
    return (_register[BQ25601_REG05] & 0x08) != 0;
}

void BQ25601_set_enable_timer(bool value)
{
    BQ25601_read_register(BQ25601_REG05);

    uint8_t new_reg05 = _register[BQ25601_REG05] & ~(0x08);
    if (value) 
    {
        new_reg05 |= 0x08;
    }
    BQ25601_set_register(BQ25601_REG05, new_reg05);
    BQ25601_read_register(BQ25601_REG05);
}

charge_time_enum BQ25601_get_charge_timer()
{
    return (_register[BQ25601_REG05] & 0x04) >> 2;
}

void BQ25601_set_charge_timer(charge_time_enum value)
{
    BQ25601_read_register(BQ25601_REG05);

    uint8_t new_reg05 = _register[BQ25601_REG05] & ~(0x04);
    if (value == CHG_TIMER_10_HRS) 
    {
        new_reg05 |= 0x04;
    }
    BQ25601_set_register(BQ25601_REG05, new_reg05);
    BQ25601_read_register(BQ25601_REG05);
}

thermal_reg_enum BQ25601_get_thermal_reg_temp()
{
    return (_register[BQ25601_REG05] & 0x02) >> 1;
}

void BQ25601_set_thermal_reg_temp(thermal_reg_enum value)
{
    BQ25601_read_register(BQ25601_REG05);

    uint8_t new_reg05 = _register[BQ25601_REG05] & ~(0x02);
    if (value == TREG_110C) 
    {
        new_reg05 |= 0x02;
    }
    BQ25601_set_register(BQ25601_REG05, new_reg05);
    BQ25601_read_register(BQ25601_REG05);
}

//////////////////////////////// REG06 ////////////////////////////////

ovp_enum BQ25601_get_ovp()
{
    return (_register[BQ25601_REG06] & 0xC0) >> 6;
}

void BQ25601_set_ovp(ovp_enum value)
{
    BQ25601_read_register(BQ25601_REG06);

    uint8_t new_reg06 = _register[BQ25601_REG06] & ~(0xC0);
    new_reg06 |= (value << 6);
    BQ25601_set_register(BQ25601_REG06, new_reg06);
    BQ25601_read_register(BQ25601_REG06);
}

boost_voltage_enum  BQ25601_get_boost_voltage()
{
    return (_register[BQ25601_REG06] & 0x30) >> 4;
}

void BQ25601_set_boost_voltage(boost_voltage_enum value)
{
    BQ25601_read_register(BQ25601_REG06);

    uint8_t new_reg06 = _register[BQ25601_REG06] & ~(0x30);
    new_reg06 |= (value << 4);
    BQ25601_set_register(BQ25601_REG06, new_reg06);
    BQ25601_read_register(BQ25601_REG06);
}

uint16_t BQ25601_get_absolute_vindpm_threshold_mV()
{
    const uint16_t offset_mV = 3900;
    uint8_t vindpm = _register[BQ25601_REG06] & 0x0F;
    uint16_t mV = (vindpm * 100) + offset_mV;

    return mV;
}

uint16_t BQ25601_set_absolute_vindpm_threshold_mV(uint16_t mV)
{
    const uint16_t offset_mV = 3900;

    if (mV < offset_mV || mV > 5400)
        return 0;

    uint16_t vindpm = (mV - offset_mV) / 100;

    BQ25601_read_register(BQ25601_REG06);
    uint8_t new_reg06 = _register[BQ25601_REG06] & ~(0x0F);
    new_reg06 |= vindpm;
    BQ25601_set_register(BQ25601_REG06, new_reg06);
    BQ25601_read_register(BQ25601_REG06);

    return BQ25601_get_absolute_vindpm_threshold_mV();
}

//////////////////////////////// REG07 ////////////////////////////////


bool BQ25601_get_in_current_lim_detection()
{
    return ((_register[BQ25601_REG07] & 0x80) != 0);
}

void BQ25601_set_in_current_lim_detection(bool value)
{
    BQ25601_read_register(BQ25601_REG07);

    uint8_t new_reg07 = _register[BQ25601_REG07] & ~(0x80);
    if (value) 
    {
        new_reg07 |= 0x80;
    }
    BQ25601_set_register(BQ25601_REG07, new_reg07);
    BQ25601_read_register(BQ25601_REG07);
}

bool BQ25601_get_x2_slow_safety_timer()
{
    return ((_register[BQ25601_REG07] & 0x40) != 0);
}

void BQ25601_set_x2_slow_safety_timer(bool value)
{
    BQ25601_read_register(BQ25601_REG07);

    uint8_t new_reg07 = _register[BQ25601_REG07] & ~(0x40);
    if (value) 
    {
        new_reg07 |= 0x40;
    }
    BQ25601_set_register(BQ25601_REG07, new_reg07);
    BQ25601_read_register(BQ25601_REG07);
}

bool BQ25601_get_batfet_disable()
{
    return ((_register[BQ25601_REG07] & 0x20) != 0);
}

void BQ25601_set_batfet_disable(bool value)
{
    BQ25601_read_register(BQ25601_REG07);

    uint8_t new_reg07 = _register[BQ25601_REG07] & ~(0x20);
    if (value) 
    {
        new_reg07 |= 0x20;
    }
    BQ25601_set_register(BQ25601_REG07, new_reg07);
    BQ25601_read_register(BQ25601_REG07);
}

bool BQ25601_get_batfet_delay()
{
    return ((_register[BQ25601_REG07] & 0x08) != 0);
}

void BQ25601_set_batfet_delay(bool value)
{
    BQ25601_read_register(BQ25601_REG07);

    uint8_t new_reg07 = _register[BQ25601_REG07] & ~(0x08);
    if (value) 
    {
        new_reg07 |= 0x08;
    }
    BQ25601_set_register(BQ25601_REG07, new_reg07);
    BQ25601_read_register(BQ25601_REG07);
}

bool BQ25601_get_batfet_reset_enable()
{
    return ((_register[BQ25601_REG07] & 0x04) != 0);
}

void BQ25601_set_batfet_reset_enable(bool value)
{
    BQ25601_read_register(BQ25601_REG07);

    uint8_t new_reg07 = _register[BQ25601_REG07] & ~(0x04);
    if (value) 
    {
        new_reg07 |= 0x04;
    }
    BQ25601_set_register(BQ25601_REG07, new_reg07);
    BQ25601_read_register(BQ25601_REG07);
}

vdpm_bat_track_enum BQ25601_get_vdpm_bat_track()
{
    return ((_register[BQ25601_REG07] & 0x03));
}

void BQ25601_set_vdpm_bat_track(vdpm_bat_track_enum value)
{
    BQ25601_read_register(BQ25601_REG07);

    uint8_t new_reg07 = _register[BQ25601_REG07] & ~(0x03);
    new_reg07 |= value;
    BQ25601_set_register(BQ25601_REG07, new_reg07);
    BQ25601_read_register(BQ25601_REG07);
}

//////////////////////////////// REG08 ////////////////////////////////

vbus_status_enum BQ25601_vbus_source()
{
    return _register[BQ25601_REG08] >> 5;
}

charge_status_enum BQ25601_charge_status()
{
    return (_register[BQ25601_REG08] >> 3) & 0x03;
}

bool BQ25601_power_good()
{
    return (_register[BQ25601_REG08] & 0x04) != 0;
}

bool BQ25601_thermal_status()
{
    return (_register[BQ25601_REG08] & 0x02) != 0;
}

bool BQ25601_vsys_reg()
{
    return (_register[BQ25601_REG08] & 0x01) != 0;
}

//////////////////////////////// REG09 ////////////////////////////////

bool BQ25601_watchdog_fault()
{
    return (_register[BQ25601_REG09] & 0x80) != 0;
}

bool BQ25601_boost_fault()
{
    return (_register[BQ25601_REG09] & 0x40) != 0;
}

charge_fault_enum BQ25601_charge_fault()
{
    return (_register[BQ25601_REG09] >> 4) & 0x03;
}

bool BQ25601_bat_fault()
{
    return (_register[BQ25601_REG09] & 0x08) != 0;
}

ntc_fault_enum BQ25601_ntc_fault()
{
    return _register[BQ25601_REG09] & 0x07;
}

//////////////////////////////// REG0A ////////////////////////////////


bool BQ25601_vbus_power_good()
{
    return (_register[BQ25601_REG0A] & 0x80) != 0;
}

bool BQ25601_in_vindpm()
{
    return (_register[BQ25601_REG0A] & 0x40) != 0;
}

bool BQ25601_in_iindpm()
{
    return (_register[BQ25601_REG0A] & 0x20) != 0;
}

bool BQ25601_top_off_active()
{
    return (_register[BQ25601_REG0A] & 0x08) != 0;
}

bool BQ25601_input_over_voltage()
{
    return (_register[BQ25601_REG0A] & 0x04) != 0;
}

bool BQ25601_get_mask_vindpm_int()
{
    return (_register[BQ25601_REG0A] & 0x02) != 0;
}

void BQ25601_set_mask_vindpm_int(bool value)
{
    BQ25601_read_register(BQ25601_REG0A);

    uint8_t new_reg0A = _register[BQ25601_REG0A] & ~(0x02);
    if (value) 
    {
        new_reg0A |= 0x02;
    }
    BQ25601_set_register(BQ25601_REG0A, new_reg0A);
    BQ25601_read_register(BQ25601_REG0A);
}

bool BQ25601_get_mask_iindpm_int()
{
    return (_register[BQ25601_REG0A] & 0x01) != 0;
}

void BQ25601_set_mask_iindpm_int(bool value)
{
    BQ25601_read_register(BQ25601_REG0A);

    uint8_t new_reg0A = _register[BQ25601_REG0A] & ~(0x01);
    if (value) 
    {
        new_reg0A |= 0x01;
    }
    BQ25601_set_register(BQ25601_REG0A, new_reg0A);
    BQ25601_read_register(BQ25601_REG0A);
}

//////////////////////////////// REG0B ////////////////////////////////


bool BQ25601_get_register_reset()
{
    return (_register[BQ25601_REG0B] & 0x80) != 0;
}

void BQ25601_set_register_reset(bool value)
{
    BQ25601_read_register(BQ25601_REG0B);

    uint8_t new_reg0A = _register[BQ25601_REG0B] & ~(0x80);
    if (value) 
    {
        new_reg0A |= 0x80;
    }
    BQ25601_set_register(BQ25601_REG0B, new_reg0A);
    BQ25601_read_register(BQ25601_REG0B);
}

pn_enum BQ25601_pn()
{
    return (_register[BQ25601_REG0B] & 0x78) >> 3;
}
