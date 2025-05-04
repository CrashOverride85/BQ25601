
# BQ25601D
This library interfaces with the Texas Instruments BQ25601D charger.

Using [@cvetaevvitaliy](https://github.com/cvetaevvitaliy)'s [bq27441](https://github.com/cvetaevvitaliy/bq27441) fuel gauge library as somewhat of a template in places..

I've tested this with a Raspberry Pico, but there shouldn't be anything Pico specific in the library.

---------------------

Usage
-----

Copy sources, add header file to your project
```c
#include "bq25601.h"
```

The library does not depend on the i2c implementation, you need to implement this yourself and register read/write callbacks before initializing BQ25601 chip


i2c read/write
```c
static int16_t BQ25601_i2cWriteBytes(uint8_t DevAddress, uint8_t subAddress, uint8_t* src, uint8_t count)
{
    /* write subAddress to DevAddress, then count bytes from src */
}

static int16_t BQ25601_i2cReadBytes(uint8_t DevAddress, uint8_t subAddress, uint8_t* dest, uint8_t count)
{
    /* write subAddress to DevAddress, then read count bytes into dest */
}
```

.... <br>

Init the library:
```c
    BQ25601_ctx_t BQ25601 = {
            .BQ25601_i2c_address = BQ25601_I2C_ADDRESS, // i2c device address, if you have another address change this
            .read_reg = BQ25601_i2cReadBytes,           // i2c read callback 
            .write_reg = BQ25601_i2cWriteBytes,         // i2c write callback 
    };
    
/**
 * Your other code
 * */

BQ25601_init(&BQ25601);

```

After init chip and register read/write functions, you can interact with the charge controller. All function descriptions are in source files.

Functions read from a cache not the BQ25601, so either call `BQ25601_read_register(<register>)` or `BQ25601_read_all_registers()` to update.

For example read charge status:
```c
    BQ25601_read_register(BQ25601_REG08); /* CHRG_STAT is in REG08 */
    charge_status_enum status = BQ25601_charge_status();
    switch (status)
    {
        case CHRG_STAT_NOT_CHARGING:    printf("Not charging\n");       break;
        case CHRG_STAT_PRE_CHARGE:      printf("Pre-charge\n");         break;
        case CHRG_STAT_FAST_CHARGING:   printf("Fast charging\n");      break;
        case CHRG_STAT_CHARGE_TERM:     printf("Charge termination\n"); break;
    };
```

