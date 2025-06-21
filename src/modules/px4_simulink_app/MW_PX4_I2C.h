
/* Copyright 2020-2022 The MathWorks, Inc. */


#ifndef __MW_PX4_I2C_H__
#define __MW_PX4_I2C_H__

#include <px4_platform_common/px4_config.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <unistd.h>

#include <drivers/device/i2c.h>

#include "board_config.h"

#ifdef __cplusplus
extern "C" { /* sbcheck:ok:extern_c */
#endif


class MW_I2C_DEVICE : public device::I2C {
  public:
    MW_I2C_DEVICE(int bus);
    virtual ~MW_I2C_DEVICE() = default;

    int init();
    int setBusSpeed(int bus, uint32_t frequency);
    int read(uint16_t address, uint8_t* data, unsigned count);
    int write(uint16_t address, uint8_t* data, unsigned count);
};

MW_I2C_DEVICE* MW_I2C_DEVICE_interface(int bus);
#ifdef __cplusplus
}
#endif

#endif /* __MW_PX4_I2C_H__ */
