/*
 * Copyright 2017-2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
  
#include <string.h>
#include "ov7670.h"
#include "fsl_i2c.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define I2C_BAUDRATE 100000U
#define I2C_MASTER_CLOCK_FREQUENCY (12000000)
#define EXAMPLE_I2C_MASTER_BASE (I2C4_BASE)
#define EXAMPLE_I2C_MASTER ((I2C_Type *)EXAMPLE_I2C_MASTER_BASE)
#define I2C_MASTER_SLAVE_ADDR_7BIT 0x21U
#define WAIT_TIME 10U

/*******************************************************************************
 * Variables
 ******************************************************************************/
i2c_master_handle_t g_m_handle;
volatile bool completionFlag = false;
volatile bool nakFlag        = false;
static const uint8_t ov7670_init_regtbl[][2]=
{
    {0x40, 0xD0},   /* Display */
    {0x3a, 0x0C},
		{0x12, 0x14},//VGA,RGB
    {0x3d, 0x80},
    {0x67, 0x11},
    {0x68, 0xFF},
    {0x32, 0x80},   // HREF
    {0x17, 0x16},   // HSTART
    {0x18, 0x04},   // HSTOP
    {0x19, 0x02},   // VSTRT
    {0x1a, 0x7a},   // VSTOP
    {0x03, 0x0a},   // VREF
    {0x0c, 0x00},
    {0x15, 0x20},   //COM10,PCLK reverse
    {0x3e, 0x00},
    {0x70, 0x00},
    {0x71, 0x01},
    {0x72, 0x11},
    {0x73, 0x00},
    {0xa2, 0x02},
    {0x11, 0x00},   // Clock Div, Input/(n+1)
    {0x7a, 0x20},
    {0x7b, 0x1c},
    {0x7c, 0x28},
    {0x7d, 0x3c},
    {0x7e, 0x55},
    {0x7f, 0x68},
    {0x80, 0x76},
    {0x81, 0x80},
    {0x82, 0x88},
    {0x83, 0x8f},
    {0x84, 0x96},
    {0x85, 0xa3},
    {0x86, 0xaf},
    {0x87, 0xc4},
    {0x88, 0xd7},
    {0x89, 0xe8},
    {0x13, 0x66},
    {0x00, 0x00},   // AGC
    {0x10, 0xFF},
    {0x0d, 0x00},
    {0x14, 0x21},   // limit the max gain
    {0xa5, 0x05},
    {0xab, 0x07},
    {0x24, 0x75},
    {0x25, 0x63},
    {0x26, 0xA5},
    {0x9f, 0x78},
    {0xa0, 0x68},	
    {0x31, 0xff},
    {0xa1, 0x03},
    {0xa6, 0xdf},
    {0xa7, 0xdf},
    {0xa8, 0xf0},
    {0xa9, 0x90},
    {0xaa, 0x94},
    {0x0e, 0x61},
    {0x0f, 0x4b},
    {0x16, 0x02},
//    {0x1e, 0x10},   //MVFP: Mirror/VFlip
    {0x1e, 0x30},   //MVFP: Mirror/VFlip
    {0x21, 0x02},
    {0x22, 0x91},
    {0x29, 0x07},
    {0x33, 0x0b},
    {0x35, 0x0b},
    {0x37, 0x1d},
    {0x38, 0x71},
    {0x39, 0x2a},
    {0x3c, 0x00},   // COM12
    {0x4d, 0x40},
    {0x4e, 0x20},
    {0x69, 0x5d},
    {0x6b, 0x00},
    {0x74, 0x19},
    {0x8d, 0x4f},
    {0x8e, 0x00},
    {0x8f, 0x00},
    {0x90, 0x00},
    {0x91, 0x00},
    {0x92, 0x00},
    {0x96, 0x00},
    {0x9a, 0x80},
    {0xb0, 0x84},
    {0xb1, 0x0c},
    {0xb2, 0x0e},
    {0xb3, 0x82},
    {0xb8, 0x0a},
    {0x43, 0x14},
    {0x44, 0xf0},
    {0x45, 0x34},
    {0x46, 0x58},
    {0x47, 0x28},
    {0x48, 0x3a},
    {0x59, 0x88},
    {0x5a, 0x88},
    {0x5b, 0x44},
    {0x5c, 0x67},
    {0x5d, 0x49},
    {0x5e, 0x0e},
    {0x64, 0x04},
    {0x65, 0x20},
    {0x66, 0x05},
    {0x94, 0x04},
    {0x95, 0x08},
    {0x6c, 0x0a},
    {0x6d, 0x55},
    {0x4f, 0x80},
    {0x50, 0x80},
    {0x51, 0x00},
    {0x52, 0x22},
    {0x53, 0x5e},
    {0x54, 0x80},
    {0x09, 0x00},   // Output Drive Capability: 1x
    {0x6e, 0x11},
    {0x6f, 0x9f},
    {0x55, 0x00},
    {0x56, 0x40},
    {0x57, 0x80},
    {0x3F, 0x04},
    {0x4C, 0x01},
    {0x3b, 0x9F},		
		{0x07, 0x3F},

};


/*******************************************************************************
 * Code
 ******************************************************************************/

uint8_t Write_OVReg(i2c_master_transfer_t *handle, uint8_t addr, uint8_t val)
{
    uint8_t buf[2];
    buf[0] = addr;
    buf[1] = val;

    handle->slaveAddress = 0x21;
    handle->direction = kI2C_Write;
    handle->subaddress = 0x0;
    handle->subaddressSize = 0x0;
    handle->data = buf;
    handle->dataSize = 2;
    handle->flags = kI2C_TransferDefaultFlag;
    I2C_MasterTransferNonBlocking(EXAMPLE_I2C_MASTER, &g_m_handle, handle);
    /*  wait for transfer completed. */
    while ((!nakFlag) && (!completionFlag))
    {
    }
    nakFlag = false;
    if (completionFlag == true)
    {
      completionFlag = false;
    }
    return 0;
}


uint8_t Read_OVReg(i2c_master_transfer_t *handle, uint8_t addr)
{
    uint8_t val;

    handle->slaveAddress = 0x21;
    handle->direction = kI2C_Write;
    handle->subaddress = 0;
    handle->subaddressSize = 0;
    handle->data = &addr;
    handle->dataSize = 1;
    handle->flags = kI2C_TransferDefaultFlag;
    I2C_MasterTransferNonBlocking(EXAMPLE_I2C_MASTER, &g_m_handle, handle);
    /*  wait for transfer completed. */
    while ((!nakFlag) && (!completionFlag))
    {
    }
    nakFlag = false;

    if (completionFlag == true)
    {
      completionFlag = false;
    }

    handle->slaveAddress = 0x21;
    handle->direction = kI2C_Read;
    handle->subaddress = 0;
    handle->subaddressSize = 0;
    handle->data = &val;
    handle->dataSize = 1;
    handle->flags = kI2C_TransferRepeatedStartFlag;
    I2C_MasterTransferNonBlocking(EXAMPLE_I2C_MASTER, &g_m_handle, handle);
    /*  wait for transfer completed. */
    while ((!nakFlag) && (!completionFlag))
    {
    }
    nakFlag = false;
    if (completionFlag == true)
    {
      completionFlag = false;
    }
    return val;
}

void OV7670_config_window(uint16_t startx, uint16_t starty, uint16_t width, uint16_t height)
{
    uint16_t endx = (startx + width * 2) % 784;
    uint16_t endy = (starty + height * 2);
    uint8_t x_reg, y_reg;
    uint8_t state, temp;
    i2c_master_transfer_t masterXfer;

    x_reg = Read_OVReg(&masterXfer, 0x32);
    x_reg &= 0xC0;
    y_reg = Read_OVReg(&masterXfer, 0x03);
    y_reg &= 0xF0;

    // HREF
    temp = x_reg | ((endx & 0x7) << 3) | (startx & 0x7);
    state = Write_OVReg(&masterXfer, 0x32, temp);

    temp = (startx & 0x7F8) >> 3;
    state = Write_OVReg(&masterXfer, 0x17, temp);
    temp = (endx & 0x7F8) >> 3;
    state = Write_OVReg(&masterXfer, 0x18, temp);

    // VREF
    temp = y_reg | ((endy & 0x3) << 2) | (starty & 0x3);
    state = Write_OVReg(&masterXfer, 0x03, temp);
    temp = (starty & 0x3FC) >> 2;
    state = Write_OVReg(&masterXfer, 0x19, temp);
    temp = (endy & 0x3FC) >> 2;
    state = Write_OVReg(&masterXfer, 0x1A, temp);
}

static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData)
{
    /* Signal transfer success when received success status. */
    if (status == kStatus_Success)
    {
      completionFlag = true;
    }
    /* Signal transfer success when received success status. */
    if ((status == kStatus_I2C_Nak) || (status == kStatus_I2C_Addr_Nak))
    {
      nakFlag = true;
    }
}

int Ov7670_Init(uint32_t instance)
{
    int i, j;

    uint8_t id;
    uint8_t id_value;
    i2c_master_config_t masterConfig;
    I2C_MasterTransferCreateHandle(EXAMPLE_I2C_MASTER, &g_m_handle, i2c_master_callback, NULL);
    /*
     * masterConfig.debugEnable = false;
     * masterConfig.ignoreAck = false;
     * masterConfig.pinConfig = kI2C_2PinOpenDrain;
     * masterConfig.baudRate_Bps = 100000U;
     * masterConfig.busIdleTimeout_ns = 0;
     * masterConfig.pinLowTimeout_ns = 0;
     * masterConfig.sdaGlitchFilterWidth_ns = 0;
     * masterConfig.sclGlitchFilterWidth_ns = 0;
     */
    I2C_MasterGetDefaultConfig(&masterConfig);
    /* Change the default baudrate configuration */
    masterConfig.baudRate_Bps = I2C_BAUDRATE;
    /* Initialize the I2C master peripheral */
    I2C_MasterInit(EXAMPLE_I2C_MASTER, &masterConfig, I2C_MASTER_CLOCK_FREQUENCY);

    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));

    id = 0x0a;
    masterXfer.slaveAddress = 0x21;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data = &id;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferDefaultFlag;
    I2C_MasterTransferNonBlocking(EXAMPLE_I2C_MASTER, &g_m_handle, &masterXfer);
    /*  wait for transfer completed. */
    while ((!nakFlag) && (!completionFlag))
    {
    }
    nakFlag = false;

    if (completionFlag == true)
    {
      completionFlag = false;
    }

    masterXfer.slaveAddress = 0x21;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data = &id_value;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferRepeatedStartFlag;
    I2C_MasterTransferNonBlocking(EXAMPLE_I2C_MASTER, &g_m_handle, &masterXfer);
    /*  wait for transfer completed. */
    while ((!nakFlag) && (!completionFlag))
    {
    }
    nakFlag = false;
    if (completionFlag == true)
    {
      completionFlag = false;
    }

    for (j = 0; j < ARRAY_SIZE(ov7670_init_regtbl); j++)
    {
      masterXfer.slaveAddress = 0x21;
      masterXfer.direction = kI2C_Write;
      masterXfer.subaddress = 0x0;
      masterXfer.subaddressSize = 0x0;
      masterXfer.data = &ov7670_init_regtbl[j][0];
      masterXfer.dataSize = 2;
      masterXfer.flags = kI2C_TransferDefaultFlag;
      I2C_MasterTransferNonBlocking(EXAMPLE_I2C_MASTER, &g_m_handle, &masterXfer);
      /*  wait for transfer completed. */
      while ((!nakFlag) && (!completionFlag))
      {
      }
      nakFlag = false;
      if (completionFlag == true)
      {
        completionFlag = false;
      }
    }

    //		OV7670_config_window(184,10,160,120);
    OV7670_config_window(184, 10, 28 * 8, 28 * 8);

    return 0;
}


