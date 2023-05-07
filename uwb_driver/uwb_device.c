#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include "uwb_device_api.h"
#include "spi_interface.h"
#include "deca_regs.h"

int32_t uwb_apiversion(void)
{
    return DW3000_DRIVER_VERSION;
}

static int uwb_linux_spi_transfer(const uint8_t* tx_data, uint8_t* rx_data, size_t len)
{
    // Replace this with your own Linux SPI transfer function
    return spi_transfer(&spi, tx_data, rx_data, len);
}

uint32_t uwb_read32bitoffsetreg(int regFileID, int regOffset)
{
    int j;
    uint32_t regval = 0;
    uint8_t buffer[5];

    uwb_readfromdevice(regFileID, regOffset, 5, buffer); // Read 4 bytes (32-bits) register into buffer

    for (j = 4; j >= 1; j--)
    {
        regval = (regval << 8) + buffer[j];
        printf("regval = %x\n", regval);
    }

    return regval;
}

uint32_t uwb_readdevid(void)
{
    return uwb_read32bitoffsetreg(DEV_ID_ID, 0);
}


void uwb_xfer3000(const uint32_t regFileID, const uint16_t indx, const uint16_t length, uint8_t* buffer, const spi_modes_e mode)
{
    uint8_t tx_buffer[5];

    // Construct the SPI command word
    tx_buffer[0] = (regFileID >> 16) & 0xFF;
    tx_buffer[1] = (regFileID >> 8) & 0xFF;
    tx_buffer[2] = regFileID & 0xFF;
    tx_buffer[3] = indx & 0xFF;
    tx_buffer[4] = mode;

    // Perform the SPI transfer
    uwb_linux_spi_transfer(tx_buffer, buffer, length);
}

void uwb_readfromdevice(uint32_t regFileID, uint16_t index, uint16_t length, uint8_t* buffer)
{
    uwb_xfer3000(regFileID, index, length, buffer, DW3000_SPI_RD_BIT);
}

int uwb_check_dev_id(void)
{
    uint32_t dev_id;

    dev_id = uwb_readdevid();

    printf("dev_id \"%x\"", dev_id);

    if (!((DWT_C0_PDOA_DEV_ID == dev_id) || (DWT_C0_DEV_ID == dev_id)))
    {
        return DWT_ERROR;
    }
    return DWT_SUCCESS;
}

