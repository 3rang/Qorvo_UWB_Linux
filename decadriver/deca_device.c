#include <assert.h>
#include <stdlib.h>
#include <stdint.h>
#include "deca_device_api.h"
#include "deca_version.h"
#include "spi_interface.h"
#include "deca_regs.h"

uint32_t dwt_readdevid(void)
{
    return dwt_read32bitoffsetreg(DEV_ID_ID, 0);
}

static int dwt_linux_spi_transfer(const uint8_t* tx_data, uint8_t* rx_data, size_t len)
{
    // Replace this with your own Linux SPI transfer function
    return spi_transfer(&spi,tx_data, rx_data, len);
}


uint32_t dwt_read32bitoffsetreg(int regFileID, int regOffset)
{
    int     j ;
    uint32_t  regval = 0 ;
    uint8_t   buffer[4] ;

    dwt_readfromdevice(regFileID,regOffset,4,buffer); // Read 4 bytes (32-bits) register into buffer

    for (j = 3 ; j >= 0 ; j --)
    {
        regval = (regval << 8) + buffer[j] ;
    }

    return (regval);

} // end dwt_read32bitoffsetreg()




void dwt_xfer3000(const uint32_t regFileID, const uint16_t indx, const uint16_t length, uint8_t* buffer, const spi_modes_e mode)
{
    uint8_t tx_buffer[5];
    uint8_t rx_buffer[5];

    // Construct the SPI command word
    tx_buffer[0] = (regFileID >> 16) & 0xFF;
    tx_buffer[1] = (regFileID >> 8) & 0xFF;
    tx_buffer[2] = regFileID & 0xFF;
    tx_buffer[3] = indx & 0xFF;
    tx_buffer[4] = mode;

    // Perform the SPI transfer
    dwt_linux_spi_transfer(tx_buffer, rx_buffer, 5);

    // If we are reading, copy the received data into the buffer
    if (mode == DW3000_SPI_RD_BIT) {
        dwt_linux_spi_transfer(NULL, buffer, length);
    }
    // Otherwise, if we are writing, send the data to the device
    else {
        dwt_linux_spi_transfer(buffer, NULL, length);
    }
}



void dwt_readfromdevice
(
    uint32_t  regFileID,
    uint16_t  index,
    uint16_t  length,
    uint8_t   *buffer
)
{
    dwt_xfer3000(regFileID, index, length, buffer, DW3000_SPI_RD_BIT);
}




int dwt_check_dev_id(void)
{
    uint32_t  dev_id;

    dev_id = dwt_readdevid();

	printf("dev_id \"%08x\"", dev_id);

    if (!((DWT_C0_PDOA_DEV_ID == dev_id) || (DWT_C0_DEV_ID == dev_id)))
    {
        return DWT_ERROR;
    }
    return DWT_SUCCESS;
}
