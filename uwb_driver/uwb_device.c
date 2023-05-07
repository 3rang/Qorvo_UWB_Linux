#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <unistd.h>
#include "uwb_device_api.h"
#include "spi_interface.h"
#include "deca_regs.h"






#define SEL_CHANNEL5            (5)
#define SEL_CHANNEL9            (9)



//Device data 


typedef struct
{
    uint32_t      partID ;            // IC Part ID - read during initialisation
    uint32_t      lotID ;             // IC Lot ID - read during initialisation
    uint8_t       bias_tune;          // bias tune code
    uint8_t       dgc_otp_set;        // Flag to check if DGC values are programmed in OTP
    uint8_t       vBatP;              // IC V bat read during production and stored in OTP (Vmeas @ 3V3)
    uint8_t       tempP;              // IC temp read during production and stored in OTP (Tmeas @ 23C)
    uint8_t       longFrames ;        // Flag in non-standard long frame mode
    uint8_t       otprev ;            // OTP revision number (read during initialisation)
    uint8_t       init_xtrim;         // initial XTAL trim value read from OTP (or defaulted to mid-range if OTP not programmed)
    uint8_t       dblbuffon;          // Double RX buffer mode and DB status flag
    uint16_t      sleep_mode;         // Used for automatic reloading of LDO tune and microcode at wake-up
    int16_t       ststhreshold;       // Threshold for deciding if received STS is good or bad
    uwb_spi_crc_mode_e   spicrc;      // Use SPI CRC when this flag is true
    uint8_t       stsconfig;          // STS configuration mode
    uint8_t       cia_diagnostic;     // CIA dignostic logging level
    uwb_cb_data_t cbData;             // Callback data structure
    uwb_spierrcb_t cbSPIRDErr;        // Callback for SPI read error events
    uwb_cb_t    cbTxDone;             // Callback for TX confirmation event
    uwb_cb_t    cbRxOk;               // Callback for RX good frame event
    uwb_cb_t    cbRxTo;               // Callback for RX timeout events
    uwb_cb_t    cbRxErr;              // Callback for RX error events
    uwb_cb_t    cbSPIErr;             // Callback for SPI error events
    uwb_cb_t    cbSPIRdy;             // Callback for SPI ready events
} uwb_local_data_t ;




// Local variable declaration 
//

static uwb_local_data_t   DW3000local[NUMBER_DEVICES] ; // Local device data, array to support multiple DW3XXX
static uwb_local_data_t *pdw3000local = &DW3000local[0];   // Local data structure pointer
static uint8_t crcTable[256];


/**************************************************************************
*                           Function definition
***************************************************************************/


/**********
 *
 *uwb_apiversion():-  version of the UWB driver API
 */

int32_t uwb_apiversion(void)
{
    return DRIVER_VERSION;
}

/**********
 *
 *uwb_setlocaldataptr():-   local data structure pointer
 */

int uwb_setlocaldataptr(unsigned int index)
{
    // Check the index is within the array bounds
    if (NUMBER_DEVICES <= index) // return error if index outside the array bounds
    {
        return DWT_ERROR ;
    }

    pdw3000local = &DW3000local[index];

    return DWT_SUCCESS ;
}



/**********
 *
 *uwb_xfer3000():- read/write to the DW3000 device registers
 */


static void uwb_xfer3000(const uint32_t regFileID, const uint16_t indx, const uint16_t length, uint8_t* buffer, const spi_modes_e mode)
{
    uint8_t header[2];
    uint16_t cnt = 0;
    uint16_t reg_file = 0x1F & ((regFileID + indx) >> 16);
    uint16_t reg_offset = 0x7F & (regFileID + indx);
    uint16_t addr;

    addr = (reg_file << 9) | (reg_offset << 2);

    header[0] = (mode | addr) >> 8;
    header[1] = addr | (mode & 0x03);

    if (length == 0) {
        header[0] = (DW3000_SPI_WR_BIT >> 8) | (regFileID << 1) | DW3000_SPI_FAC;
        cnt = 1;
    } else if (reg_offset == 0 && (mode == DW3000_SPI_WR_BIT || mode == DW3000_SPI_RD_BIT)) {
        cnt = 1;
    } else {
        cnt = 2;
    }

    switch (mode) {
        case DW3000_SPI_AND_OR_8:
        case DW3000_SPI_AND_OR_16:
        case DW3000_SPI_AND_OR_32:
        case DW3000_SPI_WR_BIT:
            // Write to SPI
            // Modify this part to use your Linux SPI library
            break;
        case DW3000_SPI_RD_BIT:
            // Read from SPI
    writetospi(header, cnt, buffer, length);
            // Modify this part to use your Linux SPI library
            break;
        default:
            // Invalid mode
            while (1);
            break;
    }

    /*uint8_t tx_buffer[5];

    // Construct the SPI command word
    tx_buffer[0] = (regFileID >> 16) & 0xFF;
    tx_buffer[1] = (regFileID >> 8) & 0xFF;
    tx_buffer[2] = regFileID & 0xFF;
    tx_buffer[3] = indx & 0xFF;
    tx_buffer[4] = mode;
*/
    // Perform the SPI transfer
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
    uint8_t buffer[4];

    uwb_readfromdevice(regFileID, regOffset, 5, buffer); 

    for (j = 4; j >= 1; j--)
    {
        regval = (regval << 8) + buffer[j];
    }

    return regval;
}

uint32_t uwb_readdevid(void)
{
    return uwb_read32bitoffsetreg(DEV_ID_ID, 0);
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


uint16_t uwb_read16bitoffsetreg(uint16_t regFileID, uint16_t regOffset)
{
    uint8_t tx_buffer[3];
    uint8_t rx_buffer[3];

    // Construct the SPI command word
    tx_buffer[0] = (regFileID >> 8) & 0xFF;
    tx_buffer[1] = regFileID & 0xFF;
    tx_buffer[2] = regOffset & 0xFF;

    // Perform the SPI transfer
    uwb_linux_spi_transfer(tx_buffer, rx_buffer, 3);

    // Extract the 16-bit register value from the received buffer
    uint16_t regval = (rx_buffer[1] << 8) | rx_buffer[2];

    return regval;
}

uint8_t uwb_checkidlerc(void)
{
    /* Wait 2 ms for DW IC to get into IDLE_RC state */
    usleep(2000);  // Sleep for 2 ms

    /* Poll DW IC until IDLE_RC event set.
     * This means that DW IC is in IDLE_RC state and ready */
    uint32_t reg = ((uint32_t)uwb_read16bitoffsetreg(SYS_STATUS_ID, 2) << 16);

    return ((reg & (SYS_STATUS_RCINIT_BIT_MASK)) == (SYS_STATUS_RCINIT_BIT_MASK));
}


