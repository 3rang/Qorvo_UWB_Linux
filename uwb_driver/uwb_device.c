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
        return ERROR ;
    }

    pdw3000local = &DW3000local[index];

    return SUCCESS ;
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
		header[0] = (DW3000_SPI_WR_BIT >> 8) | (regFileID << 1);
		cnt = 1;
	} else if (reg_offset == 0 && (mode == DW3000_SPI_WR_BIT || mode == DW3000_SPI_RD_BIT)) {
		header[0] |= 1;
		cnt = 1;
	} else {
		header[0] |= 2;
		cnt = 2;
	}

	switch (mode) {
		case    DW3000_SPI_AND_OR_8:
		case    DW3000_SPI_AND_OR_16:
		case    DW3000_SPI_AND_OR_32:
		case    DW3000_SPI_WR_BIT:
			{	// Write to SPI
			uint8_t crc8 = 0;
			if (pdw3000local->spicrc != DWT_SPI_CRC_MODE_NO)
			{
				//generate 8 bit CRC
				crc8 = uwb_generatecrc8(header, cnt, 0);
				crc8 = uwb_generatecrc8(buffer, length, crc8);

				// Write it to the SPI
				writetospiwithcrc(cnt, header, length, buffer, crc8);
			}
			else
			{
				// Write it to the SPI
				writetospi(cnt, header, length, buffer);
			}
			break;
			}
		case    DW3000_SPI_RD_BIT:
			// Read from SPI
			readfromspi(cnt, header, length,buffer);
			break;
		default:
			// Invalid mode
			while (1);
			break;
	}

}

//read functions

uint32_t uwb_read32bitoffsetreg(int regFileID, int regOffset)
{
	int j;
	uint32_t regval = 0;
	uint8_t buffer[4];

	uwb_readfromdevice(regFileID, regOffset, 4, buffer); 

	for (j = 3; j >= 0; j--)
	{
		regval = (regval << 8) + buffer[j];
	}

	return regval;
}


uint16_t uwb_read16bitoffsetreg(uint16_t regFileID, uint16_t regOffset)
{

	uint16_t  regval = 0 ;
	uint8_t   buffer[2] ;

	uwb_readfromdevice(regFileID,regOffset,2,buffer); // Read 2 bytes (16-bits) register into buffer

	regval = ((uint16_t)buffer[1] << 8) + buffer[0] ;
	return regval ;
}

uint8_t uwb_read8bitoffsetreg(int regFileID, int regOffset)
{
	uint8_t regval;

	uwb_readfromdevice(regFileID, regOffset, 1, &regval);

	return regval ;
}

//write functions

void uwb_write32bitoffsetreg(int regFileID, int regOffset, uint32_t regval)
{
	int     j ;
	uint8_t   buffer[4] ;

	for ( j = 0 ; j < 4 ; j++ )
	{
		buffer[j] = (uint8_t)regval;
		regval >>= 8 ;
	}

	uwb_writetodevice(regFileID,regOffset,4,buffer);
} 

void uwb_write16bitoffsetreg(int regFileID, int regOffset, uint16_t regval)
{
	uint8_t   buffer[2] ;

	buffer[0] = (uint8_t)regval;
	buffer[1] = regval >> 8 ;

	uwb_writetodevice(regFileID,regOffset,2,buffer);
} 

//modify
//

void uwb_modify32bitoffsetreg(const int regFileID, const int regOffset, const uint32_t _and, const uint32_t _or)
{
    uint8_t buf[8];
    buf[0] = (uint8_t)_and;//       &0xFF;
    buf[1] = (uint8_t)(_and>>8);//  &0xFF;
    buf[2] = (uint8_t)(_and>>16);// &0xFF;
    buf[3] = (uint8_t)(_and>>24);// &0xFF;
    buf[4] = (uint8_t)_or;//        &0xFF;
    buf[5] = (uint8_t)(_or>>8);//   &0xFF;
    buf[6] = (uint8_t)(_or>>16);//  &0xFF;
    buf[7] = (uint8_t)(_or>>24);//  &0xFF;
    uwb_xfer3000(regFileID, regOffset, sizeof(buf), buf, DW3000_SPI_AND_OR_32);
}

void uwb_modify16bitoffsetreg(const int regFileID, const int regOffset, const uint16_t _and, const uint16_t _or)
{
    uint8_t buf[4];
    buf[0] = (uint8_t)_and;//       &0xFF;
    buf[1] = (uint8_t)(_and>>8);//  &0xFF;
    buf[2] = (uint8_t)_or;//        &0xFF;
    buf[3] = (uint8_t)(_or>>8);//   &0xFF;
    uwb_xfer3000(regFileID, regOffset, sizeof(buf), buf, DW3000_SPI_AND_OR_16);
}

void uwb_modify8bitoffsetreg(const int regFileID, const int regOffset, const uint8_t _and, const uint8_t _or)
{
    uint8_t buf[2];
    buf[0] = _and;
    buf[1] = _or;
    uwb_xfer3000(regFileID, regOffset, sizeof(buf),buf, DW3000_SPI_AND_OR_8);
}


static void _uwb_prog_ldo_and_bias_tune(void)
{
	uwb_or16bitoffsetreg(OTP_CFG_ID, 0, LDO_BIAS_KICK);
	uwb_and_or16bitoffsetreg(BIAS_CTRL_ID, 0, (uint16_t)~BIAS_CTRL_BIAS_MASK, pdw3000local->bias_tune);
}

static void _uwb_kick_ops_table_on_wakeup(void)
{
    /* Restore OPS table config and kick. */
    /* Correct sleep mode should be set by dwt_configure() */

    /* Using the mask of all available OPS table options, check for OPS table options in the sleep mode mask */
    switch (pdw3000local->sleep_mode & (DWT_ALT_OPS | DWT_SEL_OPS0 | DWT_SEL_OPS1 | DWT_SEL_OPS2 | DWT_SEL_OPS3))
    {
    /* If preamble length >= 256 and set by dwt_configure(), the OPS table should be kicked off like so upon wakeup. */
    case (DWT_ALT_OPS | DWT_SEL_OPS0):
        uwb_modify32bitoffsetreg(OTP_CFG_ID, 0, ~(OTP_CFG_OPS_ID_BIT_MASK), DWT_OPSET_LONG | OTP_CFG_OPS_KICK_BIT_MASK);
        break;
    /* If SCP mode is enabled by dwt_configure(), the OPS table should be kicked off like so upon wakeup. */
    case (DWT_ALT_OPS | DWT_SEL_OPS1):
        uwb_modify32bitoffsetreg(OTP_CFG_ID, 0, ~(OTP_CFG_OPS_ID_BIT_MASK), DWT_OPSET_SCP | OTP_CFG_OPS_KICK_BIT_MASK);
        break;
    default:
        break;
    }
}


static void _dwt_kick_dgc_on_wakeup(int8_t channel)
{
    /* The DGC_SEL bit must be set to '0' for channel 5 and '1' for channel 9 */
    if (channel == 5)
    {
        uwb_modify32bitoffsetreg(OTP_CFG_ID, 0, ~(OTP_CFG_DGC_SEL_BIT_MASK),
                (DWT_DGC_SEL_CH5 << OTP_CFG_DGC_SEL_BIT_OFFSET) | OTP_CFG_DGC_KICK_BIT_MASK);
    }
    else if (channel == 9)
    {
        uwb_modify32bitoffsetreg(OTP_CFG_ID, 0, ~(OTP_CFG_DGC_SEL_BIT_MASK),
                (DWT_DGC_SEL_CH9 << OTP_CFG_DGC_SEL_BIT_OFFSET) | OTP_CFG_DGC_KICK_BIT_MASK);
    }
}


int uwb_initialise(int mode)
{
   //uint16_t otp_addr;
   //uint32_t devid;
    uint32_t ldo_tune_lo;
    uint32_t ldo_tune_hi;

    pdw3000local->dblbuffon = DBL_BUFF_OFF; // Double buffer mode off by default / clear the flag
    pdw3000local->sleep_mode = DWT_RUNSAR;  // Configure RUN_SAR on wake by default as it is needed when running PGF_CAL
    pdw3000local->spicrc = 0;
    pdw3000local->stsconfig = 0; //STS off
    pdw3000local->vBatP = 0;
    pdw3000local->tempP = 0;

    pdw3000local->cbTxDone = NULL;
    pdw3000local->cbRxOk = NULL;
    pdw3000local->cbRxTo = NULL;
    pdw3000local->cbRxErr = NULL;
    pdw3000local->cbSPIRdy = NULL;
    pdw3000local->cbSPIErr = NULL;

    // Read and validate device ID return -1 if not recognised
    if (uwb_check_dev_id()!=SUCCESS)
    {
        return ERROR;
    }

    //Read LDO_TUNE and BIAS_TUNE from OTP
    ldo_tune_lo = _uwb_otpread(LDOTUNELO_ADDRESS);
    ldo_tune_hi = _uwb_otpread(LDOTUNEHI_ADDRESS);
    pdw3000local->bias_tune = (_uwb_otpread(BIAS_TUNE_ADDRESS) >> 16) & BIAS_CTRL_BIAS_MASK;

    if ((ldo_tune_lo != 0) && (ldo_tune_hi != 0) && (pdw3000local->bias_tune != 0))
    {
        _uwb_prog_ldo_and_bias_tune();
    }

    // Read DGC_CFG from OTP
    if (_uwb_otpread(DGC_TUNE_ADDRESS) == DWT_DGC_CFG0)
    {
        pdw3000local->dgc_otp_set = DWT_DGC_LOAD_FROM_OTP;
    }
    else
    {
        pdw3000local->dgc_otp_set = DWT_DGC_LOAD_FROM_SW;
    }

    // Load Part and Lot ID from OTP
    if(mode & DWT_READ_OTP_PID)
        pdw3000local->partID = _uwb_otpread(PARTID_ADDRESS);
    if (mode & DWT_READ_OTP_LID)
        pdw3000local->lotID = _uwb_otpread(LOTID_ADDRESS);
    if (mode & DWT_READ_OTP_BAT)
        pdw3000local->vBatP = (uint8_t)_uwb_otpread(VBAT_ADDRESS);
    if (mode & DWT_READ_OTP_TMP)
        pdw3000local->tempP = (uint8_t)_uwb_otpread(VTEMP_ADDRESS);


    if(pdw3000local->tempP == 0) //if the reference temperature has not been programmed in OTP (early eng samples) set to default value
    {
        pdw3000local->tempP = 0x85 ; //@temp of 20 deg
    }

    if(pdw3000local->vBatP == 0) //if the reference voltage has not been programmed in OTP (early eng samples) set to default value
    {
        pdw3000local->vBatP = 0x74 ;  //@Vref of 3.0V
    }

    pdw3000local->otprev = (uint8_t) _uwb_otpread(OTPREV_ADDRESS);

    pdw3000local->init_xtrim = _uwb_otpread(XTRIM_ADDRESS) & 0x7f;
    if(pdw3000local->init_xtrim == 0)
    {
        pdw3000local->init_xtrim = 0x2E ; //set default value
    }
    uwb_write8bitoffsetreg(XTAL_ID, 0, pdw3000local->init_xtrim);


    return SUCCESS ;

} // end dwt_initialise()


void uwb_setdwstate(int state)
{
    if (state == DWT_DW_IDLE) // Set the auto INIT2IDLE bit so that DW3000 enters IDLE mode before switching clocks to system_PLL
    //NOTE: PLL should be configured prior to this, and the device should be in IDLE_RC (if the PLL does not lock device will remain in IDLE_RC)
    {
        //switch clock to auto - if coming here from INIT_RC the clock will be FOSC/4, need to switch to auto prior to setting auto INIT2IDLE bit
        uwb_force_clocks(FORCE_CLK_AUTO);
        uwb_or8bitoffsetreg(SEQ_CTRL_ID, 0x01, SEQ_CTRL_AINIT2IDLE_BIT_MASK>>8);
    }
    else if(state == DWT_DW_IDLE_RC)  //Change state to IDLE_RC and clear auto INIT2IDLE bit
    {
        //switch clock to FOSC
        uwb_or8bitoffsetreg(CLK_CTRL_ID, 0, FORCE_SYSCLK_FOSC);
        //clear the auto INIT2IDLE bit and set FORCE2INIT
        uwb_modify32bitoffsetreg(SEQ_CTRL_ID, 0x0, (uint32_t) ~SEQ_CTRL_AINIT2IDLE_BIT_MASK, SEQ_CTRL_FORCE2INIT_BIT_MASK);
        //clear force bits (device will stay in IDLE_RC)
        uwb_and8bitoffsetreg(SEQ_CTRL_ID, 0x2, (uint8_t) ~(SEQ_CTRL_FORCE2INIT_BIT_MASK>>16));
        //switch clock to auto
        uwb_force_clocks(FORCE_CLK_AUTO);
    }
    else
    //NOTE: the SPI rate needs to be <= 7MHz as device is switching to INIT_RC state
    {
        uwb_or8bitoffsetreg(CLK_CTRL_ID, 0, FORCE_SYSCLK_FOSCDIV4);
        //clear the auto INIT2IDLE bit and set FORCE2INIT
        uwb_modify32bitoffsetreg(SEQ_CTRL_ID, 0x0, (uint32_t) ~SEQ_CTRL_AINIT2IDLE_BIT_MASK, SEQ_CTRL_FORCE2INIT_BIT_MASK);
        uwb_and8bitoffsetreg(SEQ_CTRL_ID, 0x2, (uint8_t) ~(SEQ_CTRL_FORCE2INIT_BIT_MASK>>16));
    }
}

void uwb_enablegpioclocks(void)
{
    uwb_or32bitoffsetreg(CLK_CTRL_ID, 0, CLK_CTRL_GPIO_CLK_EN_BIT_MASK);
}


uint8_t uwb_otprevision(void)
{
    return pdw3000local->otprev ;
}

void uwb_setfinegraintxseq(int enable)
{
    if (enable)
    {
        uwb_write32bitoffsetreg(PWR_UP_TIMES_LO_ID, 2, PMSC_TXFINESEQ_ENABLE);
    }
    else
    {
        uwb_write32bitoffsetreg(PWR_UP_TIMES_LO_ID, 2, PMSC_TXFINESEQ_DISABLE);
    }
}



void uwb_setlnapamode(int lna_pa)
{
    uint32_t gpio_mode = uwb_read32bitreg(GPIO_MODE_ID);
    gpio_mode &= (~(GPIO_MODE_MSGP0_MODE_BIT_MASK | GPIO_MODE_MSGP1_MODE_BIT_MASK
            | GPIO_MODE_MSGP4_MODE_BIT_MASK | GPIO_MODE_MSGP5_MODE_BIT_MASK | GPIO_MODE_MSGP6_MODE_BIT_MASK)); //clear GPIO 4, 5, 6, configuration
    if (lna_pa & DWT_LNA_ENABLE)
    {
        gpio_mode |= GPIO_PIN6_EXTRX;
    }
    if (lna_pa & DWT_PA_ENABLE)
    {
        gpio_mode |= (GPIO_PIN4_EXTDA | GPIO_PIN5_EXTTX);
    }
    if (lna_pa & DWT_TXRX_EN)
    {
        gpio_mode |= (GPIO_PIN0_EXTTXE | GPIO_PIN1_EXTRXE);
    }

    uwb_write32bitreg(GPIO_MODE_ID, gpio_mode);
}


uint8_t uwb_readpgdelay(void)
{
    return uwb_read8bitoffsetreg(TX_CTRL_HI_ID, 0);
}


uint8_t uwb_geticrefvolt(void)
{
    return pdw3000local->vBatP;
}

uint8_t uwb_geticreftemp(void)
{
    return pdw3000local->tempP;
}

uint32_t uwb_getpartid(void)
{
    return pdw3000local->partID;
}

uint32_t uwb_getlotid(void)
{
    return pdw3000local->lotID;
}


uint32_t uwb_readdevid(void)
{
    return uwb_read32bitoffsetreg(DEV_ID_ID, 0);
}


void uwb_configuretxrf(uwb_txconfig_t *config)
{
    if (config->PGcount == 0) {
        // Configure RF TX PG_DELAY
        uwb_write8bitoffsetreg(TX_CTRL_HI_ID, 0, config->PGdly);
    }
    else
    {
        uint8_t channel = 5;
        if (uwb_read8bitoffsetreg(CHAN_CTRL_ID, 0) & 0x1)
        {
            channel = 9;
        }
        uwb_calcbandwidthadj(config->PGcount, channel);
    }

    // Configure TX power
    uwb_write32bitreg(TX_POWER_ID, config->power);
}


void uwb_configurestskey(uwb_sts_cp_key_t* pStsKey)
{
    uwb_write32bitreg(STS_KEY0_ID, pStsKey->key0);
    uwb_write32bitreg(STS_KEY1_ID, pStsKey->key1);
    uwb_write32bitreg(STS_KEY2_ID, pStsKey->key2);
    uwb_write32bitreg(STS_KEY3_ID, pStsKey->key3);
}

void uwb_configurestsiv(uwb_sts_cp_iv_t* pStsIv)
{
    uwb_write32bitreg(STS_IV0_ID, pStsIv->iv0);
    uwb_write32bitreg(STS_IV1_ID, pStsIv->iv1);
    uwb_write32bitreg(STS_IV2_ID, pStsIv->iv2);
    uwb_write32bitreg(STS_IV3_ID, pStsIv->iv3);
}

void uwb_configurestsloadiv(void)
{
    uwb_or8bitoffsetreg(STS_CTRL_ID, 0, STS_CTRL_LOAD_IV_BIT_MASK);
}

static uint16_t get_sts_mnth(uint16_t cipher, uint8_t threshold, uint8_t shift_val)
{
    uint32_t  value;
    uint16_t  mod_val;

    value = cipher* (uint32_t)threshold;
    if (shift_val == 3)
    {
        value *= SQRT_FACTOR;//Factor to sqrt(2)
        value >>= SQRT_SHIFT_VAL;
    }

    mod_val = value % MOD_VALUE+ HALF_MOD;
    value >>= SHIFT_VALUE;
    /* Check if modulo greater than MOD_VALUE, if yes add 1 */
    if (mod_val >= MOD_VALUE)
        value += 1;

    return (uint16_t)value;
}




void uwb_configmrxlut(int channel)
{
	uint32_t lut0, lut1, lut2, lut3, lut4, lut5, lut6 = 0;

    if (channel == 5)
    {
        lut0 = (uint32_t)CH5_DGC_LUT_0;
        lut1 = (uint32_t)CH5_DGC_LUT_1;
        lut2 = (uint32_t)CH5_DGC_LUT_2;
        lut3 = (uint32_t)CH5_DGC_LUT_3;
        lut4 = (uint32_t)CH5_DGC_LUT_4;
        lut5 = (uint32_t)CH5_DGC_LUT_5;
        lut6 = (uint32_t)CH5_DGC_LUT_6;
    }
    else
    {
        lut0 = (uint32_t)CH9_DGC_LUT_0;
        lut1 = (uint32_t)CH9_DGC_LUT_1;
        lut2 = (uint32_t)CH9_DGC_LUT_2;
        lut3 = (uint32_t)CH9_DGC_LUT_3;
        lut4 = (uint32_t)CH9_DGC_LUT_4;
        lut5 = (uint32_t)CH9_DGC_LUT_5;
        lut6 = (uint32_t)CH9_DGC_LUT_6;
    }
    uwb_write32bitoffsetreg(DGC_LUT_0_CFG_ID, 0x0, lut0);
    uwb_write32bitoffsetreg(DGC_LUT_1_CFG_ID, 0x0, lut1);
    uwb_write32bitoffsetreg(DGC_LUT_2_CFG_ID, 0x0, lut2);
    uwb_write32bitoffsetreg(DGC_LUT_3_CFG_ID, 0x0, lut3);
    uwb_write32bitoffsetreg(DGC_LUT_4_CFG_ID, 0x0, lut4);
    uwb_write32bitoffsetreg(DGC_LUT_5_CFG_ID, 0x0, lut5);
    uwb_write32bitoffsetreg(DGC_LUT_6_CFG_ID, 0x0, lut6);
    uwb_write32bitoffsetreg(DGC_CFG0_ID, 0x0, DWT_DGC_CFG0);
    uwb_write32bitoffsetreg(DGC_CFG1_ID, 0x0, DWT_DGC_CFG1);
}

void uwb_restoreconfig(void)
{
    uint8_t channel = 5;
    uint16_t chan_ctrl;

    if (pdw3000local->bias_tune != 0)
    {
        _uwb_prog_ldo_and_bias_tune();
    }
    uwb_write8bitoffsetreg(LDO_RLOAD_ID, 1, LDO_RLOAD_VAL_B1);
    /*Restoring indirect access register B configuration as this is not preserved when device is in DEEPSLEEP/SLEEP state.
     * Indirect access register B is configured to point to the "Double buffer diagnostic SET 2"*/
    uwb_write32bitreg(INDIRECT_ADDR_B_ID, (BUF1_RX_FINFO >> 16));
    uwb_write32bitreg(ADDR_OFFSET_B_ID, BUF1_RX_FINFO & 0xffff);

    /* Restore OPS table configuration */
    _uwb_kick_ops_table_on_wakeup();

    chan_ctrl = uwb_read16bitoffsetreg(CHAN_CTRL_ID, 0);

    //assume RX code is the same as TX (e.g. we will not RX on 16 MHz or SCP and TX on 64 MHz)
    if( (((chan_ctrl>> CHAN_CTRL_TX_PCODE_BIT_OFFSET)&CHAN_CTRL_TX_PCODE_BIT_MASK) >= 9) && (((chan_ctrl >> CHAN_CTRL_TX_PCODE_BIT_OFFSET)&CHAN_CTRL_TX_PCODE_BIT_MASK) <= 24)) //only enable DGC for PRF 64
    {
        if (chan_ctrl & 0x1)
        {
            channel = 9;
        }

        /* If the OTP has DGC info programmed into it, do a manual kick from OTP. */
        if (pdw3000local->dgc_otp_set == DWT_DGC_LOAD_FROM_OTP)
        {
            _uwb_kick_dgc_on_wakeup(channel);
        }
        /* Else we manually program hard-coded values into the DGC registers. */
        else
        {
            uwb_configmrxlut(channel);
        }
    }
}



void uwb_configurestsmode(uint8_t stsMode)
{

    pdw3000local->stsconfig = stsMode;

    /////////////////////////////////////////////////////////////////////////
    //SYS_CFG
    //clear the PHR Mode, PHR Rate, STS Protocol, SDC, PDOA Mode,
    //then set the relevant bits according to configuration of the PHR Mode, PHR Rate, STS Protocol, SDC, PDOA Mode,
    uwb_modify32bitoffsetreg(SYS_CFG_ID, 0, ~(SYS_CFG_CP_SPC_BIT_MASK | SYS_CFG_CP_SDC_BIT_MASK),
        ((uint16_t)stsMode & DWT_STS_CONFIG_MASK) << SYS_CFG_CP_SPC_BIT_OFFSET);

    if((stsMode & DWT_STS_MODE_ND) == DWT_STS_MODE_ND)
    {
        //configure lower preamble detection threshold for no data STS mode
        uwb_write32bitoffsetreg(DTUNE3_ID, 0, PD_THRESH_NO_DATA);
    }
    else
    {
        uwb_write32bitoffsetreg(DTUNE3_ID, 0, PD_THRESH_DEFAULT);
    }
}

int uwb_configure(uwb_config_t *config)
{
    uint8_t chan = config->chan,cnt,flag;
    uint32_t temp;
    uint8_t scp = ((config->rxCode > 24) || (config->txCode > 24)) ? 1 : 0;
    uint8_t mode = (config->phrMode == DWT_PHRMODE_EXT) ? SYS_CFG_PHR_MODE_BIT_MASK : 0;
    uint16_t sts_len;
    int error = SUCCESS;
    int preamble_len;

    switch (config->txPreambLength)
    {
    case DWT_PLEN_32:
        preamble_len = 32;
        break;
    case DWT_PLEN_64:
        preamble_len = 64;
        break;
    case DWT_PLEN_72:
        preamble_len = 72;
        break;
    case DWT_PLEN_128:
        preamble_len = 128;
        break;
    default:
        preamble_len = 256;
        break;
    }

    pdw3000local->sleep_mode &= (~(DWT_ALT_OPS | DWT_SEL_OPS3));  //clear the sleep mode ALT_OPS bit
    pdw3000local->longFrames = config->phrMode ;
    sts_len=GET_STS_REG_SET_VALUE((uint16_t)(config->stsLength));
    pdw3000local->ststhreshold = (int16_t)((((uint32_t)sts_len) * 8) * STSQUAL_THRESH_64);
    pdw3000local->stsconfig = config->stsMode;


    /////////////////////////////////////////////////////////////////////////
    //SYS_CFG
    //clear the PHR Mode, PHR Rate, STS Protocol, SDC, PDOA Mode,
    //then set the relevant bits according to configuration of the PHR Mode, PHR Rate, STS Protocol, SDC, PDOA Mode,
    uwb_modify32bitoffsetreg(SYS_CFG_ID, 0, ~(SYS_CFG_PHR_MODE_BIT_MASK  |
                                              SYS_CFG_PHR_6M8_BIT_MASK   |
                                              SYS_CFG_CP_SPC_BIT_MASK    |
                                              SYS_CFG_PDOA_MODE_BIT_MASK |
                                              SYS_CFG_CP_SDC_BIT_MASK),
        ((uint32_t)config->pdoaMode) << SYS_CFG_PDOA_MODE_BIT_OFFSET
        | ((uint16_t)config->stsMode & DWT_STS_CONFIG_MASK) << SYS_CFG_CP_SPC_BIT_OFFSET
        | (SYS_CFG_PHR_6M8_BIT_MASK & ((uint32_t)config->phrRate << SYS_CFG_PHR_6M8_BIT_OFFSET))
        | mode);

     if (scp)
    {
        //configure OPS tables for SCP mode
        pdw3000local->sleep_mode |= DWT_ALT_OPS | DWT_SEL_OPS1;  //configure correct OPS table is kicked on wakeup
        uwb_modify32bitoffsetreg(OTP_CFG_ID, 0, ~(OTP_CFG_OPS_ID_BIT_MASK), DWT_OPSET_SCP | OTP_CFG_OPS_KICK_BIT_MASK);

        uwb_write32bitoffsetreg(IP_CONFIG_LO_ID, 0, IP_CONFIG_LO_SCP);       //Set this if Ipatov analysis is used in SCP mode
        uwb_write32bitoffsetreg(IP_CONFIG_HI_ID, 0, IP_CONFIG_HI_SCP);

        uwb_write32bitoffsetreg(STS_CONFIG_LO_ID, 0, STS_CONFIG_LO_SCP);
        uwb_write8bitoffsetreg(STS_CONFIG_HI_ID, 0, STS_CONFIG_HI_SCP);
    }
    else //
    {
        uint16_t sts_mnth;
        if (config->stsMode != DWT_STS_MODE_OFF)
        {

            //configure CIA STS lower bound
            if ((config->pdoaMode == DWT_PDOA_M1) || (config->pdoaMode == DWT_PDOA_M0))
            {
                //In PDOA mode 1, number of accumulated symbols is the whole length of the STS
                sts_mnth=get_sts_mnth(sts_length_factors[(uint8_t)(config->stsLength)], CIA_MANUALLOWERBOUND_TH_64, 3);
            }
            else
            {
                //In PDOA mode 3 number of accumulated symbols is half of the length of STS symbols
                sts_mnth=get_sts_mnth(sts_length_factors[(uint8_t)(config->stsLength)], CIA_MANUALLOWERBOUND_TH_64, 4);
            }

            preamble_len += (sts_len) * 8;

            uwb_modify16bitoffsetreg(STS_CONFIG_LO_ID, 2, (uint16_t)~(STS_CONFIG_LO_STS_MAN_TH_BIT_MASK >> 16), sts_mnth & 0x7F);

        }


        //configure OPS tables for non-SCP mode
        if (preamble_len >= 256)
        {
            pdw3000local->sleep_mode |= DWT_ALT_OPS | DWT_SEL_OPS0;
            uwb_modify32bitoffsetreg(OTP_CFG_ID, 0, ~(OTP_CFG_OPS_ID_BIT_MASK), DWT_OPSET_LONG | OTP_CFG_OPS_KICK_BIT_MASK);
        }
        else
        {
            uwb_modify32bitoffsetreg(OTP_CFG_ID, 0, ~(OTP_CFG_OPS_ID_BIT_MASK), DWT_OPSET_SHORT | OTP_CFG_OPS_KICK_BIT_MASK);
        }

    }
    uwb_modify8bitoffsetreg(DTUNE0_ID, 0, (uint8_t) ~DTUNE0_PRE_PAC_SYM_BIT_MASK, config->rxPAC);

    uwb_write8bitoffsetreg(STS_CFG0_ID, 0, sts_len-1);    /*Starts from 0 that is why -1*/

    if (config->txPreambLength == DWT_PLEN_72)
    {
        uwb_setplenfine(8); //value 8 sets fine preamble length to 72 symbols - this is needed to set 72 length.
    }
    else
    {
        uwb_setplenfine(0); //clear the setting in the FINE_PLEN register.
    }

    if((config->stsMode & DWT_STS_MODE_ND) == DWT_STS_MODE_ND)
    {
        //configure lower preamble detection threshold for no data STS mode
        uwb_write32bitoffsetreg(DTUNE3_ID, 0, PD_THRESH_NO_DATA);
    }
    else
    {
        //configure default preamble detection threshold for other modes
        uwb_write32bitoffsetreg(DTUNE3_ID, 0, PD_THRESH_DEFAULT);
    }

     /////////////////////////////////////////////////////////////////////////
    //CHAN_CTRL
    temp = uwb_read32bitoffsetreg(CHAN_CTRL_ID, 0);
    temp &= (~(CHAN_CTRL_RX_PCODE_BIT_MASK | CHAN_CTRL_TX_PCODE_BIT_MASK | CHAN_CTRL_SFD_TYPE_BIT_MASK | CHAN_CTRL_RF_CHAN_BIT_MASK));

    if (chan == 9) temp |= CHAN_CTRL_RF_CHAN_BIT_MASK;

    temp |= (CHAN_CTRL_RX_PCODE_BIT_MASK & ((uint32_t)config->rxCode << CHAN_CTRL_RX_PCODE_BIT_OFFSET));
    temp |= (CHAN_CTRL_TX_PCODE_BIT_MASK & ((uint32_t)config->txCode << CHAN_CTRL_TX_PCODE_BIT_OFFSET));
    temp |= (CHAN_CTRL_SFD_TYPE_BIT_MASK & ((uint32_t)config->sfdType << CHAN_CTRL_SFD_TYPE_BIT_OFFSET));

    uwb_write32bitoffsetreg(CHAN_CTRL_ID, 0, temp);

    /////////////////////////////////////////////////////////////////////////
    //TX_FCTRL
    // Set up TX Preamble Size, PRF and Data Rate
    uwb_modify32bitoffsetreg(TX_FCTRL_ID, 0, ~(TX_FCTRL_TXBR_BIT_MASK | TX_FCTRL_TXPSR_BIT_MASK),
                                              ((uint32_t)config->dataRate << TX_FCTRL_TXBR_BIT_OFFSET)
                                              | ((uint32_t) config->txPreambLength) << TX_FCTRL_TXPSR_BIT_OFFSET);


    //DTUNE (SFD timeout)
    // Don't allow 0 - SFD timeout will always be enabled
    if (config->sfdTO == 0)
    {
        config->sfdTO = DWT_SFDTOC_DEF;
    }
    uwb_write16bitoffsetreg(DTUNE0_ID, 2, config->sfdTO);


    ///////////////////////
    // RF
    if (chan == 9)
    {
        // Setup TX analog for ch9
        uwb_write32bitoffsetreg(TX_CTRL_HI_ID, 0, RF_TXCTRL_CH9);
        uwb_write16bitoffsetreg(PLL_CFG_ID, 0, RF_PLL_CFG_CH9);
        // Setup RX analog for ch9
        uwb_write32bitoffsetreg(RX_CTRL_HI_ID, 0, RF_RXCTRL_CH9);
    }
    else
    {
        // Setup TX analog for ch5
        uwb_write32bitoffsetreg(TX_CTRL_HI_ID, 0, RF_TXCTRL_CH5);
        uwb_write16bitoffsetreg(PLL_CFG_ID, 0, RF_PLL_CFG_CH5);
    }


    uwb_write8bitoffsetreg(LDO_RLOAD_ID, 1, LDO_RLOAD_VAL_B1);
    uwb_write8bitoffsetreg(TX_CTRL_LO_ID, 2, RF_TXCTRL_LO_B2);
    uwb_write8bitoffsetreg(PLL_CAL_ID, 0, RF_PLL_CFG_LD);        // Extend the lock delay

    //Verify PLL lock bit is cleared
    uwb_write8bitoffsetreg(SYS_STATUS_ID, 0, SYS_STATUS_CP_LOCK_BIT_MASK);

    ///////////////////////
    // auto cal the PLL and change to IDLE_PLL state
    uwb_setdwstate(DWT_DW_IDLE);

    for (flag=1, cnt=0; cnt < MAX_RETRIES_FOR_PLL; cnt++)
    {
        usleep(20);
        if ((uwb_read8bitoffsetreg(SYS_STATUS_ID, 0) & SYS_STATUS_CP_LOCK_BIT_MASK))
        {
            /* PLL is locked */
            flag = 0;
            break;
        }
    }
    if (flag)
    {
        return ERROR;
    }

    if ((config->rxCode >= 9) && (config->rxCode <= 24)) //only enable DGC for PRF 64
    {
        //load RX LUTs
        /* If the OTP has DGC info programmed into it, do a manual kick from OTP. */
        if (pdw3000local->dgc_otp_set == DWT_DGC_LOAD_FROM_OTP)
        {
            _uwb_kick_dgc_on_wakeup(chan);
        }
        /* Else we manually program hard-coded values into the DGC registers. */
        else
        {
            uwb_configmrxlut(chan);
        }
        uwb_modify16bitoffsetreg(DGC_CFG_ID, 0x0, (uint16_t)~DGC_CFG_THR_64_BIT_MASK, DWT_DGC_CFG << DGC_CFG_THR_64_BIT_OFFSET);
    }
    else
    {
        uwb_and8bitoffsetreg(DGC_CFG_ID, 0x0, (uint8_t)~DGC_CFG_RX_TUNE_EN_BIT_MASK);
    }

     ///////////////////
    // PGF
    error = uwb_pgf_cal(1);  //if the RX calibration routine fails the device receiver performance will be severely affected, the application should reset and try again

    return error;
} // end dwt_configure()



int uwb_pgf_cal(int ldoen)
{
    int temp;
    uint16_t val;

    //PGF needs LDOs turned on - ensure PGF LDOs are enabled
    if (ldoen == 1)
    {
        val = uwb_read16bitoffsetreg(LDO_CTRL_ID, 0);

        uwb_or16bitoffsetreg(LDO_CTRL_ID, 0, (
            LDO_CTRL_LDO_VDDIF2_EN_BIT_MASK |
            LDO_CTRL_LDO_VDDMS3_EN_BIT_MASK |
            LDO_CTRL_LDO_VDDMS1_EN_BIT_MASK));
    }

    //Run PGF Cal
    temp = uwb_run_pgfcal();

    //Turn off RX LDOs if previously off
    if (ldoen == 1)
    {
        uwb_and16bitoffsetreg(LDO_CTRL_ID, 0, val); // restore LDO values
    }
    return temp;
}


int uwb_run_pgfcal(void)
{
    int result = SUCCESS;
    uint32_t    data;
    uint32_t    val = 0;
    uint8_t     cnt,flag;

    //put into cal mode
    //Turn on delay mode
    data = (((uint32_t)0x02) << RX_CAL_CFG_COMP_DLY_BIT_OFFSET) | (RX_CAL_CFG_CAL_MODE_BIT_MASK & 0x1);
    uwb_write32bitoffsetreg(RX_CAL_CFG_ID, 0x0, data);
    // Trigger PGF Cal
    uwb_or8bitoffsetreg(RX_CAL_CFG_ID, 0x0, RX_CAL_CFG_CAL_EN_BIT_MASK);

    for (flag=1,cnt=0;cnt<MAX_RETRIES_FOR_PGF;cnt++)
    {
        usleep(20);
        if(uwb_read8bitoffsetreg(RX_CAL_STS_ID, 0x0) == 1)
        {//PGF cal is complete
            flag=0;
            break;
        }
    }
    if (flag)
    {
        result = ERROR;
    }

    // Put into normal mode
    uwb_write8bitoffsetreg(RX_CAL_CFG_ID, 0x0, 0);
    uwb_write8bitoffsetreg(RX_CAL_STS_ID, 0x0, 1); //clear the status
    uwb_or8bitoffsetreg(RX_CAL_CFG_ID, 0x2, 0x1); //enable reading
    val = uwb_read32bitoffsetreg(RX_CAL_RESI_ID, 0x0);
    if (val == ERR_RX_CAL_FAIL)
    {
        //PGF I Cal Fail
        result = ERROR;
    }
    val = uwb_read32bitoffsetreg(RX_CAL_RESQ_ID, 0x0);
    if (val == ERR_RX_CAL_FAIL)
    {
        //PGF Q Cal Fail
        result = ERROR;
    }

    return result;
}


void uwb_setrxantennadelay(uint16_t rxDelay)
{
    // Set the RX antenna delay for auto TX timestamp adjustment
    uwb_write16bitoffsetreg(CIA_CONF_ID, 0, rxDelay);
}


void uwb_settxantennadelay(uint16_t txDelay)
{
    // Set the TX antenna delay for auto TX timestamp adjustment
    uwb_write16bitoffsetreg(TX_ANTD_ID, 0, txDelay);
}



int uwb_writetxdata(uint16_t txDataLength, uint8_t *txDataBytes, uint16_t txBufferOffset)
{

    if ((txBufferOffset + txDataLength) < TX_BUFFER_MAX_LEN)
    {
        if(txBufferOffset <= REG_DIRECT_OFFSET_MAX_LEN)
        {
            /* Directly write the data to the IC TX buffer */
            uwb_writetodevice(TX_BUFFER_ID, txBufferOffset, txDataLength, txDataBytes);
        }
        else
        {
            /* Program the indirect offset register A for specified offset to TX buffer */
            uwb_write32bitreg(INDIRECT_ADDR_A_ID, (TX_BUFFER_ID >> 16) );
            uwb_write32bitreg(ADDR_OFFSET_A_ID,   txBufferOffset);

            /* Indirectly write the data to the IC TX buffer */
            uwb_writetodevice(INDIRECT_POINTER_A_ID, 0, txDataLength, txDataBytes);
        }
        return SUCCESS;
    }
    else
        return ERROR;
} // end dwt_writetxdata()




void uwb_write8bitoffsetreg(int regFileID, int regOffset, uint8_t regval)
{
	uwb_writetodevice(regFileID, regOffset, 1, &regval);
}

void uwb_writetodevice(uint32_t regFileID,uint16_t index,uint16_t length,uint8_t *buffer)
{
	uwb_xfer3000(regFileID, index, length, buffer, DW3000_SPI_WR_BIT);
}

void uwb_readfromdevice(uint32_t regFileID, uint16_t index, uint16_t length, uint8_t *buffer)
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
        return ERROR;
    }
    return SUCCESS;
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

uint8_t uwb_generatecrc8(const uint8_t* byteArray, int len, uint8_t crcRemainderInit)
{
    uint8_t data;
    int byte;
    /*
    * Divide the message by the polynomial, a byte at a time.
    */
    for (byte = 0; byte < len; ++byte)
    {
        data = byteArray[byte] ^ crcRemainderInit;
        crcRemainderInit = crcTable[data];// ^ (crcRemainderInit << 8);
    }
    /*
    * The final remainder is the CRC.
    */
    return crcRemainderInit;
}
