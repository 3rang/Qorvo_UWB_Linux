



#ifndef _UWB_DEVICE_API_H_
#define _UWB_DEVICE_API_H_

#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>
#include <stddef.h>


#define DW3000_DRIVER_VERSION               0x040000
#define DW3000_DEVICE_DRIVER_VER_STRING     "DW3000 C0 Device Driver Version 04.00.00"



/* */
#define DWT_SUCCESS (0)
#define DWT_ERROR   (-1)

#define DWT_TIME_UNITS      (1.0/499.2e6/128.0) //!< = 15.65e-12 s

#define DWT_A0_DEV_ID       (0xDECA0300)        //!< DW3000 MPW A0 (non PDOA) silicon device ID
#define DWT_A0_PDOA_DEV_ID  (0xDECA0310)        //!< DW3000 MPW A0 (with PDOA) silicon device ID
#define DWT_B0_DEV_ID       (0xDECA0301)        //!< DW3000 MPW B0 (non PDOA) silicon device ID
#define DWT_B0_PDOA_DEV_ID  (0xDECA0311)        //!< DW3000 MPW B0 (with PDOA) silicon device ID
#define DWT_C0_DEV_ID       (0xDECA0302)        //!< DW3000 MPW C0 (non PDOA) silicon device ID
#define DWT_C0_PDOA_DEV_ID  (0xDECA0312)        //!< DW3000 MPW C0 (with PDOA) silicon device ID

#define DELAY_20uUSec           (20)            /* Delay of 20uSec(measured 24uSec) */
#define MAX_RETRIES_FOR_PLL     (6)
#define MAX_RETRIES_FOR_PGF     (3)

typedef enum {
    AOA,
    NON_AOA
} dw3000type_e;

typedef enum {
    DW3000_SPI_RD_BIT    = 0x0000U,
    DW3000_SPI_WR_BIT    = 0x8000U,
    DW3000_SPI_AND_OR_8  = 0x8001U,
    DW3000_SPI_AND_OR_16 = 0x8002U,
    DW3000_SPI_AND_OR_32 = 0x8003U,
} spi_modes_e;

//! fast commands
#define CMD_DB_TOGGLE     0x13   //!< Toggle double buffer pointer
#define CMD_CLR_IRQS      0x12   //!< Clear all events/clear interrupt
#define CMD_CCA_TX_W4R    0x11   //!< Check if channel clear prior to TX, enable RX when TX done
#define CMD_DTX_REF_W4R   0x10   //!< Start delayed TX (as DTX_REF below), enable RX when TX done
#define CMD_DTX_RS_W4R    0xF    //!< Start delayed TX (as DTX_RS below), enable RX when TX done
#define CMD_DTX_TS_W4R    0xE    //!< Start delayed TX (as DTX_TS below), enable RX when TX done
#define CMD_DTX_W4R       0xD    //!< Start delayed TX (as DTX below), enable RX when TX done
#define CMD_TX_W4R        0xC    //!< Start TX (as below), enable RX when TX done
#define CMD_CCA_TX        0xB    //!< Check if channel clear prior to TX
#define CMD_DRX_REF       0xA    //!< Enable RX @ time = DREF_TIME + DX_TIME
#define CMD_DTX_REF       0x9    //!< Start delayed TX (RMARKER will be @ time = DREF_TIME + DX_TIME)
#define CMD_DRX_RS        0x8    //!< Enable RX @ time = RX_TIME + DX_TIME
#define CMD_DTX_RS        0x7    //!< Start delayed TX (RMARKER will be @ time = RX_TIME + DX_TIME)
#define CMD_DRX_TS        0x6    //!< Enable RX @ time = TX_TIME + DX_TIME
#define CMD_DTX_TS        0x5    //!< Start delayed TX (RMARKER will be @ time = TX_TIME + DX_TIME)
#define CMD_DRX           0x4    //!< Enable RX @ time specified in DX_TIME register
#define CMD_DTX           0x3    //!< Start delayed TX (RMARKER will be @ time set in DX_TIME register)
#define CMD_RX            0x2    //!< Enable RX
#define CMD_TX            0x1    //!< Start TX
#define CMD_TXRXOFF       0x0    //!< Turn off TX or RX, clear any TX/RX events and put DW3000 into IDLE

//! constants for selecting the bit rate for data TX (and RX)
//! These are defined for write (with just a shift) the TX_FCTRL register
#define DWT_BR_850K     0   //!< UWB bit rate 850 kbits/s
#define DWT_BR_6M8      1   //!< UWB bit rate 6.8 Mbits/s
#define DWT_BR_NODATA   2   //!< No data (SP3 packet format)

//! constants for specifying the (Nominal) mean Pulse Repetition Frequency
//! These are defined for direct write (with a shift if necessary) to CHAN_CTRL and TX_FCTRL regs
#define DWT_PRF_16M     1   //!< UWB PRF 16 MHz
#define DWT_PRF_64M     2   //!< UWB PRF 64 MHz
#define DWT_PRF_SCP     3   //!< SCP UWB PRF ~100 MHz

//! constants for specifying Preamble Acquisition Chunk (PAC) Size in symbols
#define DWT_PAC8        0   //!< PAC  8 (recommended for RX of preamble length  128 and below
#define DWT_PAC16       1   //!< PAC 16 (recommended for RX of preamble length  256
#define DWT_PAC32       2   //!< PAC 32 (recommended for RX of preamble length  512
#define DWT_PAC4        3   //!< PAC  4 (recommended for RX of preamble length  < 127

//! constants for specifying SFD Types and size
#define DWT_SFD_IEEE_4A 0   //!< IEEE 8-bit ternary
#define DWT_SFD_DW_8    1   //!< DW 8-bit
#define DWT_SFD_DW_16   2   //!< DW 16-bit
#define DWT_SFD_IEEE_4Z 3   //!< IEEE 8-bit binary (4z)
#define DWT_SFD_LEN8    (8) //!< IEEE, and DW 8-bit are length 8
#define DWT_SFD_LEN16   (16)//!< DW 16-bit is length 16

//! constants for specifying TX Preamble length in symbols
//! These are defined to allow them be directly written into byte 2 of the TX_FCTRL register
//! (i.e. a four bit value destined for bits 20..18 but shifted left by 2 for byte alignment)
#define DWT_PLEN_4096   0x03    //! Standard preamble length 4096 symbols
#define DWT_PLEN_2048   0x0A    //! Non-standard preamble length 2048 symbols
#define DWT_PLEN_1536   0x06    //! Non-standard preamble length 1536 symbols
#define DWT_PLEN_1024   0x02    //! Standard preamble length 1024 symbols
#define DWT_PLEN_512    0x0d    //! Non-standard preamble length 512 symbols
#define DWT_PLEN_256    0x09    //! Non-standard preamble length 256 symbols
#define DWT_PLEN_128    0x05    //! Non-standard preamble length 128 symbols
#define DWT_PLEN_64     0x01    //! Standard preamble length 64 symbols
#define DWT_PLEN_32     0x04    //! Non-standard length 32
#define DWT_PLEN_72     0x07    //! Non-standard length 72

#define DWT_SFDTOC_DEF          129  // default SFD timeout value

#define DWT_PHRMODE_STD         0x0     // standard PHR mode
#define DWT_PHRMODE_EXT         0x1     // DW proprietary extended frames PHR mode

#define DWT_PHRRATE_STD         0x0     // standard PHR rate
#define DWT_PHRRATE_DTA         0x1     // PHR at data rate (6M81)

// Define DW3000 PDOA modes
#define DWT_PDOA_M0           0x0     // DW PDOA mode is off
#define DWT_PDOA_M1           0x1     // DW PDOA mode  mode 1
#define DWT_PDOA_M3           0x3     // DW PDOA mode  mode 3

// Define DW3000 STS modes
#define DWT_STS_MODE_OFF         0x0     // STS is off
#define DWT_STS_MODE_1           0x1     // STS mode 1
#define DWT_STS_MODE_2           0x2     // STS mode 2
#define DWT_STS_MODE_ND          0x3     // STS with no data
#define DWT_STS_MODE_SDC         0x8     // Enable Super Deterministic Codes
#define DWT_STS_CONFIG_MASK      0xB


#define DWT_SFD_COUNT_WARN      (0x2000 >> 7)   // SFD count warning bit (STS quality status bit)

// Defined constants for "mode" bitmask parameter passed into dwt_starttx() function.
#define DWT_START_TX_IMMEDIATE      0x00    //! Send the frame immediately
#define DWT_START_TX_DELAYED        0x01    //! Send the frame at specified time (time must be less that half period away)
#define DWT_RESPONSE_EXPECTED       0x02    //! Will enable the receiver after TX has completed
#define DWT_START_TX_DLY_REF        0x04    //! Send the frame at specified time (time in DREF_TIME register + any time in DX_TIME register)
#define DWT_START_TX_DLY_RS         0x08    //! Send the frame at specified time (time in RX_TIME_0 register + any time in DX_TIME register)
#define DWT_START_TX_DLY_TS         0x10    //! Send the frame at specified time (time in TX_TIME_LO register + any time in DX_TIME register)

#define DWT_START_TX_CCA            0x20    //! Send the frame if no preamble detected within PTO time

// Defined constants for "mode" bitmask parameter passed into dwt_rxenable() function.
#define DWT_START_RX_IMMEDIATE      0x00    //! Enable the receiver immediately
#define DWT_START_RX_DELAYED        0x01    //! Set up delayed RX, if "late" error triggers, then the RX will be enabled immediately
#define DWT_IDLE_ON_DLY_ERR         0x02    //! If delayed RX failed due to "late" error then if this
                                            //! flag is set the RX will not be re-enabled immediately, and device will be in IDLE when function exits
#define DWT_START_RX_DLY_REF        0x04    //! Enable the receiver at specified time (time in DREF_TIME register + any time in DX_TIME register)
#define DWT_START_RX_DLY_RS         0x08    //! Enable the receiver at specified time (time in RX_TIME_0 register + any time in DX_TIME register)
#define DWT_START_RX_DLY_TS         0x10    //! Enable the receiver at specified time (time in TX_TIME_LO register + any time in DX_TIME register)

// Defined constants when SPI CRC mode is used:
typedef enum
{
    DWT_SPI_CRC_MODE_NO = 0,    /* No CRC */
    DWT_SPI_CRC_MODE_WR,        /* This is used to enable SPI CRC check (the SPI CRC check will be enabled on DW3000 and CRC-8 added for SPI write transactions) */
    DWT_SPI_CRC_MODE_WRRD       /* This is used to optionally enable additional CRC check on the SPI read operations, while the CRC check on the SPI write operations is also enabled */
}dwt_spi_crc_mode_e;


// Defined constants for "mode" bit field parameter passed to dwt_setleds() function.
#define DWT_LEDS_DISABLE     0x00
#define DWT_LEDS_ENABLE      0x01
#define DWT_LEDS_INIT_BLINK  0x02
// Default blink time. Blink time is expressed in multiples of 14 ms. The value defined here is ~225 ms.
#define DWT_LEDS_BLINK_TIME_DEF 0x10

#define GPIO_PIN2_RXLED         (((uint32_t)0x1)<<6)    /* The pin operates as the RXLED output */
#define GPIO_PIN3_TXLED         (((uint32_t)0x1)<<9)    /* The pin operates as the TXLED output */

#define GPIO_PIN0_EXTTXE        (((uint32_t)0x2)<<0)    /* The pin operates as the EXTTXE output (output TX state) */
#define GPIO_PIN1_EXTRXE        (((uint32_t)0x2)<<3)    /* The pin operates as the EXTRXE output (output RX state) */

#define GPIO_PIN4_EXTDA         (((uint32_t)0x1)<<12)   /* The pin operates to support external DA/PA */
#define GPIO_PIN5_EXTTX         (((uint32_t)0x1)<<15)   /* The pin operates to support external PA */
#define GPIO_PIN6_EXTRX         (((uint32_t)0x1)<<18)   /* The pin operates to support external LNA */

// Defined constants for "lna_pa" bit field parameter passed to dwt_setlnapamode() function
#define DWT_LNA_PA_DISABLE     0x00
#define DWT_LNA_ENABLE         0x01
#define DWT_PA_ENABLE          0x02
#define DWT_TXRX_EN            0x04

//Reset options
#define DWT_RESET_ALL          0x00
#define DWT_RESET_CTRX         0x0F
#define DWT_RESET_RX           0xEF
#define DWT_RESET_CLEAR        0xFF

//frame filtering configuration options
#define DWT_FF_ENABLE_802_15_4      0x2             // enable FF for 802.15.4
#define DWT_FF_DISABLE              0x0             // disable FF
#define DWT_FF_BEACON_EN            0x001           // beacon frames allowed
#define DWT_FF_DATA_EN              0x002           // data frames allowed
#define DWT_FF_ACK_EN               0x004           // ack frames allowed
#define DWT_FF_MAC_EN               0x008           // mac control frames allowed
#define DWT_FF_RSVD_EN              0x010           // reserved frame types allowed
#define DWT_FF_MULTI_EN             0x020           // multipurpose frames allowed
#define DWT_FF_FRAG_EN              0x040           // fragmented frame types allowed
#define DWT_FF_EXTEND_EN            0x080           // extended frame types allowed
#define DWT_FF_COORD_EN             0x100           // behave as coordinator (can receive frames with no dest address (PAN ID has to match))
#define DWT_FF_IMPBRCAST_EN         0x200           // allow MAC implicit broadcast
#define DWT_FF_MAC_LE0_EN           0x408           // mac frames allowed if address in LE0_PEND mathces source address
#define DWT_FF_MAC_LE1_EN           0x808           // mac frames allowed if address in LE1_PEND mathces source address
#define DWT_FF_MAC_LE2_EN           0x1008           // mac frames allowed if address in LE2_PEND mathces source address
#define DWT_FF_MAC_LE3_EN           0x2008           // mac frames allowed if address in LE3_PEND mathces source address


// SYS_STATE_LO register errors
#define DW_SYS_STATE_TXERR          0xD0000         // TSE is in TX but TX is in IDLE in SYS_STATE_LO register

//DW3000 interrupt events

#define DWT_INT_SCRC            0x00000004          // SPI write CRC error event
#define DWT_INT_TFRS            0x00000080          // frame sent
#define DWT_INT_LDED            0x00000400          // micro-code has finished execution
#define DWT_INT_RFCG            0x00004000          // frame received with good CRC
#define DWT_INT_RPHE            0x00001000          // receiver PHY header error
#define DWT_INT_RFCE            0x00008000          // receiver CRC error
#define DWT_INT_RFSL            0x00010000          // receiver sync loss error
#define DWT_INT_RFTO            0x00020000          // frame wait timeout
#define DWT_INT_LDEERR          0x00040000          // CIA error
#define DWT_INT_RXOVRR          0x00100000          // receiver overrun
#define DWT_INT_RXPTO           0x00200000          // preamble detect timeout
#define DWT_INT_LCSSERR         0x00400000          // LCSS set up error
#define DWT_INT_SFDT            0x04000000          // SFD timeout
#define DWT_INT_HPDWARN         0x08000000          // HPDWARN timeout
#define DWT_INT_CPERR           0x10000000          // STS Error
#define DWT_INT_ARFE            0x20000000          // frame rejected (due to frame filtering configuration)
#define DWT_INT_ALL             0x3FFFFFFF
#define DWT_INT_RX              (DWT_INT_LDED | DWT_INT_RFCG | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_RFTO | DWT_INT_LDEERR | DWT_INT_RXPTO | DWT_INT_SFDT | DWT_INT_ARFE)


//DW3000 SLEEP and WAKEUP configuration parameters
#define DWT_PGFCAL       0x0800
#define DWT_GOTORX       0x0200
#define DWT_GOTOIDLE     0x0100
#define DWT_SEL_OPS3     0x00C0
#define DWT_SEL_OPS2     0x0080                     // Short OPS table
#define DWT_SEL_OPS1     0x0040                     // SCP
#define DWT_SEL_OPS0     0x0000                     // Long OPS table
#define DWT_ALT_OPS      0x0020
#define DWT_LOADLDO      0x0010
#define DWT_LOADDGC      0x0008
#define DWT_LOADBIAS     0x0004
#define DWT_RUNSAR       0x0002
#define DWT_CONFIG       0x0001                     // download the AON array into the HIF (configuration download)

#define DWT_PRES_SLEEP   0x20                       // allows for SLEEP_EN bit to be "preserved", although it will self - clear on wake up
#define DWT_WAKE_WUP     0x10                       // wake up on WAKEUP PIN
#define DWT_WAKE_CSN     0x8                        // wake up on chip select
#define DWT_BROUT_EN     0x4                        // enable brownout detector during sleep/deep sleep
#define DWT_SLEEP        0x2                        // enable sleep (if this bit is clear the device will enter deep sleep)
#define DWT_SLP_EN       0x1                        // enable sleep/deep sleep functionality

//DW3000 IDLE/INIT mode definitions
#define DWT_DW_INIT      0x0
#define DWT_DW_IDLE      0x1
#define DWT_DW_IDLE_RC   0x2

#define DWT_READ_OTP_PID  0x10    //read part ID from OTP
#define DWT_READ_OTP_LID  0x20    //read lot ID from OTP
#define DWT_READ_OTP_BAT  0x40    //read ref voltage from OTP
#define DWT_READ_OTP_TMP  0x80    //read ref temperature from OTP

//DW3000 OTP operating parameter set selection
#define DWT_OPSET_LONG   (0x0<<11)
#define DWT_OPSET_SCP    (0x1<<11)
#define DWT_OPSET_SHORT  (0x2<<11)

//Conversion factor to convert clock offset from PPM to ratio
#define CLOCK_OFFSET_PPM_TO_RATIO (1.0/(1<<26))

#define AON_SLPCNT_LO (0x102) //address of SLEEP counter bits [19:12] in AON memory
#define AON_SLPCNT_HI (0x103) //address of SLEEP counter bits [27:20] in AON memory
#define AON_SLPCNT_CAL_CTRL (0x104) //address of SLEEP counter cal control
#define AON_SLPCNT_CAL_LO   (0x10E) //address of SLEEP counter cal value low byte
#define AON_SLPCNT_CAL_HI   (0x10F) //address of SLEEP counter cal value high byte

#define DW_CIA_DIAG_LOG_MAX (0x8)   //CIA to copy to swinging set a maximum set of diagnostic registers in Double Buffer mode
#define DW_CIA_DIAG_LOG_MID (0x4)   //CIA to copy to swinging set a medium set of diagnostic registers in Double Buffer mode
#define DW_CIA_DIAG_LOG_MIN (0x2)   //CIA to copy to swinging set a minimal set of diagnostic registers in Double Buffer mode
#define DW_CIA_DIAG_LOG_ALL (0x1)   //CIA to log all diagnostic registers
#define DW_CIA_DIAG_LOG_OFF (0x0)   //CIA to log reduced set of diagnostic registers

// Call-back data RX frames flags
#define DWT_CB_DATA_RX_FLAG_RNG  0x01 // Ranging bit
#define DWT_CB_DATA_RX_FLAG_ND   0x02 // No data mode
#define DWT_CB_DATA_RX_FLAG_CIA  0x04 // CIA done
#define DWT_CB_DATA_RX_FLAG_CER  0x08 // CIA error
#define DWT_CB_DATA_RX_FLAG_CPER 0x10 // CP error

// LDO and BIAS tune kick
#define LDO_BIAS_KICK (0x180)  // Writing to bit 7 and 8

// Multiplication factors to convert carrier integrator value to a frequency offset in Hertz
#define FREQ_OFFSET_MULTIPLIER          (998.4e6/2.0/1024.0/131072.0)

// Multiplication factors to convert frequency offset in Hertz to PPM crystal offset
// NB: also changes sign so a positive value means the local RX clock is running slower than the remote TX device.
#define HERTZ_TO_PPM_MULTIPLIER_CHAN_5     (-1.0e6/6489.6e6)
#define HERTZ_TO_PPM_MULTIPLIER_CHAN_9     (-1.0e6/7987.2e6)

// Low Energy (LE) device addresses
#define LE0 0 //LE0_PEND address
#define LE1 1 //LE1_PEND address
#define LE2 2 //LE2_PEND address
#define LE3 3 //LE3_PEND address

// TX/RX call-back data
typedef struct
{
    uint32_t status;      //initial value of register as ISR is entered
    uint16_t status_hi;   //initial value of register as ISR is entered, if relevant for that event type
    uint16_t datalength;  //length of frame
    uint8_t  rx_flags;    //RX frame flags, see above
} dwt_cb_data_t;

// Call-back type for SPI read error event (if the DW3000 generated CRC does not match the one calculated by the dwt_generatecrc8 function)
typedef void(*dwt_spierrcb_t)(void);

// Call-back type for all interrupt events
typedef void (*dwt_cb_t)(const dwt_cb_data_t *);


#define SQRT_FACTOR             181 /*Factor of sqrt(2) for calculation*/
#define STS_LEN_SUPPORTED       7   /*The supported STS length options*/
#define SQRT_SHIFT_VAL          7
#define SHIFT_VALUE             11
#define MOD_VALUE               2048
#define HALF_MOD                (MOD_VALUE>>1)

/*This Enum holds INT working options.*/
typedef enum
{
    DWT_DISABLE_INT=0,/*Disable these INT*/
    DWT_ENABLE_INT,/*Enable these INT*/
    DWT_ENABLE_INT_ONLY/*Enable only these INT*/
} dwt_INT_options_e;


/*This Enum holds the index for factor calculation.*/
typedef enum
{
    DWT_STS_LEN_32  =0,
    DWT_STS_LEN_64  =1,
    DWT_STS_LEN_128 =2,
    DWT_STS_LEN_256 =3,
    DWT_STS_LEN_512 =4,
    DWT_STS_LEN_1024=5,
    DWT_STS_LEN_2048=6
} dwt_sts_lengths_e;

#define GET_STS_REG_SET_VALUE(x)     ((uint16_t)1<<((x)+2))    /* Returns the value to set in CP_CFG0_ID for STS length. The x is the enum value from dwt_sts_lengths_e */

/* Enum used for selecting channel for DGC on-wake kick. */
typedef enum
{
    DWT_DGC_SEL_CH5=0,
    DWT_DGC_SEL_CH9
} dwt_dgc_chan_sel;

/* Enum used for selecting location to load DGC data from */
typedef enum
{
    DWT_DGC_LOAD_FROM_SW=0,
    DWT_DGC_LOAD_FROM_OTP
} dwt_dgc_load_location;

/*! ------------------------------------------------------------------------------------------------------------------
 * Structure typedef: dwt_config_t
 *
 * Structure for setting device configuration via dwt_configure() function
 *
 */
typedef struct
{
    uint8_t chan ;           //!< Channel number (5 or 9)
    uint8_t txPreambLength ; //!< DWT_PLEN_64..DWT_PLEN_4096
    uint8_t rxPAC ;          //!< Acquisition Chunk Size (Relates to RX preamble length)
    uint8_t txCode ;         //!< TX preamble code (the code configures the PRF, e.g. 9 -> PRF of 64 MHz)
    uint8_t rxCode ;         //!< RX preamble code (the code configures the PRF, e.g. 9 -> PRF of 64 MHz)
    uint8_t sfdType;         //!< SFD type (0 for short IEEE 8-bit standard, 1 for DW 8-bit, 2 for DW 16-bit, 3 for 4z BPRF)
    uint8_t dataRate ;       //!< Data rate {DWT_BR_850K or DWT_BR_6M8}
    uint8_t phrMode ;        //!< PHR mode {0x0 - standard DWT_PHRMODE_STD, 0x3 - extended frames DWT_PHRMODE_EXT}
    uint8_t phrRate;         //!< PHR rate {0x0 - standard DWT_PHRRATE_STD, 0x1 - at datarate DWT_PHRRATE_DTA}
    uint16_t sfdTO ;         //!< SFD timeout value (in symbols)
    uint8_t stsMode;         //!< STS mode (no STS, STS before PHR or STS after data)
    dwt_sts_lengths_e stsLength;    //!< STS length (the allowed values are listed in dwt_sts_lengths_e
    uint8_t pdoaMode;        //!< PDOA mode
} dwt_config_t ;


typedef struct
{
    uint8_t   PGdly;
    //TX POWER
    //31:24     TX_CP_PWR
    //23:16     TX_SHR_PWR
    //15:8      TX_PHR_PWR
    //7:0       TX_DATA_PWR
    uint32_t  power;
    uint16_t  PGcount;
} dwt_txconfig_t ;


typedef struct
{
    uint8_t       ipatovRxTime[5] ;   // RX timestamp from Ipatov sequence
    uint8_t       ipatovRxStatus ;    // RX status info for Ipatov sequence
    uint16_t      ipatovPOA ;         // POA of Ipatov

    uint8_t       stsRxTime[5] ;   // RX timestamp from STS
    uint16_t      stsRxStatus ;    // RX status info for STS
    uint16_t      stsPOA;          // POA of STS block 1
    uint8_t       sts2RxTime[5];   // RX timestamp from STS
    uint16_t      sts2RxStatus;    // RX status info for STS
    uint16_t      sts2POA;         // POA of STS block 2

    uint8_t       tdoa[6];            // TDOA from two STS RX timestamps
    int16_t       pdoa;               // PDOA from two STS POAs signed int [1:-11] in radians

    int16_t       xtalOffset ;        // estimated xtal offset of remote device
    uint32_t      ciaDiag1 ;          // Diagnostics common to both sequences

    uint32_t      ipatovPeak ;        // index and amplitude of peak sample in Ipatov sequence CIR
    uint32_t      ipatovPower ;       // channel area allows estimation of channel power for the Ipatov sequence
    uint32_t      ipatovF1 ;          // F1 for Ipatov sequence
    uint32_t      ipatovF2 ;          // F2 for Ipatov sequence
    uint32_t      ipatovF3 ;          // F3 for Ipatov sequence
    uint16_t      ipatovFpIndex ;     // First path index for Ipatov sequence
    uint16_t      ipatovAccumCount ;  // Number accumulated symbols for Ipatov sequence

    uint32_t      stsPeak ;        // index and amplitude of peak sample in STS CIR
    uint16_t      stsPower ;       // channel area allows estimation of channel power for the STS
    uint32_t      stsF1 ;          // F1 for STS
    uint32_t      stsF2 ;          // F2 for STS
    uint32_t      stsF3 ;          // F3 for STS
    uint16_t      stsFpIndex ;     // First path index for STS
    uint16_t      stsAccumCount ;  // Number accumulated symbols for STS

    uint32_t      sts2Peak;        // index and amplitude of peak sample in STS CIR
    uint16_t      sts2Power;       // channel area allows estimation of channel power for the STS
    uint32_t      sts2F1;          // F1 for STS
    uint32_t      sts2F2;          // F2 for STS
    uint32_t      sts2F3;          // F3 for STS
    uint16_t      sts2FpIndex;     // First path index for STS
    uint16_t      sts2AccumCount;  // Number accumulated symbols for STS

} dwt_rxdiag_t ;


typedef struct
{
    //all of the below are mapped to a register in DW3000
    uint16_t PHE;                    //12-bit number of received header error events
    uint16_t RSL;                    //12-bit number of received frame sync loss event events
    uint16_t CRCG;                   //12-bit number of good CRC received frame events
    uint16_t CRCB;                   //12-bit number of bad CRC (CRC error) received frame events
    uint8_t  ARFE;                   //8-bit number of address filter error events
    uint8_t  OVER;                   //8-bit number of receive buffer overrun events (used in double buffer mode)
    uint16_t SFDTO;                  //12-bit number of SFD timeout events
    uint16_t PTO;                    //12-bit number of Preamble timeout events
    uint8_t  RTO;                    //8-bit number of RX frame wait timeout events
    uint16_t TXF;                    //12-bit number of transmitted frame events
    uint8_t  HPW;                    //8-bit half period warning events (when delayed RX/TX enable is used)
    uint8_t  CRCE;                   //8-bit SPI CRC error events
    uint16_t PREJ;                   //12-bit number of Preamble rejection events

} dwt_deviceentcnts_t ;

/********************************************************************************************************************/
/*                                                AES BLOCK                                                         */
/********************************************************************************************************************/

//enums below are defined in such a way as to allow a simple write to DW3000 AES configuration registers

/* For MIC size definition */
typedef enum {
    MIC_0 = 0,
    MIC_4, MIC_6, MIC_8, MIC_10, MIC_12, MIC_14, MIC_16
}dwt_mic_size_e;

/* Key size definition */
typedef enum {
    AES_KEY_128bit = 0,
    AES_KEY_192bit = 1,
    AES_KEY_256bit = 2
}dwt_aes_key_size_e;

/* Load key from RAM selection */
typedef enum {
    AES_KEY_No_Load = 0,
    AES_KEY_Load
}dwt_aes_key_load_e;

/* Key source - RAM or registers */
typedef enum {
    AES_KEY_Src_Register = 0,    /* Use AES KEY from registers */
    AES_KEY_Src_RAMorOTP         /* Use AES KEY from RAM or OTP (depending if AES_key_OTP set),
                                    AES_KEY_Load needs to be set as well */
}dwt_aes_key_src_e;

/* Operation selection */
typedef enum {
    AES_Encrypt = 0,
    AES_Decrypt
}dwt_aes_mode_e;

/* This defines the source port for encrypted/unencrypted data */
typedef enum {
    AES_Src_Scratch = 0,
    AES_Src_Rx_buf_0,
    AES_Src_Rx_buf_1,
    AES_Src_Tx_buf
}dwt_aes_src_port_e;

/* This defines the dest port for encrypted/unencrypted data */
typedef enum {
    AES_Dst_Scratch = 0,
    AES_Dst_Rx_buf_0,
    AES_Dst_Rx_buf_1,
    AES_Dst_Tx_buf,
    AES_Dst_STS_key
}dwt_aes_dst_port_e;

/* storage for 128/192/256-bit key */
typedef struct {
      uint32_t key0;
      uint32_t key1;
      uint32_t key2;
      uint32_t key3;
      uint32_t key4;
      uint32_t key5;
      uint32_t key6;
      uint32_t key7;
}dwt_aes_key_t;

typedef enum
{
    AES_core_type_GCM=0,    /* Core type GCM */
    AES_core_type_CCM       /* Core type CCM */
}dwt_aes_core_type_e;


typedef enum
{
    AES_key_RAM =0,     /* Use the AES KEY from RAM */
    AES_key_OTP         /* Use the AES KEY from OTP, key_load needs to match -> needs to be set to AES_KEY_Src_Ram */
}dwt_aes_key_otp_type_e;

typedef struct {
    dwt_aes_key_otp_type_e  aes_key_otp_type; //!< Using KEY from OTP or RAM, if this is set to AES_key_OTP, KEY from OTP is used
    dwt_aes_core_type_e     aes_core_type;    //!< Core type GCM or CCM*
    dwt_mic_size_e          mic;              //!< Message integrity code size
    dwt_aes_key_src_e       key_src;          //!< Location of the key: either as programmed in registers(128 bit) or in the RAM or in the OTP
    dwt_aes_key_load_e      key_load;         //!< Loads key from RAM or uses KEY from the registers
    uint8_t                 key_addr;         //!< Address offset of AES key when using AES key in RAM
    dwt_aes_key_size_e      key_size;         //!< AES key length configuration corresponding to AES_KEY_128/192/256bit
    dwt_aes_mode_e          mode;             //!< Operation type encrypt/decrypt
} dwt_aes_config_t ;

typedef struct {
    uint8_t             *nonce;      //!< Pointer to the nonce
    uint8_t             *header;     //!< Pointer to header (this is not encrypted/decrypted)
    uint8_t             *payload;    //!< Pointer to payload (this is encrypted/decrypted)
    uint8_t             header_len;  //!< Header size
    uint16_t            payload_len; //!< Payload size
    dwt_aes_src_port_e  src_port;    //!< Source port
    dwt_aes_dst_port_e  dst_port;    //!< Dest port
    dwt_aes_mode_e      mode;        //!< Encryption or decryption
    uint8_t             mic_size;    //!< tag_size;
}dwt_aes_job_t;

/* storage for 128-bit STS CP key */
typedef struct {
    uint32_t key0;
    uint32_t key1;
    uint32_t key2;
    uint32_t key3;
}dwt_sts_cp_key_t;

/* storage for 128-bit STS CP IV (nonce) */
typedef struct {
    uint32_t iv0;
    uint32_t iv1;
    uint32_t iv2;
    uint32_t iv3;
}dwt_sts_cp_iv_t;


#define ERROR_DATA_SIZE      (-1)
#define ERROR_WRONG_MODE     (-2)
#define ERROR_WRONG_MIC_SIZE (-3)
#define ERROR_PAYLOAD_SIZE   (-4)
#define MIC_ERROR            (0xff)
#define STS_LEN_128BIT       (16)

typedef enum
{
    DBL_BUF_STATE_EN=0,/*Double buffer enabled*/
    DBL_BUF_STATE_DIS/*Double buffer disabled*/
}dwt_dbl_buff_state_e;

typedef enum
{
    DBL_BUF_MODE_AUTO=0,/*Automatic*/
    DBL_BUF_MODE_MAN/*Manual*/
}dwt_dbl_buff_mode_e;

/*
	Lookup table default values for channel 5
*/
typedef enum
{
    CH5_DGC_LUT_0 = 0x1c0fd,
    CH5_DGC_LUT_1 = 0x1c43e,
    CH5_DGC_LUT_2 = 0x1c6be,
    CH5_DGC_LUT_3 = 0x1c77e,
    CH5_DGC_LUT_4 = 0x1cf36,
    CH5_DGC_LUT_5 = 0x1cfb5,
    CH5_DGC_LUT_6 = 0x1cff5
} dwt_configmrxlut_ch5_e;

/*
	Lookup table default values for channel 9
*/
typedef enum
{
    CH9_DGC_LUT_0 = 0x2a8fe,
    CH9_DGC_LUT_1 = 0x2ac36,
    CH9_DGC_LUT_2 = 0x2a5fe,
    CH9_DGC_LUT_3 = 0x2af3e,
    CH9_DGC_LUT_4 = 0x2af7d,
    CH9_DGC_LUT_5 = 0x2afb5,
    CH9_DGC_LUT_6 = 0x2afb5
} dwt_configmrxlut_ch9_e;

#define DBL_BUFF_OFF             0x0
#define DBL_BUFF_ACCESS_BUFFER_0 0x1
#define DBL_BUFF_ACCESS_BUFFER_1 0x3

/********************************************************************************************************************/
/*                                                     API LIST                                                     */
/********************************************************************************************************************/














#ifdef __cplusplus
}
#endif

#endif /* _UWB_DEVICE_API_H_ */
