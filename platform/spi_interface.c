#include "spi_interface.h"
#include "uwb_device_api.h"
#include <string.h>

//#define uint8 unsigned char
//#define uint16 unsigned short
//#define uint32 unsigned int


void set_spi_speed_slow(void)
{
	set_spi_speed(&spi,1000000);
}
void set_spi_speed_fast(void)
{
	set_spi_speed(&spi,8000000);
}

int closespi(void)
{

}



// Calculate the CRC-16 using the CCITT standard
uint16_t dwt_calc_crc(const uint8_t* data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint8_t i;

    while (len--) {
        crc ^= *data++ << 8;

        for (i = 0; i < 8; ++i) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }

    return crc;
}





//int writetospiwithcrc(uint16_t headerLength, const uint8_t *headerBuffer, uint16_t bodylength, const uint8_t *bodyBuffer, uint8_t crc8
int writetospiwithcrc(uint16_t headerLength, const uint8_t *headerBuffer, uint32_t bodylength, const uint8_t *bodyBuffer)
{
    uint8_t txBuffer[headerLength + bodylength + 2]; // 2 bytes for the CRC
    uint8_t rxBuffer[headerLength + bodylength + 2]; // 2 bytes for the CRC

    memcpy(txBuffer, headerBuffer, headerLength);
    memcpy(txBuffer + headerLength, bodyBuffer, bodylength);

    uint16_t crc = dwt_calc_crc(txBuffer, headerLength + bodylength);
    txBuffer[headerLength + bodylength] = (crc & 0xFF);
    txBuffer[headerLength + bodylength + 1] = ((crc >> 8) & 0xFF);

    spi_transfer(&spi, txBuffer, rxBuffer, headerLength + bodylength + 2);

    return 0;
}
// Wrapper function for writetospi
int writetospi(uint16_t headerLength, const uint8_t *headerBuffer, uint16_t bodylength, const uint8_t *bodyBuffer)
{
    uint8_t txBuffer[headerLength + bodylength];
    uint8_t rxBuffer[headerLength + bodylength];

    memcpy(txBuffer, headerBuffer, headerLength);
    memcpy(txBuffer + headerLength, bodyBuffer, bodylength);

    spi_transfer(&spi, txBuffer, rxBuffer, headerLength + bodylength);

    return 0;
}

// Wrapper function for readfromspi
int readfromspi(uint16_t headerLength, const uint8_t *headerBuffer, uint16_t readlength, uint8_t *readBuffer)
{
    uint8_t txBuffer[headerLength];
    uint8_t rxBuffer[headerLength + readlength];

    memcpy(txBuffer, headerBuffer, headerLength);
    txBuffer[0] |= 0x80; // Set the read flag

    spi_transfer(&spi, txBuffer, rxBuffer, headerLength + readlength);

    memcpy(readBuffer, rxBuffer + headerLength, readlength);

    return 0;
}
