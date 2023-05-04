#include "spi.h"

spi_t spi;
const char *device = "/dev/spidev0.0";

void set_spi_speed_slow(void);

void set_spi_speed_fast(void);

int closespi(void);

//int writetospiwithcrc(uint16_t headerLength, const uint8_t *headerBuffer, uint32_t bodylength, const uint8_t *bodyBuffer)

//int writetospi(unsigned short headerLength,const unsigned char *headerBuffer,unsigned short bodyLength,const unsigned char *bodyBuffer);

//int readfromspi(unsigned short headerLength,const unsigned char * headerBuffer,unsigned short readLength,unsigned char *readBuffer);

