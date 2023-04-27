#include "spi.h"

void set_spi_speed_slow(void);

void set_spi_speed_fast(void);

int closespi(void);

int writetospiwithcrc(unsigned short headerLength,const unsigned char *headerBuffer,unsigned short bodyLength,const unsigned char *bodyBuffer,unsigned char crc8);

int writetospi(unsigned short headerLength,const unsigned char *headerBuffer,unsigned short bodyLength,const unsigned char *bodyBuffer);

int readfromspi(unsigned short headerLength,const unsigned char * headerBuffer,unsigned short readLength,unsigned char *readBuffer);

