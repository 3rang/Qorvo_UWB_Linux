#include "spi_interface.h"


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

int writetospiwithcrc(unsigned short headerLength,const unsigned char *headerBuffer,unsigned short bodyLength,const unsigned char *bodyBuffer,unsigned char crc8)
{

}

int writetospi(unsigned short headerLength,const unsigned char *headerBuffer,unsigned short bodyLength,const unsigned char *bodyBuffer)
{
}
int readfromspi(unsigned short headerLength,const unsigned char * headerBuffer,unsigned short readLength,unsigned char *readBuffer)
{
}
