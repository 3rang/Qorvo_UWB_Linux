
#include "spi_interface.h"

unsigned char tx_buf [255];
unsigned char rx_buf [255];


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
    unsigned short len =  headerLength + bodyLength + sizeof(crc8);

    if (len > sizeof(tx_buf))
        return -1;

    memcpy(&tx_buf[0],            headerBuffer, headerLength);
    memcpy(&tx_buf[headerLength], bodyBuffer,   bodyLength);

    tx_buf[headerLength + bodyLength] = crc8;

    bufs[0].len = len;
    bufs[1].len = len;

    spi_transfer(spi, &tx_buf, &rx_buf,len);

    return 0;

}

int writetospi(unsigned short headerLength,const unsigned char *headerBuffer,unsigned short bodyLength,const unsigned char *bodyBuffer)
{
	 memcpy(&tx_buf[0], headerBuffer, headerLength);
    memcpy(&tx_buf[headerLength], bodyBuffer, bodyLength);

    bufs[0].len = headerLength + bodyLength;
    bufs[1].len = headerLength + bodyLength;

    spi_transfer(spi,&tx_buf, &rx_buf,sizeof(tx_buf);

    return 0;
}
int readfromspi(unsigned short headerLength,const unsigned char * headerBuffer,unsigned short readLength,unsigned char *readBuffer)
{
 memset(&tx_buf[0], 0, headerLength + readLength);
    memcpy(&tx_buf[0], headerBuffer, headerLength);

    bufs[0].len = headerLength + readLength;
    bufs[1].len = headerLength + readLength;

   spi_transfer(spi, &tx_buf, &rx_buf,sizeof(rx_buf));

     return 0;

}
