#ifndef SPI_H_
#define SPI_H_

#include <stddef.h>

typedef struct {
    int fd;
    unsigned char mode;
    unsigned char bits_per_word;
    unsigned int speed_hz;
} spi_t;

int spi_init(spi_t* spi, const char* device, unsigned char mode, unsigned char bits_per_word, unsigned int speed_hz);

int spi_transfer(spi_t* spi, const unsigned char* tx_data, unsigned char* rx_data, size_t len);

int spi_close(spi_t* spi);

#endif /* SPI_H_ */

