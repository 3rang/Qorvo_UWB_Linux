#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include "spi.h"

int spi_init(spi_t* spi, const char* device, unsigned char mode, unsigned char bits_per_word, unsigned int speed_hz) {
    int ret;

    spi->fd = open(device, O_RDWR);
    if (spi->fd < 0) {
        perror("can't open device");
        return -1;
    }

    spi->mode = mode;
    spi->bits_per_word = bits_per_word;
    spi->speed_hz = speed_hz;

    ret = ioctl(spi->fd, SPI_IOC_WR_MODE, &spi->mode);
    if (ret < 0) {
        perror("can't set SPI mode");
        return -1;
    }

    ret = ioctl(spi->fd, SPI_IOC_WR_BITS_PER_WORD, &spi->bits_per_word);
    if (ret < 0) {
        perror("can't set bits per word");
        return -1;
    }

    ret = ioctl(spi->fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi->speed_hz);
    if (ret < 0) {
        perror("can't set max speed hz");
        return -1;
    }

    return 0;
}

int spi_transfer(spi_t* spi, const unsigned char* tx_data, unsigned char* rx_data, size_t len) {
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_data,
        .rx_buf = (unsigned long)rx_data,
        .len = len,
        .delay_usecs = 0,
        .speed_hz = spi->speed_hz,
        .bits_per_word = spi->bits_per_word,
        .cs_change = 0,
    };

    int ret = ioctl(spi->fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1) {
        perror("can't send spi message");
        return -1;
    }

    return 0;
}

int spi_close(spi_t* spi) {
    int ret = close(spi->fd);
    if (ret < 0) {
        perror("can't close device");
        return -1;
    }

    return 0;
}

