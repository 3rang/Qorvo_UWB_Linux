#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include "spi.h"

int main()
{
    spi_t spi;
    const char *device = "/dev/spidev0.0";
    unsigned char tx_data[] = {0x00, 0x00, 0x00, 0x00, 0x00};
    unsigned char rx_data[5] = {0};
    unsigned char *device_id;

    if (spi_init(&spi, device, SPI_MODE_0, 8, 1000000) < 0) {
        perror("Failed to initialize SPI device");
        return -1;
    }

    if (spi_transfer(&spi, tx_data, rx_data, sizeof(tx_data)) < 0) {
        perror("Failed to perform SPI transaction");
        spi_close(&spi);
        return -1;
    }

    device_id = rx_data;

    if (device_id != 0) {
        printf("Device ID: %02x %02x %02x %02x %02x\n",
               rx_data[4], rx_data[3], rx_data[2], rx_data[1], rx_data[0]);
        printf("SPI communication test successful!\n");
    } else {
        printf("SPI communication test failed.\n");
    }

    spi_close(&spi);

    return 0;
}

