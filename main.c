#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include "spi_interface.h"
       

int main()
{


	if (spi_init(&spi, device, SPI_MODE_0, 8, 1000000) < 0) {
		perror("Failed to initialize SPI device");
		return -1;
	}

	/*    if (spi_transfer(&spi, tx_data, rx_data, sizeof(tx_data)) < 0) {
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
	      */
	app_main(spi);

	spi_close(&spi);


	return 0;
}

