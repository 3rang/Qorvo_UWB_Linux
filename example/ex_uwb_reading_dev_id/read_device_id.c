#include <stdio.h>
#include "spi.h"

int app_main(spi_t spi)
{
	unsigned char tx_data[] = {0x00, 0x00, 0x00, 0x00, 0x00};
	unsigned char rx_data[5] = {0};



	if (spi_transfer(&spi, tx_data, rx_data, sizeof(tx_data)) < 0) {
		perror("Failed to perform SPI transaction");
		spi_close(&spi);
		return -1;
	}

	

	printf("%d \n",rx_data[0]); //= rx_data;
	printf("%d \n",rx_data[1]); //= rx_data;
	printf("%d \n",rx_data[2]); //= rx_data;
	printf("%d \n",rx_data[3]); //= rx_data;
	printf("%d \n",rx_data[4]); //= rx_data;

//	device_id = rx_data[0];
//	printf("%d\n",device_id); //= rx_data;

	if (rx_data[2] != 0) {
		printf("Device ID: %02x %02x %02x %02x %02x\n",
				rx_data[4], rx_data[3], rx_data[2], rx_data[1], rx_data[0]);
		printf("SPI communication test successful!\n");
	} else {
		printf("SPI communication test failed.\n");
	}


}
