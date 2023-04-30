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

	app_main();
	//app_main(spi);

	spi_close(&spi);

	return 0;
}

