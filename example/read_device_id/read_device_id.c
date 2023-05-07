#include <stdio.h>
#include "uwb_device_api.h"

int app_main()
{

	int32_t apiVersion = uwb_apiversion();

	// Print the API version
	printf("UWB Driver API Version: %x\n", apiVersion);


	uint32_t devId = uwb_readdevid();

	// Print the device ID
	printf("Device ID: 0x%08X\n", devId);

	// Cleanup and close your SPI interface

	return 0;
}
