#include <stdio.h>
#include "deca_device_api.h"
#include "deca_regs.h"

int app_main()

{
int err;
err = dwt_check_dev_id();
    if (err == DWT_SUCCESS) {
        printf("DEV ID OK");
    }
    else {
        printf("DEV ID FAILED");
    }
}
