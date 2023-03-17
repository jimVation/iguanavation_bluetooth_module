
#include "nrf_soc.h"

int32_t temperature_c_0_25_increments;

void temperature_read_core(void)
{
    sd_temp_get(&temperature_c_0_25_increments);
}
