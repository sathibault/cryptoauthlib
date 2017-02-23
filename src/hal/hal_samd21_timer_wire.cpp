#include <Arduino.h>
#include "atca_hal.h"

extern "C" void atca_delay_ms(uint32_t ms)
{
	delay(ms);
}

extern "C" void atca_delay_us(uint32_t us)
{
	delayMicroseconds(us);
}
