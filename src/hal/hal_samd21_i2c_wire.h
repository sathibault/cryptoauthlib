#ifndef HAL_SAMD21_I2C_WIRE_H_
#define HAL_SAMD21_I2C_WIRE_H_

#define MAX_I2C_BUSES    1

typedef struct atcaI2Cmaster {
	int ref_ct;
	int bus_index;
} ATCAI2CMaster_t;

void change_i2c_speed( ATCAIface iface, uint32_t speed );

/** @} */
#endif /* HAL_SAMD21_I2C_WIRE_H_ */
