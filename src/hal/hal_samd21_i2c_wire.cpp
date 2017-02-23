#include <string.h>
#include <stdio.h>
#include <Wire.h>
#include <Arduino.h>

#include "atca_hal.h"
#include "atca_device.h"
#include "hal_samd21_i2c_wire.h"

#define DEBUG_LOG( x )

extern "C" ATCA_STATUS hal_i2c_init(void *hal, ATCAIfaceCfg *cfg);
extern "C" ATCA_STATUS hal_i2c_wake(ATCAIface iface);
extern "C" ATCA_STATUS hal_i2c_idle(ATCAIface iface);
extern "C" ATCA_STATUS hal_i2c_release(void *hal_data);


static bool wake_complete = false;

static uint8_t revs508[1][4] = { { 0x00, 0x00, 0x50, 0x00 } };
static uint8_t revs108[2][4] = { { 0x80, 0x00, 0x10, 0x01 },
				 { 0x00, 0x00, 0x10, 0x05 } };
static uint8_t revs204[3][4] = { { 0x00, 0x02, 0x00, 0x08 },
				 { 0x00, 0x02, 0x00, 0x09 },
				 { 0x00, 0x04, 0x05, 0x00 } };

extern "C" ATCA_STATUS hal_i2c_discover_devices(int busNum, ATCAIfaceCfg *cfg, int *found)
{
	ATCAIfaceCfg *head = cfg;
	uint8_t slaveAddress = 0x01;
	ATCADevice device;
	ATCAIface discoverIface;
	ATCACommand command;
	ATCAPacket packet;
	uint32_t execution_time;
	ATCA_STATUS status;
	int i;

	ATCAIfaceCfg discoverCfg;
	discoverCfg.iface_type				= ATCA_I2C_IFACE;
	discoverCfg.devtype				= ATECC508A;
	discoverCfg.wake_delay				= 800;
	discoverCfg.rx_retries				= 3;
	discoverCfg.atcai2c = {
	  .slave_address	= 0x07,
	  .bus			= busNum,
	  .baud			= 400000
	};

	ATCAHAL_t hal;

	hal_i2c_init( &hal, &discoverCfg );
	device = newATCADevice( &discoverCfg );
	discoverIface = atGetIFace( device );
	command = atGetCommands( device );

	*found = 0;
	for ( slaveAddress = 0x07; slaveAddress <= 0x78; slaveAddress++ ) {
		discoverCfg.atcai2c.slave_address = slaveAddress << 1;

		if ( hal_i2c_wake( discoverIface ) == ATCA_SUCCESS ) {
			(*found)++;
			memcpy( (uint8_t*)head, (uint8_t*)&discoverCfg, sizeof(ATCAIfaceCfg));
			head->devtype = ATCA_DEV_UNKNOWN;

			memset( packet.data, 0x00, sizeof(packet.data));

			atInfo( command, &packet );
			execution_time = atGetExecTime(command, CMD_INFO) + 1;

			if ( (status = atsend( discoverIface, (uint8_t*)&packet, packet.txsize )) != ATCA_SUCCESS ) {
				Serial.println("packet send error");
				continue;
			}

			atca_delay_ms(execution_time);

			if ( (status = atreceive( discoverIface, &(packet.data[0]), &(packet.rxsize) )) != ATCA_SUCCESS )
				continue;

			if ( (status = isATCAError(packet.data)) != ATCA_SUCCESS ) {
				Serial.println("command response error\r\n");
				continue;
			}

			for ( i = 0; i < (int)sizeof(revs508) / 4; i++ ) {
				if ( memcmp( &packet.data[1], &revs508[i], 4) == 0 ) {
					head->devtype = ATECC508A;
					break;
				}
			}

			for ( i = 0; i < (int)sizeof(revs204) / 4; i++ ) {
				if ( memcmp( &packet.data[1], &revs204[i], 4) == 0 ) {
					head->devtype = ATSHA204A;
					break;
				}
			}

			for ( i = 0; i < (int)sizeof(revs108) / 4; i++ ) {
				if ( memcmp( &packet.data[1], &revs108[i], 4) == 0 ) {
					head->devtype = ATECC108A;
					break;
				}
			}

			atca_delay_ms(15);
			head++;
		}

		hal_i2c_idle(discoverIface);
	}

	hal_i2c_release(&hal);

	return ATCA_SUCCESS;
}

extern "C" ATCA_STATUS hal_i2c_init(void *hal, ATCAIfaceCfg *cfg)
{
	return ATCA_SUCCESS;
}

extern "C" ATCA_STATUS hal_i2c_post_init(ATCAIface iface)
{
	return ATCA_SUCCESS;
}

extern "C" ATCA_STATUS hal_i2c_send(ATCAIface iface, uint8_t *txdata, int txlength)
{
  ATCAIfaceCfg *cfg = atgetifacecfg(iface);
  
  DEBUG_LOG( Serial.print("> "); )
  DEBUG_LOG( Serial.println(cfg->atcai2c.slave_address >> 1, HEX); )

  txdata[0] = 0x03;   // insert the Word Address Value, Command token
  txlength++;
  
  // txdata has ATCAPacket format
  Wire.beginTransmission(cfg->atcai2c.slave_address >> 1);
  for (int i = 0; i < txlength; i++)
    Wire.write(txdata[i]);
  Wire.endTransmission();
  
  return ATCA_SUCCESS;
}

extern "C" ATCA_STATUS hal_i2c_receive( ATCAIface iface, uint8_t *rxdata, uint16_t *rxlength)
{
  uint16_t i;
  ATCAIfaceCfg *cfg = atgetifacecfg(iface);
  
  DEBUG_LOG( Serial.print(cfg->atcai2c.slave_address >> 1, HEX); )
  DEBUG_LOG( Serial.print(" < "); )

  // Get length of response
  Wire.requestFrom(cfg->atcai2c.slave_address >> 1, 1);
  if (Wire.available() != 1) {
    Serial.println("no count");
    return ATCA_COMM_FAIL;
  }
  rxdata[0] = Wire.read();

  Wire.requestFrom(cfg->atcai2c.slave_address >> 1, rxdata[0]-1);
  if (Wire.available() != (rxdata[0]-1)) {
    Serial.println("no data");
    return ATCA_COMM_FAIL;
  }
  
  for (int i = 1; i < rxdata[0] && i < *rxlength; i++) {
    rxdata[i] = Wire.read();
    DEBUG_LOG( Serial.print(":"); )
    DEBUG_LOG( Serial.print(rxdata[i], HEX); )
  }
  DEBUG_LOG( Serial.println(); )
  
  return ATCA_SUCCESS;
}

extern "C" ATCA_STATUS hal_i2c_wake(ATCAIface iface)
{
  int status;
  ATCAIfaceCfg *cfg = atgetifacecfg(iface);
  int retries = cfg->rx_retries;
  uint8_t data[4], expected[4] = {0x04, 0x11, 0x33, 0x43};

  if (wake_complete)
    return ATCA_SUCCESS;

  DEBUG_LOG( Serial.print("wake "); )
  DEBUG_LOG( Serial.println(cfg->atcai2c.slave_address >> 1, HEX); )

  Wire.beginTransmission(0x00);
  Wire.endTransmission();
  
  atca_delay_us(cfg->wake_delay);
  
  while (Wire.requestFrom(cfg->atcai2c.slave_address >> 1, 4) == 0 &&
	 retries > 0)
    retries--;

  if (Wire.available() != 4) {
    Serial.print("invalid response ");
    Serial.println(Wire.available());
    return ATCA_COMM_FAIL;
  }
  
  wake_complete = true;

  for (int i = 0; i < 4; i++)
    data[i] = Wire.read();
  if (memcmp(data, expected, 4) == 0)
    return ATCA_SUCCESS;

  Serial.print("incorrect response ");
  for (int i = 0; i < 4; i++) {
    Serial.print(":");
    Serial.print(data[i], HEX);
  }
  Serial.println("");
  
  return ATCA_COMM_FAIL;
}

extern "C" ATCA_STATUS hal_i2c_idle(ATCAIface iface)
{
  ATCAIfaceCfg *cfg = atgetifacecfg(iface);
  
  DEBUG_LOG( Serial.print("idle "); )
  DEBUG_LOG( Serial.println(cfg->atcai2c.slave_address >> 1, HEX); )

  Wire.beginTransmission(cfg->atcai2c.slave_address >> 1);
  Wire.write(0x02); // idle word address value
  Wire.endTransmission();
  wake_complete = false;
  
  return ATCA_SUCCESS;
}

extern "C" ATCA_STATUS hal_i2c_sleep(ATCAIface iface)
{
  ATCAIfaceCfg *cfg = atgetifacecfg(iface);

  DEBUG_LOG( Serial.print("sleep "); )
  DEBUG_LOG( Serial.println(cfg->atcai2c.slave_address >> 1, HEX); )

  Wire.beginTransmission(cfg->atcai2c.slave_address >> 1);
  Wire.write(0x01); // sleep word address value
  Wire.endTransmission();
  wake_complete = false;
  
  return ATCA_SUCCESS;
}

extern "C" ATCA_STATUS hal_i2c_release( void *hal_data )
{
  return ATCA_SUCCESS;
}
