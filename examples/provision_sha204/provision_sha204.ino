#include <Wire.h>
#include <Arduino.h>
#include <CryptoAuthLib.h>

// Set to 1 to lock the data zone on success
#define LOCK_DATA 0

// Some slot configuration
uint8_t slot_config[3][4] {
  {0x80, 0x40, 0x81, 0x41}, // slot 0 - private key,        slot 1 - private key
  {0x82, 0x30, 0x83, 0x31}, // slot 2 - derived key from 0, slot 3 - derived key from 1
  {0x04, 0x04, 0x05, 0x05}  // slot 4 - user r/w data,      slot 5 - user r/w data
};

// Private root keys.  REPLACE THESE WITH SECRET KEYS
uint8_t root_keys[2][32] = {
  {0x08,0xe6,0xe6,0x93,0x69,0x4a,0x4c,0xd7,0x84,0xf2,0x09,0xb6,0x74,0x6d,0xea,0x1e,0x8e,0x1d,0xa0,0xa8,0x21,0xaa,0x4f,0x59,0xb0,0x63,0xe7,0x80,0x93,0x69,0x2f,0x7c},
  {0x16,0x90,0x16,0x64,0x1b,0xe8,0x48,0x10,0x96,0xe2,0x94,0x18,0x1b,0x34,0x72,0x6d,0x6f,0x25,0xc4,0xac,0xa3,0x3a,0x4e,0xdc,0xa0,0xb3,0x8b,0xd6,0x43,0x16,0xf6,0x0c}
};

// A test challenge
uint8_t challenge[32] = {
  0x11,0x11,0x11,0x11,0x21,0x21,0x21,0x21,0x31,0x31,0x31,0x31,0x41,0x41,0x41,0x41,0xa2,0xa2,0xa2,0xa2,0xb2,0xb2,0xb2,0xb2,0xc2,0xc2,0xc2,0xc2,0xd2,0xd2,0xd2,0xd2
};

// The expected reponse for the sample root keys and test challenge above
uint8_t answers[2][32] = {
  {0x10,0x2a,0xd5,0xa1,0xfa,0xc1,0xfd,0x52,0xf6,0xac,0x02,0xb1,0xf7,0xd8,0x3d,0x86,0x31,0x25,0x0b,0x0c,0x9e,0xbd,0xd1,0x31,0xda,0x5e,0x59,0x36,0x4d,0xf3,0x10,0xb3},
  {0x76,0xa3,0x47,0x93,0xd0,0x9c,0x25,0xaf,0x91,0xed,0x49,0x3f,0x0b,0xaa,0xb6,0xb3,0x55,0x0c,0x51,0x0a,0x26,0x13,0x00,0xbb,0xbb,0x62,0x89,0xc7,0x37,0x2f,0x79,0x1a}
};

// Helper function to initialize library for a particular device.

bool config_device(ATCADeviceType dev, uint8_t addr, uint32_t baud = 400000, int retries = 10) {
  static ATCAIfaceCfg cfg;
  cfg.iface_type = ATCA_I2C_IFACE,
  cfg.devtype = dev;
  cfg.atcai2c.slave_address = addr << 1;
  cfg.atcai2c.baud = baud;
  cfg.atcai2c.bus = 0;
  cfg.wake_delay = (dev == ATECC108A) ? 800 : 2560;
  cfg.rx_retries = retries;
  if (atcab_init(&cfg) != ATCA_SUCCESS)
    return false;
  atcab_wakeup(); // may return error if device is already awake
  return true;
};

// Provision and test SHA204A chip

void setup() {  
  int i, res;
  bool locked;
  uint8_t cfg_word[4];
  uint8_t response[32];
  
  Serial.begin(9600);  
  Wire.begin();
  
  delay(15000); // a little time to open serial monitor please!

  Serial.println("Provision SHA204");
  
  config_device(ATSHA204A, 0x64);
  res = atcab_is_locked(LOCK_ZONE_CONFIG, &locked);
  if (res != ATCA_SUCCESS) {
    Serial.print("atcab_is_locked error ");
    Serial.println(res, HEX);
    return;
  }
  if (!locked) {
    // Not locked, configure slots 0-5 with our slot_config data.
    for (i = 0; i < 3; i++) {
      res = atcab_write_zone(ATCA_ZONE_CONFIG, 0, 0, 0x5+i, slot_config[i], 4);
      if (res != ATCA_SUCCESS) {
        Serial.print("atcab_write_zone error ");
        Serial.println(res, HEX);
        return;
      }
    }  
  } else {
    Serial.println("config already locked");
  }

  for (i = 0; i < 3; i++) {
    // Validate the configuration of slots 0-5
    res = atcab_read_zone(ATCA_ZONE_CONFIG, 0, 0, 0x5+i, cfg_word, 4);
    if (res != ATCA_SUCCESS) {
      Serial.print("atcab_read_zone error ");
      Serial.println(res, HEX);
      return;
    }
    Serial.print(0x5+i, HEX);
    Serial.print(": ");
    Serial.print(cfg_word[0], HEX);
    Serial.print(" ");
    Serial.print(cfg_word[1], HEX);
    Serial.print(" ");
    Serial.print(cfg_word[2], HEX);
    Serial.print(" ");
    Serial.println(cfg_word[3], HEX);
    if (memcmp(cfg_word, slot_config[i], 4) != 0) {
      Serial.println("read back error");
      return;
    }
  }
  Serial.println("configuration good");
  
  if (!locked) {
    // If the configuration is good, lock it down!
    res = atcab_lock_config_zone(response);
    if (res != ATCA_SUCCESS) {
      Serial.print("atcab_lock_config_zone error ");
      Serial.println(res, HEX);
      return;
    }
    if (response[0] != 0) {
      Serial.print("configuration lock failed ");
      Serial.println(response[0], HEX);
      return;
    }
    Serial.println("configuration successfully locked!");
  }

  res = atcab_is_locked(LOCK_ZONE_DATA, &locked);
  if (res != ATCA_SUCCESS) {
    Serial.print("atcab_is_locked error ");
    Serial.println(res, HEX);
    return;
  }
  if (!locked) {
    // Write our root keys to slots 0/1
    for (i = 0; i < 2; i++) {
      res = atcab_write_zone(ATCA_ZONE_DATA, i, 0, 0, root_keys[i], 32);
      if (res != ATCA_SUCCESS) {
        Serial.print("atcab_write_zone error ");
        Serial.println(res, HEX);
        return;
      }
    }  
  } else {
    Serial.println("data already locked");
  }

  // Run test with slot 0 key
  Serial.println("test root key 0");
  atcab_mac(0x00, 0x0, challenge, response);
  if (res != ATCA_SUCCESS) {
    Serial.print("atcab_mac error ");
    Serial.println(res, HEX);
    return;
  }
  if (memcmp(response, answers[0], 32) == 0) {
    Serial.println("TEST 1 PASSED!");
  } else {
    Serial.print("TEST 1 FAILED:");
    for (i = 0; i < 32; i++) {
      Serial.print(" ");
      Serial.print(response[i], HEX);
    }
    Serial.println("");
    return;
  }

  // Run test with slot 1 key
  Serial.println("test root key 1");
  atcab_mac(0x00, 0x1, challenge, response);
  if (res != ATCA_SUCCESS) {
    Serial.print("atcab_mac error ");
    Serial.println(res, HEX);
    return;
  }
  if (memcmp(response, answers[1], 32) == 0) {
    Serial.println("TEST 2 PASSED!");
  } else {
    Serial.print("TEST 2 FAILED:");
    for (i = 0; i < 32; i++) {
      Serial.print(" ");
      Serial.print(response[i], HEX);
    }
    Serial.println("");
    return;
  }

#if LOCK_DATA
  res = atcab_lock_data_zone(response);
  if (res != ATCA_SUCCESS) {
    Serial.print("atcab_lock_data_zone error ");
    Serial.println(res, HEX);
    return;
  }
  if (response[0] != 0) {
    Serial.print("data lock failed ");
    Serial.println(response[0], HEX);
    return;
  }
  Serial.println("DATA successfully locked!");
#else
  Serial.println("DON'T FORGET TO LOCK DATA");
#endif
}

void loop() {
  delay(1000);
}

