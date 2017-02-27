#include <Wire.h>
#include <Arduino.h>
#include <CryptoAuthLib.h>

// Set to 1 to lock the data zone on success
#define LOCK_DATA 0

#define ECC108_I2C_ADDR 0x60 // 60h is factory default

// Some slot configuration
uint8_t slot_config[3][4] {
  {0x81, 0xA0, 0x81, 0xA0}, // slot 0 - private key,   slot 1 - private key
  {0x81, 0xA0, 0x81, 0xA0}, // slot 2 - private key,   slot 3 - private key
  {0x04, 0x04, 0x05, 0x05}  // slot 4 - user r/w data, slot 5 - user r/w data
};

// Device public keys
uint8_t public_keys[2][64];

// Test message to sign
const char *message = "Is this really me?";

// Intermediates for signing test
uint8_t digest_tmp[32];
uint8_t signature_tmp[64];

// Helper function to initialize library for a particular device.

bool config_device(ATCADeviceType dev, uint8_t addr, uint32_t baud = 400000, int retries = 25) {
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

// Print hex values (64 byte max)
void printHex(uint8_t *ptr, int len) {
  char hex[4];
  for (int i = 0; i < len; i++)  {
    sprintf(hex, "%02X", ptr[i]);
    Serial.print(hex);
  }
  Serial.println("");
}

// Provision and test ECC108A chip

void setup() {  
  int i, res;
  bool locked, verified;
  uint8_t cfg_word[4];
  uint8_t response[32];
  
  Serial.begin(9600);  
  Wire.begin();
  
  delay(15000); // a little time to open serial monitor please!

  Serial.println("Provision ECC108A");
  
  config_device(ATECC108A, ECC108_I2C_ADDR);
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
    // Create two signing key pairs
    for (i = 0; i < 2; i++) {
      res = atcab_genkey(i, public_keys[i]);
      if (res != ATCA_SUCCESS) {
        Serial.print("atcab_genkey error ");
        Serial.println(res, HEX);
        return;
      } else {
        Serial.print("Public key ");
        Serial.print(i);
        Serial.print(": ");
        printHex(public_keys[i], 64);
      }
    }
  } else {
    Serial.println("data already locked");
  }

  // Sign test message with key 0
  Serial.println("test keypair 0");
  res = atcab_sha(strlen(message), (const uint8_t *)message, digest_tmp);
  if (res != ATCA_SUCCESS) {
    Serial.print("atcab_sha error ");
    Serial.println(res, HEX);
    return;
  }
  Serial.print("message digest: ");
  printHex(digest_tmp, 32);

  res = atcab_sign(0 /* key # */, digest_tmp, signature_tmp);
  if (res != ATCA_SUCCESS) {
    Serial.print("atcab_sign error ");
    Serial.println(res, HEX);
    return;
  }
  Serial.print("message signature: ");
  printHex(signature_tmp, 64);

  Serial.print("public key: ");
  printHex(public_keys[0], 64);

  // Verify generated signature is good
  res = atcab_verify_extern(digest_tmp, signature_tmp, public_keys[0], &verified);
  if (res != ATCA_SUCCESS) {
    Serial.print("atcab_verify_extern error ");
    Serial.println(res, HEX);
    return;
  }
  if (verified) {
    Serial.println("TEST 1 PASSED!");
  } else {
    Serial.println("TEST 1 FAILED");
    return;
  }

  // Modify message and verify signature is NOT good
  digest_tmp[0] ^= 0x11;
  res = atcab_verify_extern(digest_tmp, signature_tmp, public_keys[0], &verified);
  if (res != ATCA_SUCCESS) {
    Serial.print("atcab_verify_extern error ");
    Serial.println(res, HEX);
    return;
  }
  if (!verified) {
    Serial.println("TEST 2 PASSED!");
  } else {
    Serial.println("TEST 2 FAILED");
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

