#pragma once

#include <Arduino.h>
#include <MeshCore.h>
#include <bluefruit.h>

#ifndef BLE_TX_POWER
#define BLE_TX_POWER 4
#endif

#ifndef RAW_LORA_BLE_QUEUE_SIZE
#define RAW_LORA_BLE_QUEUE_SIZE 6
#endif

#define RAW_LORA_BLE_FRAME_MAGIC      0x4C
#define RAW_LORA_BLE_FRAME_OVERHEAD   4
#define RAW_LORA_BLE_MAX_GATT_FRAME   (MAX_TRANS_UNIT + RAW_LORA_BLE_FRAME_OVERHEAD)

// BLE characteristic values use: magic, packet_id, total_len, offset, raw_chunk...

class RawLoRaBLEInterface {
  BLEDfu bledfu;
  BLEService raw_service;
  BLECharacteristic raw_rx_characteristic;
  BLECharacteristic raw_tx_characteristic;
  bool _isEnabled;
  bool _isDeviceConnected;
  uint16_t _conn_handle;
  unsigned long _last_health_check;
  unsigned long _last_retry_attempt;

  struct PacketFrame {
    uint16_t len;
    uint16_t offset;
    uint8_t id;
    uint8_t buf[MAX_TRANS_UNIT];
  };

  uint8_t notify_queue_len;
  PacketFrame notify_queue[RAW_LORA_BLE_QUEUE_SIZE];

  uint8_t recv_queue_len;
  PacketFrame recv_queue[RAW_LORA_BLE_QUEUE_SIZE];

  bool reassembly_active;
  uint8_t reassembly_id;
  uint16_t reassembly_total;
  uint16_t reassembly_received;
  uint8_t reassembly_buf[MAX_TRANS_UNIT];
  uint8_t next_notify_packet_id;

  void clearBuffers();
  void shiftNotifyQueueLeft();
  void shiftRecvQueueLeft();
  bool enqueueRecvPacket(const uint8_t* data, uint16_t len);
  bool isAdvertising() const;

  static void onConnect(uint16_t connection_handle);
  static void onDisconnect(uint16_t connection_handle, uint8_t reason);
  static bool onPairingPasskey(uint16_t connection_handle, uint8_t const passkey[6], bool match_request);
  static void onPairingComplete(uint16_t connection_handle, uint8_t auth_status);
  static void onRawTxWrite(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len);
  static void onRawRxCccdWrite(uint16_t conn_handle, BLECharacteristic* chr, uint16_t value);

public:
  RawLoRaBLEInterface();

  void begin(const char* prefix, char* name, uint32_t pin_code);
  void loop();
  void enable();
  void disable();
  void setAdvertisingEnabled(bool enabled);
  void disconnect();
  bool isEnabled() const { return _isEnabled; }
  bool isConnected() const;
  bool isListening();
  bool isWriteBusy() const;
  bool notifyRawPacket(const uint8_t raw[], uint16_t len);
  size_t checkRecvRawPacket(uint8_t dest[], size_t max_len);
};

#if BLE_DEBUG_LOGGING && ARDUINO
  #include <Arduino.h>
  #define RAW_LORA_BLE_DEBUG_PRINT(F, ...) Serial.printf("RAW-BLE: " F, ##__VA_ARGS__)
  #define RAW_LORA_BLE_DEBUG_PRINTLN(F, ...) Serial.printf("RAW-BLE: " F "\n", ##__VA_ARGS__)
#else
  #define RAW_LORA_BLE_DEBUG_PRINT(...) {}
  #define RAW_LORA_BLE_DEBUG_PRINTLN(...) {}
#endif
