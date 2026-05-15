#include "RawLoRaBLEInterface.h"

#define RAW_LORA_BLE_SERVICE_UUID "7A3E0001-6D2F-4A99-A45E-7B15C5E3D001"
#define RAW_LORA_BLE_RX_UUID      "7A3E0002-6D2F-4A99-A45E-7B15C5E3D001"
#define RAW_LORA_BLE_TX_UUID      "7A3E0003-6D2F-4A99-A45E-7B15C5E3D001"

#define RAW_LORA_BLE_HEALTH_CHECK_INTERVAL 10000
#define RAW_LORA_BLE_RETRY_THROTTLE_MS     250
#define RAW_LORA_BLE_MIN_CONN_INTERVAL     12
#define RAW_LORA_BLE_MAX_CONN_INTERVAL     24
#define RAW_LORA_BLE_SLAVE_LATENCY         4
#define RAW_LORA_BLE_CONN_SUP_TIMEOUT      200
#define RAW_LORA_BLE_ADV_INTERVAL_MIN      32
#define RAW_LORA_BLE_ADV_INTERVAL_MAX      244
#define RAW_LORA_BLE_ADV_FAST_TIMEOUT      30

static RawLoRaBLEInterface* instance = nullptr;

RawLoRaBLEInterface::RawLoRaBLEInterface()
  : raw_service(RAW_LORA_BLE_SERVICE_UUID),
    raw_rx_characteristic(RAW_LORA_BLE_RX_UUID),
    raw_tx_characteristic(RAW_LORA_BLE_TX_UUID) {
  _isEnabled = false;
  _isDeviceConnected = false;
  _conn_handle = BLE_CONN_HANDLE_INVALID;
  _last_health_check = 0;
  _last_retry_attempt = 0;
  notify_queue_len = 0;
  recv_queue_len = 0;
  reassembly_active = false;
  reassembly_id = 0;
  reassembly_total = 0;
  reassembly_received = 0;
  next_notify_packet_id = 1;
}

void RawLoRaBLEInterface::onConnect(uint16_t connection_handle) {
  if (!instance) return;

  instance->_conn_handle = connection_handle;
  instance->_isDeviceConnected = true;
  instance->_last_health_check = millis();
  RAW_LORA_BLE_DEBUG_PRINTLN("connected handle=0x%04X", connection_handle);
}

void RawLoRaBLEInterface::onDisconnect(uint16_t connection_handle, uint8_t reason) {
  if (!instance) return;

  RAW_LORA_BLE_DEBUG_PRINTLN("disconnected handle=0x%04X reason=%u", connection_handle, reason);
  if (instance->_conn_handle == connection_handle) {
    instance->_conn_handle = BLE_CONN_HANDLE_INVALID;
    instance->_isDeviceConnected = false;
    instance->clearBuffers();
  }
}

bool RawLoRaBLEInterface::onPairingPasskey(uint16_t connection_handle, uint8_t const passkey[6], bool match_request) {
  (void)connection_handle;
  (void)passkey;
  (void)match_request;
  return true;
}

void RawLoRaBLEInterface::onPairingComplete(uint16_t connection_handle, uint8_t auth_status) {
  RAW_LORA_BLE_DEBUG_PRINTLN("pairing complete handle=0x%04X status=%u", connection_handle, auth_status);
  if (!instance || instance->_conn_handle != connection_handle) return;

  if (auth_status != BLE_GAP_SEC_STATUS_SUCCESS) {
    sd_ble_gap_disconnect(connection_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
  }
}

void RawLoRaBLEInterface::onRawTxWrite(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  (void)chr;

  if (!instance || instance->_conn_handle != conn_handle || !instance->isConnected()) return;

  if (len < RAW_LORA_BLE_FRAME_OVERHEAD || data[0] != RAW_LORA_BLE_FRAME_MAGIC) {
    RAW_LORA_BLE_DEBUG_PRINTLN("invalid frame len=%u", len);
    return;
  }

  uint8_t packet_id = data[1];
  uint16_t total_len = data[2];
  uint16_t offset = data[3];
  uint16_t chunk_len = len - RAW_LORA_BLE_FRAME_OVERHEAD;

  if (total_len == 0 || total_len > MAX_TRANS_UNIT || chunk_len == 0 || offset + chunk_len > total_len) {
    RAW_LORA_BLE_DEBUG_PRINTLN("invalid framed write total=%u offset=%u chunk=%u", total_len, offset, chunk_len);
    instance->reassembly_active = false;
    return;
  }

  if (!instance->reassembly_active || instance->reassembly_id != packet_id) {
    instance->reassembly_active = true;
    instance->reassembly_id = packet_id;
    instance->reassembly_total = total_len;
    instance->reassembly_received = 0;
  }

  if (instance->reassembly_total != total_len || offset != instance->reassembly_received) {
    RAW_LORA_BLE_DEBUG_PRINTLN("out-of-order framed write id=%u offset=%u expected=%u", packet_id, offset, instance->reassembly_received);
    instance->reassembly_active = false;
    return;
  }

  memcpy(&instance->reassembly_buf[offset], data + RAW_LORA_BLE_FRAME_OVERHEAD, chunk_len);
  instance->reassembly_received += chunk_len;

  if (instance->reassembly_received >= instance->reassembly_total) {
    instance->enqueueRecvPacket(instance->reassembly_buf, instance->reassembly_total);
    instance->reassembly_active = false;
  }
}

void RawLoRaBLEInterface::onRawRxCccdWrite(uint16_t conn_handle, BLECharacteristic* chr, uint16_t value) {
  (void)chr;

  if (!instance || instance->_conn_handle != conn_handle) return;

  if ((value & BLE_GATT_HVX_NOTIFICATION) == 0) {
    instance->notify_queue_len = 0;
    instance->_last_retry_attempt = 0;
  }
}

void RawLoRaBLEInterface::begin(const char* prefix, char* name, uint32_t pin_code) {
  instance = this;

  char charpin[20];
  snprintf(charpin, sizeof(charpin), "%lu", (unsigned long)pin_code);

  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();

  char dev_name[32 + 16];
  if (strcmp(name, "@@MAC") == 0) {
    ble_gap_addr_t addr;
    if (sd_ble_gap_addr_get(&addr) == NRF_SUCCESS) {
      sprintf(name, "%02X%02X%02X%02X%02X%02X",
          addr.addr[5], addr.addr[4], addr.addr[3], addr.addr[2], addr.addr[1], addr.addr[0]);
    }
  }
  snprintf(dev_name, sizeof(dev_name), "%s%s", prefix, name);

  ble_gap_conn_params_t ppcp_params;
  ppcp_params.min_conn_interval = RAW_LORA_BLE_MIN_CONN_INTERVAL;
  ppcp_params.max_conn_interval = RAW_LORA_BLE_MAX_CONN_INTERVAL;
  ppcp_params.slave_latency = RAW_LORA_BLE_SLAVE_LATENCY;
  ppcp_params.conn_sup_timeout = RAW_LORA_BLE_CONN_SUP_TIMEOUT;
  sd_ble_gap_ppcp_set(&ppcp_params);

  Bluefruit.setTxPower(BLE_TX_POWER);
  Bluefruit.setName(dev_name);

  Bluefruit.Security.setMITM(true);
  Bluefruit.Security.setPIN(charpin);
  Bluefruit.Security.setIOCaps(true, false, false);
  Bluefruit.Security.setPairPasskeyCallback(onPairingPasskey);
  Bluefruit.Security.setPairCompleteCallback(onPairingComplete);

  Bluefruit.Periph.setConnectCallback(onConnect);
  Bluefruit.Periph.setDisconnectCallback(onDisconnect);

  raw_service.begin();

  raw_rx_characteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  raw_rx_characteristic.setPermission(SECMODE_ENC_WITH_MITM, SECMODE_NO_ACCESS);
  raw_rx_characteristic.setMaxLen(RAW_LORA_BLE_MAX_GATT_FRAME);
  raw_rx_characteristic.setCccdWriteCallback(onRawRxCccdWrite);
  raw_rx_characteristic.begin();

  raw_tx_characteristic.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  raw_tx_characteristic.setPermission(SECMODE_NO_ACCESS, SECMODE_ENC_WITH_MITM);
  raw_tx_characteristic.setMaxLen(RAW_LORA_BLE_MAX_GATT_FRAME);
  raw_tx_characteristic.setWriteCallback(onRawTxWrite);
  raw_tx_characteristic.begin();

  bledfu.setPermission(SECMODE_ENC_WITH_MITM, SECMODE_ENC_WITH_MITM);
  bledfu.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(raw_service);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.setInterval(RAW_LORA_BLE_ADV_INTERVAL_MIN, RAW_LORA_BLE_ADV_INTERVAL_MAX);
  Bluefruit.Advertising.setFastTimeout(RAW_LORA_BLE_ADV_FAST_TIMEOUT);
  Bluefruit.Advertising.restartOnDisconnect(true);
}

void RawLoRaBLEInterface::clearBuffers() {
  notify_queue_len = 0;
  recv_queue_len = 0;
  reassembly_active = false;
  reassembly_received = 0;
  _last_retry_attempt = 0;
}

void RawLoRaBLEInterface::shiftNotifyQueueLeft() {
  if (notify_queue_len == 0) return;

  notify_queue_len--;
  for (uint8_t i = 0; i < notify_queue_len; i++) {
    notify_queue[i] = notify_queue[i + 1];
  }
}

void RawLoRaBLEInterface::shiftRecvQueueLeft() {
  if (recv_queue_len == 0) return;

  recv_queue_len--;
  for (uint8_t i = 0; i < recv_queue_len; i++) {
    recv_queue[i] = recv_queue[i + 1];
  }
}

bool RawLoRaBLEInterface::isAdvertising() const {
  ble_gap_addr_t adv_addr;
  return sd_ble_gap_adv_addr_get(0, &adv_addr) == NRF_SUCCESS;
}

bool RawLoRaBLEInterface::enqueueRecvPacket(const uint8_t* data, uint16_t len) {
  if (len == 0 || len > MAX_TRANS_UNIT) return false;
  if (recv_queue_len >= RAW_LORA_BLE_QUEUE_SIZE) {
    RAW_LORA_BLE_DEBUG_PRINTLN("recv queue full, dropping packet");
    return false;
  }

  PacketFrame& frame = recv_queue[recv_queue_len++];
  frame.len = len;
  frame.offset = 0;
  frame.id = 0;
  memcpy(frame.buf, data, len);
  return true;
}

void RawLoRaBLEInterface::enable() {
  setAdvertisingEnabled(true);
}

void RawLoRaBLEInterface::setAdvertisingEnabled(bool enabled) {
  if (enabled == _isEnabled) return;

  _isEnabled = enabled;
  _last_health_check = millis();

  if (enabled) {
    Bluefruit.Advertising.restartOnDisconnect(true);
    if (!isConnected()) {
      Bluefruit.Advertising.start(0);
    }
  } else {
    Bluefruit.Advertising.restartOnDisconnect(false);
    Bluefruit.Advertising.stop();
    _last_health_check = 0;
  }
}

void RawLoRaBLEInterface::disconnect() {
  if (_conn_handle != BLE_CONN_HANDLE_INVALID) {
    sd_ble_gap_disconnect(_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
  }
}

void RawLoRaBLEInterface::disable() {
  setAdvertisingEnabled(false);
  disconnect();
  clearBuffers();
}

bool RawLoRaBLEInterface::isConnected() const {
  return _isDeviceConnected && Bluefruit.connected() > 0;
}

bool RawLoRaBLEInterface::isListening() {
  return isConnected() && raw_rx_characteristic.notifyEnabled(_conn_handle);
}

bool RawLoRaBLEInterface::isWriteBusy() const {
  return notify_queue_len >= (RAW_LORA_BLE_QUEUE_SIZE * 2 / 3);
}

bool RawLoRaBLEInterface::notifyRawPacket(const uint8_t raw[], uint16_t len) {
  if (len == 0 || len > MAX_TRANS_UNIT || !isListening()) return false;
  if (notify_queue_len >= RAW_LORA_BLE_QUEUE_SIZE) {
    RAW_LORA_BLE_DEBUG_PRINTLN("notify queue full, dropping packet");
    return false;
  }

  PacketFrame& frame = notify_queue[notify_queue_len++];
  frame.len = len;
  frame.offset = 0;
  frame.id = next_notify_packet_id++;
  if (next_notify_packet_id == 0) next_notify_packet_id = 1;
  memcpy(frame.buf, raw, len);
  return true;
}

size_t RawLoRaBLEInterface::checkRecvRawPacket(uint8_t dest[], size_t max_len) {
  if (recv_queue_len == 0) return 0;

  uint16_t len = recv_queue[0].len;
  if (len > max_len) {
    RAW_LORA_BLE_DEBUG_PRINTLN("recv packet too big for dest, dropping len=%u", len);
    shiftRecvQueueLeft();
    return 0;
  }

  memcpy(dest, recv_queue[0].buf, len);
  shiftRecvQueueLeft();
  return len;
}

void RawLoRaBLEInterface::loop() {
  if (notify_queue_len > 0) {
    if (!isListening()) {
      notify_queue_len = 0;
    } else {
      unsigned long now = millis();
      bool throttle_active = _last_retry_attempt > 0 && (now - _last_retry_attempt) < RAW_LORA_BLE_RETRY_THROTTLE_MS;

      if (!throttle_active) {
        PacketFrame& frame = notify_queue[0];
        BLEConnection* conn = Bluefruit.Connection(_conn_handle);
        uint16_t max_payload = conn ? conn->getMtu() - 3 : 20;

        if (max_payload <= RAW_LORA_BLE_FRAME_OVERHEAD) {
          _last_retry_attempt = now;
          return;
        }

        uint16_t max_chunk = max_payload - RAW_LORA_BLE_FRAME_OVERHEAD;
        uint16_t chunk_len = frame.len - frame.offset;
        if (chunk_len > max_chunk) chunk_len = max_chunk;

        uint8_t ble_frame[RAW_LORA_BLE_MAX_GATT_FRAME];
        ble_frame[0] = RAW_LORA_BLE_FRAME_MAGIC;
        ble_frame[1] = frame.id;
        ble_frame[2] = (uint8_t)frame.len;
        ble_frame[3] = (uint8_t)frame.offset;
        memcpy(ble_frame + RAW_LORA_BLE_FRAME_OVERHEAD, frame.buf + frame.offset, chunk_len);

        if (raw_rx_characteristic.notify(_conn_handle, ble_frame, chunk_len + RAW_LORA_BLE_FRAME_OVERHEAD)) {
          _last_retry_attempt = 0;
          frame.offset += chunk_len;
          if (frame.offset >= frame.len) {
            shiftNotifyQueueLeft();
          }
        } else {
          _last_retry_attempt = now;
        }
      }
    }
  }

  unsigned long now = millis();
  if (_isEnabled && !isConnected() && _conn_handle == BLE_CONN_HANDLE_INVALID) {
    if (now - _last_health_check >= RAW_LORA_BLE_HEALTH_CHECK_INTERVAL) {
      _last_health_check = now;
      if (!isAdvertising()) {
        RAW_LORA_BLE_DEBUG_PRINTLN("advertising stopped, restarting");
        Bluefruit.Advertising.start(0);
      }
    }
  }
}
