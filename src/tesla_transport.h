#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <functional>
#include <vector>

// Low-level BLE GATT client for a Tesla vehicle.
//
// Responsibilities:
//   * scan for the vehicle whose advertised local name matches our VIN
//   * open a GATT connection, negotiate a big MTU
//   * discover the Tesla service (0x0211) and its TX (0x0212) / RX (0x0213) characteristics
//   * reassemble RX indications (each message is prefixed with a 2-byte big-endian length)
//   * fragment outbound writes to the current (MTU - 3) payload size
//
// The transport exposes messages as fully-reassembled protobuf RoutableMessage byte
// buffers; framing is internal. Thread-safety: all public calls must be made from
// the Arduino main loop (NimBLE-Arduino is not safe to call from its own notify
// callback - we push received messages onto a FreeRTOS queue and drain them
// synchronously via poll()).
namespace tesla_transport {

using MessageCallback    = std::function<void(const uint8_t *data, size_t len)>;
using ConnectionCallback = std::function<void(bool connected)>;

void begin();
void setVin(const String &vin);

// Callbacks. Both may be null.
void setOnMessage(MessageCallback cb);
void setOnConnection(ConnectionCallback cb);

// Start (or resume) trying to find & connect to the vehicle. Safe to call
// repeatedly; becomes a no-op when already connected/connecting.
void requestConnect();
void disconnect();

bool isConnected();
bool isReady();                  // connected AND characteristics discovered

// Send a fully-formed RoutableMessage protobuf payload. The transport
// adds the 2-byte length prefix and fragments to MTU. Returns false if
// not currently ready.
bool sendMessage(const uint8_t *data, size_t len);

// Drive reassembly & event dispatch. Call from the main loop.
void loop();

}  // namespace tesla_transport
