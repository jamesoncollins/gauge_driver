#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void handleBleCommand(const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

namespace BleCommand
{
static constexpr uint8_t kMagic = 0x47;
static constexpr uint8_t kVersion = 0x01;
static constexpr uint8_t kMaxFrameLen = 20;
static constexpr uint8_t kHeaderLen = 6;
static constexpr uint16_t kResponseId = 0x434D;

enum Opcode : uint8_t
{
  OPCODE_BUTTON = 0x01,
  OPCODE_SET_MODE = 0x02,
  OPCODE_SET_BRIGHTNESS = 0x03,
  OPCODE_REQUEST_SNAPSHOT = 0x04,
  OPCODE_CONFIG_GET = 0x10,
  OPCODE_CONFIG_SET = 0x11,
  OPCODE_DIAG_PING = 0x20,
  OPCODE_DIAG_RESET_STATS = 0x21,
  OPCODE_DIAG_SQUARE = 0x22
};

enum Status : uint8_t
{
  STATUS_OK = 0x00,
  STATUS_BAD_FRAME = 0x01,
  STATUS_BAD_CRC = 0x02,
  STATUS_UNSUPPORTED_VERSION = 0x03,
  STATUS_UNKNOWN_OPCODE = 0x04,
  STATUS_BAD_LENGTH = 0x05,
  STATUS_BAD_VALUE = 0x06,
  STATUS_BUSY = 0x07,
  STATUS_DENIED = 0x08,
  STATUS_INTERNAL_ERROR = 0x09
};

uint8_t crc8(const uint8_t *data, uint8_t len);
}
