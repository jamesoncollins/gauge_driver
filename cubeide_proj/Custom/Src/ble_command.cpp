#include "ble_command.hpp"

#include <string.h>

#include "board_model.hpp"
#include "platform_api.h"
#include "main_shared.h"
#include "gauge_layouts.hpp"
#include "../BTBuffer-lib/BTBuffer.hpp"

#if !defined(CUSTOM_PLATFORM_X86)
#include "runtime_context.hpp"
#endif

namespace
{
static constexpr uint8_t FLAG_ACK_REQUESTED = 0x01;
static constexpr uint8_t RESPONSE_HEADER_LEN = 3;
static constexpr uint8_t RESPONSE_MAX_DETAIL_LEN = BTBuffer::dataLen - RESPONSE_HEADER_LEN;

struct ParsedCommand
{
  uint8_t seq = 0;
  uint8_t opcode = 0;
  uint8_t flags = 0;
  const uint8_t *payload = nullptr;
  uint8_t payload_len = 0;
};

void emit_response(uint8_t seq, uint8_t opcode, BleCommand::Status status, const uint8_t *detail, uint8_t detail_len)
{
  if (!BTBuffer::IsCreated())
    return;

  uint8_t response[BTBuffer::dataLen] = {};
  if (detail_len > RESPONSE_MAX_DETAIL_LEN)
    detail_len = RESPONSE_MAX_DETAIL_LEN;

  response[0] = static_cast<uint8_t>(status);
  response[1] = opcode;
  response[2] = detail_len;
  if (detail_len > 0 && detail != nullptr)
    memcpy(&response[RESPONSE_HEADER_LEN], detail, detail_len);

  BTBuffer::pushBuffer(BleCommand::kResponseId, seq, HAL_GetTick(), response, RESPONSE_HEADER_LEN + detail_len);
}

bool should_ack(const ParsedCommand &cmd, BleCommand::Status status)
{
  return status != BleCommand::STATUS_OK || ((cmd.flags & FLAG_ACK_REQUESTED) != 0);
}

BleCommand::Status parse_command(const uint8_t *data, uint8_t len, ParsedCommand &cmd)
{
  if (data == nullptr || len < BleCommand::kHeaderLen + 1 || len > BleCommand::kMaxFrameLen)
    return BleCommand::STATUS_BAD_FRAME;

  if (data[0] != BleCommand::kMagic)
    return BleCommand::STATUS_BAD_FRAME;

  if (data[1] != BleCommand::kVersion)
    return BleCommand::STATUS_UNSUPPORTED_VERSION;

  const uint8_t payload_len = data[5];
  const uint8_t frame_len = static_cast<uint8_t>(BleCommand::kHeaderLen + payload_len + 1);
  if (payload_len > BleCommand::kMaxFrameLen - BleCommand::kHeaderLen - 1)
    return BleCommand::STATUS_BAD_LENGTH;

  if (len != frame_len && len != BleCommand::kMaxFrameLen)
    return BleCommand::STATUS_BAD_LENGTH;

  if (BleCommand::crc8(data, static_cast<uint8_t>(frame_len - 1)) != data[frame_len - 1])
    return BleCommand::STATUS_BAD_CRC;

  cmd.seq = data[2];
  cmd.opcode = data[3];
  cmd.flags = data[4];
  cmd.payload_len = payload_len;
  cmd.payload = &data[BleCommand::kHeaderLen];
  return BleCommand::STATUS_OK;
}

BleCommand::Status dispatch_button(const ParsedCommand &cmd)
{
  if (cmd.payload_len != 1)
    return BleCommand::STATUS_BAD_LENGTH;

  const uint8_t button = cmd.payload[0];
  if (button > BTN_R)
    return BleCommand::STATUS_BAD_VALUE;

#if !defined(CUSTOM_PLATFORM_X86)
  btnCmd = static_cast<button_e>(button);
  return BleCommand::STATUS_OK;
#else
  (void)button;
  return BleCommand::STATUS_DENIED;
#endif
}

BleCommand::Status dispatch_command(const ParsedCommand &cmd, const uint8_t **detail, uint8_t *detail_len)
{
  *detail = nullptr;
  *detail_len = 0;

  switch (cmd.opcode)
  {
  case BleCommand::OPCODE_BUTTON:
    return dispatch_button(cmd);

  case BleCommand::OPCODE_DIAG_PING:
    *detail = cmd.payload;
    *detail_len = cmd.payload_len;
    return BleCommand::STATUS_OK;

  case BleCommand::OPCODE_DIAG_RESET_STATS:
    if (cmd.payload_len != 0)
      return BleCommand::STATUS_BAD_LENGTH;
    runtime_diag_reset_shared();
    return BleCommand::STATUS_OK;

  case BleCommand::OPCODE_DIAG_SQUARE:
    if (cmd.payload_len != 1)
      return BleCommand::STATUS_BAD_LENGTH;
    runtime_diag_set_square_enabled(cmd.payload[0] != 0U);
    return BleCommand::STATUS_OK;

  case BleCommand::OPCODE_SET_MODE:
    if (cmd.payload_len != 1)
      return BleCommand::STATUS_BAD_LENGTH;
    return gauge_layouts_set_mode(cmd.payload[0]) == GaugeLayouts::SET_MODE_OK
               ? BleCommand::STATUS_OK
               : BleCommand::STATUS_BAD_VALUE;

  case BleCommand::OPCODE_SET_BRIGHTNESS:
  case BleCommand::OPCODE_REQUEST_SNAPSHOT:
  case BleCommand::OPCODE_CONFIG_GET:
  case BleCommand::OPCODE_CONFIG_SET:
    return BleCommand::STATUS_DENIED;

  default:
    return BleCommand::STATUS_UNKNOWN_OPCODE;
  }
}
}

namespace BleCommand
{
uint8_t crc8(const uint8_t *data, uint8_t len)
{
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; ++i)
  {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07) : static_cast<uint8_t>(crc << 1);
  }
  return crc;
}
}

extern "C" void handleBleCommand(const uint8_t *data, uint8_t len)
{
  ParsedCommand cmd = {};
  BleCommand::Status status = parse_command(data, len, cmd);
  const uint8_t *detail = nullptr;
  uint8_t detail_len = 0;

  if (status == BleCommand::STATUS_OK)
    status = dispatch_command(cmd, &detail, &detail_len);

  if (status != BleCommand::STATUS_OK || should_ack(cmd, status))
    emit_response(cmd.seq, cmd.opcode, status, detail, detail_len);
}
