#include <cstring>

#include "telemetry.hpp"
#include "../BTBuffer-lib/BTBuffer.hpp"

namespace
{
static uint32_t g_telemetry_sequence = 0;

struct TlvWriter
{
  uint8_t *buf = nullptr;
  int cap = 0;
  int len = 0;

  bool write_raw(uint8_t field_id, uint8_t value_type, const void *value, uint8_t value_len)
  {
    if (buf == nullptr || value == nullptr)
      return false;
    const int needed = 3 + (int)value_len;
    if ((len + needed) > cap)
      return false;
    buf[len++] = field_id;
    buf[len++] = value_type;
    buf[len++] = value_len;
    std::memcpy(&buf[len], value, value_len);
    len += value_len;
    return true;
  }

  bool write_u32(uint8_t field_id, uint32_t value)
  {
    return write_raw(field_id, TELEMETRY_VALUE_UINT32, &value, sizeof(value));
  }

  bool write_float(uint8_t field_id, float value)
  {
    return write_raw(field_id, TELEMETRY_VALUE_FLOAT32, &value, sizeof(value));
  }
};
}

bool telemetry_publish_board_data(const BoardSharedData &data, uint32_t now_ms)
{
  if (!BTBuffer::IsCreated())
    return false;

  uint8_t payload[BTBuffer::dataLen] = {};
  TlvWriter writer = {payload, BTBuffer::dataLen, 0};

  (void)writer.write_u32(TELEMETRY_FIELD_SEQUENCE, g_telemetry_sequence++);

  if (data.rpm.supported && data.rpm.good)
    (void)writer.write_float(TELEMETRY_FIELD_RPM, data.rpm.value);
  if (data.speed_mph.supported && data.speed_mph.good)
    (void)writer.write_float(TELEMETRY_FIELD_SPEED_MPH, data.speed_mph.value);
  if (data.loop_count.supported && data.loop_count.good)
    (void)writer.write_u32(TELEMETRY_FIELD_LOOP_COUNT, data.loop_count.value);
  if (data.loop_period_ms.supported && data.loop_period_ms.good)
    (void)writer.write_u32(TELEMETRY_FIELD_LOOP_PERIOD_MS, data.loop_period_ms.value);
  if (data.worst_loop_period_ms.supported && data.worst_loop_period_ms.good)
    (void)writer.write_u32(TELEMETRY_FIELD_WORST_LOOP_PERIOD_MS, data.worst_loop_period_ms.value);

  return BTBuffer::pushBuffer(
      TELEMETRY_PACKET_DASHBOARD,
      TELEMETRY_PACKET_DASHBOARD_V1,
      now_ms,
      payload,
      writer.len);
}