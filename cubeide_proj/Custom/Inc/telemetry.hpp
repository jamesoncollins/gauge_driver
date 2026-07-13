#ifndef INC_TELEMETRY_HPP_
#define INC_TELEMETRY_HPP_

#include "board_model.hpp"

enum TelemetryPacketType : uint16_t
{
  TELEMETRY_PACKET_DASHBOARD = 1,
};

enum TelemetryPacketVersion : uint16_t
{
  TELEMETRY_PACKET_DASHBOARD_V1 = 1,
};

enum TelemetryValueType : uint8_t
{
  TELEMETRY_VALUE_UINT32 = 1,
  TELEMETRY_VALUE_FLOAT32 = 2,
  TELEMETRY_VALUE_BOOL8 = 3,
  TELEMETRY_VALUE_INT32 = 4,
};

enum TelemetryFieldId : uint8_t
{
  TELEMETRY_FIELD_SEQUENCE = 1,
  TELEMETRY_FIELD_RPM = 2,
  TELEMETRY_FIELD_SPEED_MPH = 3,
  TELEMETRY_FIELD_LOOP_COUNT = 4,
  TELEMETRY_FIELD_LOOP_PERIOD_MS = 5,
  TELEMETRY_FIELD_WORST_LOOP_PERIOD_MS = 6,
};

bool telemetry_publish_board_data(const BoardSharedData &data, uint32_t now_ms);

#endif /* INC_TELEMETRY_HPP_ */