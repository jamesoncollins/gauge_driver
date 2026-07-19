#ifndef INC_APP_MODEL_HPP_
#define INC_APP_MODEL_HPP_

#include "platform_api.h"

struct SignalSample
{
  float value = 0.0f;
  bool fresh = false;
  bool valid = false;
  uint32_t timestamp_ms = 0;
};

struct CoreSignals
{
  SignalSample rpm;
  SignalSample speed_mph;
  SignalSample tps;
  SignalSample knock;
  SignalSample wb_afr;
  SignalSample map_psi;
  SignalSample ecu_batt_v;
  SignalSample board_batt_v;
};

struct ProductConfig
{
  const char *product_name = "unknown";
  int rpm_alert_init = 5700;
  int rpm_alert_final = 6500;
};

#endif /* INC_APP_MODEL_HPP_ */
