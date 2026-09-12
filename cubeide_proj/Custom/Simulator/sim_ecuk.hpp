#ifndef CUSTOM_SIMULATOR_SIM_ECUK_HPP_
#define CUSTOM_SIMULATOR_SIM_ECUK_HPP_

#include "platform_api.h"
#include "../ECUK-lib/ECUK.hpp"
struct SimVehicleSnapshot
{
  float rpm = 0.0f;
  float speed_mph = 0.0f;
  float throttle_pct = 0.0f;
  float wideband_afr = 14.7f;
  float map_psi = 0.0f;
  float knock_count = 0.0f;
  float timing_deg = 10.0f;
  float afr_target = 14.7f;
  float battery_v = 13.8f;
  float fuel_trim_front_low_pct = 0.0f;
  float fuel_trim_front_med_pct = 0.0f;
  float fuel_trim_front_high_pct = 0.0f;
  float fuel_trim_rear_low_pct = 0.0f;
  float fuel_trim_rear_med_pct = 0.0f;
  float fuel_trim_rear_high_pct = 0.0f;
  float acceleration_mps2 = 0.0f;
  int gear = 0;
};

class SimECUK : public ECUK
{
public:
  enum ParamIndex
  {
    PARAM_TPS = 0,
    PARAM_WB,
    PARAM_MAP,
    PARAM_KNOCK,
    PARAM_TIMING,
    PARAM_AFR_TARGET,
    PARAM_VBAT,
    PARAM_FFTL,
    PARAM_FFTM,
    PARAM_FFTH,
    PARAM_RFTL,
    PARAM_RFTM,
    PARAM_RFTH,
    PARAM_COUNT
  };

  SimECUK();
  void connect();
  void simulate(uint32_t now_ms, const SimVehicleSnapshot &vehicle);

  ecuParam_t *getParam(int ind) override;

private:
  int parse5BaudReply(const uint8_t *) override;
  void loadRequest(uint8_t *, int &txLen, int &rxLen) override;
  int parseRequest(uint8_t *) override;
  int getNumParams() override;

  ecuParam_t params_[PARAM_COUNT] = {
      {"TPS", "%", 0, 1, 1, 0, false, 0, 0, false, 1, 0},
      {"Wideband", "AFR", 0, 1, 1, 0, false, 0, 0, false, 1, 0},
      {"MAP", "PSI", 0, 1, 1, 0, false, 0, 0, false, 1, 0},
      {"Knock", "Count", 0, 1, 1, 0, false, 0, 0, false, 1, 0},
      {"Timing", "DEG", 0, 1, 1, 0, false, 0, 0, false, 1, 0},
      {"AFR Target", "AFR", 0, 1, 1, 0, false, 0, 0, false, 1, 0},
      {"Battery", "V", 0, 1, 1, 0, false, 0, 0, false, 1, 0},
      {"FFTL", "%", 0, 1, 1, 1, 0, false, 0, 0, false, 1000},
      {"FFTM", "%", 0, 1, 1, 1, 0, false, 0, 0, false, 1001},
      {"FFTH", "%", 0, 1, 1, 1, 0, false, 0, 0, false, 1002},
      {"RFTL", "%", 0, 1, 1, 1, 0, false, 0, 0, false, 1003},
      {"RFTM", "%", 0, 1, 1, 1, 0, false, 0, 0, false, 1004},
      {"RFTH", "%", 0, 1, 1, 1, 0, false, 0, 0, false, 1005},
  };
  volatile bool tx_done_ = false;
  volatile bool rx_done_ = false;
  UART_HandleTypeDef uart_ = {};
};

#endif /* CUSTOM_SIMULATOR_SIM_ECUK_HPP_ */
