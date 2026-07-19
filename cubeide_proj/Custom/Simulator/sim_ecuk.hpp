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
  float battery_v = 13.8f;
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
    PARAM_VBAT,
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
      {"Battery", "V", 0, 1, 1, 0, false, 0, 0, false, 1, 0},
  };
  volatile bool tx_done_ = false;
  volatile bool rx_done_ = false;
  UART_HandleTypeDef uart_ = {};
};

#endif /* CUSTOM_SIMULATOR_SIM_ECUK_HPP_ */
