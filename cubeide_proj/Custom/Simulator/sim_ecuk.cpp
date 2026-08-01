#include "sim_ecuk.hpp"

SimECUK::SimECUK() : ECUK(&uart_, &tx_done_, &rx_done_)
{
}

void SimECUK::connect()
{
  initSuccess = true;
}

void SimECUK::simulate(uint32_t now_ms, const SimVehicleSnapshot &vehicle)
{
  params_[PARAM_MAP].val = vehicle.map_psi;
  params_[PARAM_WB].val = vehicle.wideband_afr;
  params_[PARAM_TPS].val = vehicle.throttle_pct;
  params_[PARAM_KNOCK].val = vehicle.knock_count;
  params_[PARAM_VBAT].val = vehicle.battery_v;
  params_[PARAM_FFTL].val = vehicle.fuel_trim_front_low_pct;
  params_[PARAM_FFTM].val = vehicle.fuel_trim_front_med_pct;
  params_[PARAM_FFTH].val = vehicle.fuel_trim_front_high_pct;
  params_[PARAM_RFTL].val = vehicle.fuel_trim_rear_low_pct;
  params_[PARAM_RFTM].val = vehicle.fuel_trim_rear_med_pct;
  params_[PARAM_RFTH].val = vehicle.fuel_trim_rear_high_pct;

  for (int i = 0; i < PARAM_COUNT; ++i)
  {
    params_[i].lastTime_ms = now_ms;
    params_[i].isNew = true;
  }
}

ECUK::ecuParam_t *SimECUK::getParam(int ind)
{
  if (ind < 0 || ind >= PARAM_COUNT)
    return &params_[PARAM_MAP];
  return &params_[ind];
}

int SimECUK::parse5BaudReply(const uint8_t *)
{
  return 0;
}

void SimECUK::loadRequest(uint8_t *, int &txLen, int &rxLen)
{
  txLen = 0;
  rxLen = 0;
}

int SimECUK::parseRequest(uint8_t *)
{
  return 0;
}

int SimECUK::getNumParams()
{
  return PARAM_COUNT;
}
