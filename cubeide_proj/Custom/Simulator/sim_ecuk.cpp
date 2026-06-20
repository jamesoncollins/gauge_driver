#include <cmath>

#include "sim_ecuk.hpp"

SimECUK::SimECUK() : ECUK(&uart_, &tx_done_, &rx_done_)
{
}

void SimECUK::connect()
{
  initSuccess = true;
}

void SimECUK::simulate(uint32_t now_ms)
{
  const float t = now_ms / 1000.0f;

  params_[PARAM_MAP].val = 8.0f + 14.0f * (0.5f + 0.5f * std::sinf(t * 0.8f));
  params_[PARAM_WB].val = 11.8f + 2.5f * (0.5f + 0.5f * std::sinf(t * 0.55f));
  params_[PARAM_TPS].val = 100.0f * (0.5f + 0.5f * std::sinf(t * 1.8f));
  params_[PARAM_KNOCK].val = 10.0f * (0.5f + 0.5f * std::sinf(t * 2.7f));
  params_[PARAM_VBAT].val = 13.8f + 0.3f * std::sinf(t * 0.25f);

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
