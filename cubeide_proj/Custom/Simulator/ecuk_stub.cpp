#include <cstdio>

#include "platform_api.h"
#include "../ECUK-lib/ECUK.hpp"

ECUK::ECUK(UART_HandleTypeDef *huart, volatile bool *_txDone, volatile bool *_rxDone)
{
  _huart = huart;
  txDone = _txDone;
  rxDone = _rxDone;
  initSuccess = false;
}

void ECUK::update()
{
}

float ECUK::getVal(int ind)
{
  ecuParam_t *p = getParam(ind);
  return p ? p->val : 0.0f;
}

const char *ECUK::getValString(int ind)
{
  static char buf[32];
  (void)std::snprintf(buf, sizeof(buf), "%.2f", getVal(ind));
  return buf;
}

const char *ECUK::getStatus()
{
  return initSuccess ? "OK" : "DISCONNECTED";
}

bool ECUK::isConnected()
{
  return initSuccess;
}

uint32_t ECUK::getMsgRate()
{
  return 0;
}

uint32_t ECUK::getMissedReplyResetCnt()
{
  return 0;
}
