#ifndef MUTII_LIB_ECUK_HPP_
#define MUTII_LIB_ECUK_HPP_

#include "ECUK.hpp"


class MUTII : public ECUK
{

public:
  MUTII(UART_HandleTypeDef *huart, bool *_txDone, bool *_rxDone) :
    ECUK(huart,_txDone,_rxDone)
  {

  }

private:
  const char INIT_SEQ = 0x01;
  const int BAUDRATE = 15625;

};


#endif
