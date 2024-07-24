#ifndef ODB2_LIB_ECUK_HPP_
#define ODB2_LIB_ECUK_HPP_

#include "ECUK.hpp"


class ODB2 : public ECUK
{

public:
  ODB2(UART_HandleTypeDef *huart, bool *_txDone, bool *_rxDone) :
    ECUK(huart,_txDone,_rxDone)
  {

  }

private:
  const char INIT_SEQ = 0xC0;
  const int BAUDRATE = 10400;

};


#endif
