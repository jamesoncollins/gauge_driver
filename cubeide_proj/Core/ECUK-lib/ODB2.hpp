#ifndef ODB2_LIB_ECUK_HPP_
#define ODB2_LIB_ECUK_HPP_

#include "ECUK.hpp"


class ODB2 : public ECUK
{

public:
  typedef enum
  {
    ECU_PARAM_SPEED,

    ECU_NUM_PARAMS
  } ecuParam_e;

  ecuParam_t ecuParams[ECU_NUM_PARAMS] = {
      {
          .name = "Speed",
          .units = "mph",
          .PID = 0x0d,//0x2F,
          .responseLen = 1,
          .scale = 1.2427424,
          .offset = 0,
          .val = 0,
          .lastTime_ms = -1,
          .priority = 1,
      },
  };

  ODB2(UART_HandleTypeDef *huart, bool *_txDone, bool *_rxDone) :
    ECUK(huart,_txDone,_rxDone)
  {

  }

private:

  int getNumParams()
  {
    return ECU_NUM_PARAMS;
  }

  int parse5BaudReply(const uint8_t *buffer_rx )
  {
    if (
             (buffer_rx[0]==0x55 && buffer_rx[1]==0x08 && buffer_rx[2]==0x08)
          || (buffer_rx[0]==0x55 && buffer_rx[1]==0x94 && buffer_rx[2]==0x94)
        )
    {
      return 0;
    }
    return 1;
  }

  void loadRequest( uint8_t *buffer_tx, int &txLen, int &rxLen )
  {
    buffer_tx[0] = 0x68; // addr
    buffer_tx[1] = 0x6a; // addr
    buffer_tx[2] = 0xf1; // addr
    buffer_tx[3] = 0x01; // mode
    buffer_tx[4] = ecuParams[ecuParamInd].PID; // PID
    buffer_tx[5] = buffer_tx[0] + buffer_tx[1] + buffer_tx[2] + buffer_tx[3] + buffer_tx[4];
    txLen = 6;
    rxLen = 12+ecuParams[ecuParamInd].responseLen;
  }

  int parseRequest(uint8_t *data)
  {
    int16_t val;

    ecuParam_t *ecuParam = &ecuParams[ecuParamInd];

    // jump past the looped back data we sent ourselve.
    // see loadRequest.
    data+=6;

    // test checksum
    uint8_t cs = 0;
    for(int i=0; i<5+ecuParam->responseLen; i++)
    {
      cs += data[i];
    }

    if(cs != data[5+ecuParam->responseLen])
    {
      ecuParam->lastTime_ms = -1;
      return -1;
    }

    if(ecuParam->responseLen==1)
      val = data[5];
    else
      val = (uint16_t)data[5] | (uint16_t)data[6]<<8;

    if(ecuParam->inverse)
      val = 1./val;

    ecuParam->val = (val*ecuParam->scale) + ecuParam->offset;
    ecuParam->lastTime_ms = HAL_GetTick();

    return 0;

  }

  ecuParam_t getParam(int ind)
  {
    return ecuParams[ind];
  }

};


#endif
