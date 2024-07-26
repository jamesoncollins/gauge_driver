
/*
 * https://evoecu.logic.net/wiki/MUT_Protocol
 */

#ifndef MUTII_LIB_ECUK_HPP_
#define MUTII_LIB_ECUK_HPP_

#include "ECUK.hpp"


class MUTII : public ECUK
{

public:


  typedef enum
  {
    ECU_PARAM_TPS,
    ECU_PARAM_SPEED,
    ECU_PARAM_RPM,
    ECU_PARAM_WB,
//    ECU_PARAM_KNOCK,
//    ECU_PARAM_TIMING,
//    ECU_PARAM_AFR_TARGET,
//
//    ECU_PARAM_FFTL,
//    ECU_PARAM_FFTM,
//    ECU_PARAM_FFTH,
//    ECU_PARAM_RFTL,
//    ECU_PARAM_RFTM,
//    ECU_PARAM_RFTH,

    ECU_NUM_PARAMS
  } ecuParam_e;

  ecuParam_t ecuParams[ECU_NUM_PARAMS] = {
      {
          .name = "TPS",
          .units = "%",
          .PID = 0x17,
          .responseLen = 1,
          .scale = 100./255.,
          .offset = 0,
          .val = 0,
          .lastTime_ms = -1,
          .priority = 1,
      },
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
      {
          .name = "RPM",
          .units = "rpm",
          .PID = 0x21,
          .responseLen = 1,
          .scale = 31.25,
          .offset = 0,
          .val = 0,
          .lastTime_ms = -1,
          .priority = 1,
      },
      {
          .name = "Wideband",
          .units = "AFT",
          .PID = 0xBF,
          .responseLen = 1,
          .scale = 0.0627,
          .offset = 7,
          .val = 14.7,
          .lastTime_ms = 0, //FIXME
          .priority = 1,
      },
//      {
//          .name = "Knock Sum",
//          .units = "Count",
//          .PID = 0x26,
//          .responseLen = 1,
//          .scale = 1,
//          .offset = 0,
//          .val = 0,
//          .lastTime_ms = -1,
//          .priority = 1,
//      },
//      {
//          .name = "Timing Adv",
//          .units = "°",
//          .PID = 0x06,
//          .responseLen = 1,
//          .scale = 1,
//          .offset = -20,
//          .val = 0,
//          .lastTime_ms = -1,
//          .priority = 1,
//      },
//      {
//          .name = "AFR Target",
//          .units = "afr",
//          .PID = 0x32,
//          .responseLen = 1,
//          .scale = 14.7*128.,
//          .offset = 0,
//          .inverse = true,
//          .val = 0,
//          .lastTime_ms = -1,
//          .priority = 1,
//      },
//
//
//
//      {
//          .name = "FFTL",
//          .units = "%",
//          .PID = 0x4c,
//          .responseLen = 1,
//          .scale = 0.1953125,
//          .offset = -25,
//          .val = 0,
//          .lastTime_ms = -1,
//          .priority = 100,
//      },
//      {
//          .name = "FFTM",
//          .units = "%",
//          .PID = 0x4d,
//          .responseLen = 1,
//          .scale = 0.1953125,
//          .offset = -25,
//          .val = 0,
//          .lastTime_ms = -1,
//          .priority = 100,
//      },
//      {
//          .name = "FFTH",
//          .units = "%",
//          .PID = 0x4e,
//          .responseLen = 1,
//          .scale = 0.1953125,
//          .offset = -25,
//          .val = 0,
//          .lastTime_ms = -1,
//          .priority = 100,
//      },
//
//      {
//          .name = "RFTL",
//          .units = "%",
//          .PID = 0x0c,
//          .responseLen = 1,
//          .scale = 0.1953125,
//          .offset = -25,
//          .val = 0,
//          .lastTime_ms = -1,
//          .priority = 100,
//      },
//      {
//          .name = "RFTM",
//          .units = "%",
//          .PID = 0x0d,
//          .responseLen = 1,
//          .scale = 0.1953125,
//          .offset = -25,
//          .val = 0,
//          .lastTime_ms = -1,
//          .priority = 100,
//      },
//      {
//          .name = "RFTH",
//          .units = "%",
//          .PID = 0x0e,
//          .responseLen = 1,
//          .scale = 0.1953125,
//          .offset = -25,
//          .val = 0,
//          .lastTime_ms = -1,
//          .priority = 100,
//      },
  };

  MUTII(UART_HandleTypeDef *huart, bool *_txDone, bool *_rxDone) :
    ECUK(huart,_txDone,_rxDone)
  {

  }

private:
  const char INIT_SEQ = 0x01;
  const int BAUDRATE = 15625;
  const int NUM5BAUDREPLYBYTES = 4;
  const bool HASINITRESPONSE = false;
  const int ECU_REQUEST_DELAY_MS = 0;

  int getNumParams()
  {
    return ECU_NUM_PARAMS;
  }

  int parse5BaudReply(const uint8_t *buffer_rx )
  {
    if (
             (buffer_rx[0]==0xC0 && buffer_rx[1]==0x55 && buffer_rx[2]==0xEf && buffer_rx[3]==0x85)
        )
    {
      return 0;
    }
    return 1;
  }

  void loadRequest( uint8_t *buffer_tx, int &txLen, int &rxLen )
  {
    buffer_tx[0] = ecuParams[ecuParamInd].PID;
    txLen = 1;
    rxLen = 1+ecuParams[ecuParamInd].responseLen;
  }

  int parseRequest( uint8_t *data)
  {
    int16_t val;

    ecuParam_t *ecuParam = &ecuParams[ecuParamInd];

    // jump past the looped back data we sent ourselve.
    // see loadRequest.
    data+=1;

    if(ecuParam->responseLen==1)
      val = data[0];
    else
      val = (uint16_t)data[0] | (uint16_t)data[1]<<8;

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
