
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
    ECU_PARAM_KNOCK,
    ECU_PARAM_TIMING,
    ECU_PARAM_AFR_TARGET,

    ECU_PARAM_LOAD_LOW,
    ECU_PARAM_LOAD,

    ECU_PARAM_FFTL,
    ECU_PARAM_FFTM,
    ECU_PARAM_FFTH,
    ECU_PARAM_RFTL,
    ECU_PARAM_RFTM,
    ECU_PARAM_RFTH,

    ECU_PARAM_VBAT,

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
          .lastTime_ms = 0,
          .priority = 1,
      },
      {
          .name = "Speed",
          .units = "mph",
          .PID = 0x2F,
          .responseLen = 1,
          .scale = 1.2427424,
          .offset = 0,
          .val = 0,
          .lastTime_ms = 0,
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
          .lastTime_ms = 0,
          .priority = 1,
      },
      {
          .name = "Wideband",
          .units = "AFT",
          .PID = 0xBF,
          .responseLen = 1,
          .scale = 0.0627,
          .offset = 7,
          .val = 0,
          .lastTime_ms = 0,
          .priority = 1,
      },
      {
          .name = "Knock Sum",
          .units = "Count",
          .PID = 0x26,
          .responseLen = 1,
          .scale = 1,
          .offset = 0,
          .val = 0,
          .lastTime_ms = 0,
          .priority = 1,
      },
      {
          .name = "Timing Adv",
          .units = "°",
          .PID = 0x06,
          .responseLen = 1,
          .scale = 1,
          .offset = -20,
          .val = 0,
          .lastTime_ms = 0,
          .priority = 1,
      },
      {
          .name = "AFR Target",
          .units = "afr",
          .PID = 0x32,
          .responseLen = 1,
          .scale = 14.7*128.,
          .offset = 0,
          .inverse = true,
          .val = 0,
          .lastTime_ms = 0,
          .priority = 1,
      },

      {
          .name = "Load (High)",
          .units = "load",
          .PID = 0x03,
          .responseLen = 1,
          .scale = 1,
          .offset = 0,
          .inverse = false,
          .val = 0,
          .lastTime_ms = 0,
          .priority = 1,
      },
      {
          .name = "Load",
          .units = "load",
          .PID = 0x02,
          .responseLen = 1,
          .numMultiByte = 2,
          .scale = 5./16.,
          .offset = 0,
          .inverse = false,
          .val = 0,
          .lastTime_ms = 0,
          .priority = 1,
      },



      {
          .name = "FFTL",
          .units = "%",
          .PID = 0x4c,
          .responseLen = 1,
          .scale = 0.1953125,
          .offset = -25,
          .val = 0,
          .lastTime_ms = 0,
          .priority = 1000,
      },
      {
          .name = "FFTM",
          .units = "%",
          .PID = 0x4d,
          .responseLen = 1,
          .scale = 0.1953125,
          .offset = -25,
          .val = 0,
          .lastTime_ms = 0,
          .priority = 1001,
      },
      {
          .name = "FFTH",
          .units = "%",
          .PID = 0x4e,
          .responseLen = 1,
          .scale = 0.1953125,
          .offset = -25,
          .val = 0,
          .lastTime_ms = 0,
          .priority = 1002,
      },

      {
          .name = "RFTL",
          .units = "%",
          .PID = 0x0c,
          .responseLen = 1,
          .scale = 0.1953125,
          .offset = -25,
          .val = 0,
          .lastTime_ms = 0,
          .priority = 1003,
      },
      {
          .name = "RFTM",
          .units = "%",
          .PID = 0x0d,
          .responseLen = 1,
          .scale = 0.1953125,
          .offset = -25,
          .val = 0,
          .lastTime_ms = 0,
          .priority = 1004,
      },
      {
          .name = "RFTH",
          .units = "%",
          .PID = 0x0e,
          .responseLen = 1,
          .scale = 0.1953125,
          .offset = -25,
          .val = 0,
          .lastTime_ms = 0,
          .priority = 1005,
      },

      {
          .name = "Battery",
          .units = "V",
          .PID = 0x14,
          .responseLen = 1,
          .scale = 0.07333,
          .offset = 0,
          .val = 0,
          .lastTime_ms = 0,
          .priority = 1006,
      },
  };

  MUTII(UART_HandleTypeDef *huart, bool *_txDone, bool *_rxDone) :
    ECUK(huart,_txDone,_rxDone)
  {
    INIT_SEQ = 0x00;
    BAUDRATE = 0;//15625, set to zero to autobaud with 0x55
    NUM5BAUDREPLYBYTES = 3;//4;
    HASINITRESPONSE = false;
    ECU_REQUEST_DELAY_US = 0;
    REQUEST_BYTE_DELAY_US = 0;
  }

  ecuParam_t *getParam(int ind)
  {
    return &ecuParams[ind];
  }

private:
  int getNumParams()
  {
    return ECU_NUM_PARAMS;
  }

  int parse5BaudReply(const uint8_t *buffer_rx )
  {
    if (
             (buffer_rx[0]==0x55 && buffer_rx[1]==0xEf && buffer_rx[2]==0x85)
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

    // mutiii doesnt have any other response len?
//    if(ecuParam->responseLen==1)
//      val = data[0];
//    else
//      val = (uint16_t)data[0] | (uint16_t)data[1]<<8;
    val = data[0];

    /*
     * if this is a multibyte message then the ecuparam before this one had the high
     * byte.
     */
    if(ecuParam->numMultiByte == 2)
    {
      val = (uint16_t)(ecuParams[ecuParamInd-1].val)<<8 | ((uint16_t)val)<<0;
    }

    if(ecuParam->inverse)
      val = 1./val;

    ecuParam->val = (val*ecuParam->scale) + ecuParam->offset;
    ecuParam->lastTime_ms = HAL_GetTick();

    return 0;

  }



};


#endif
