#ifndef ECUK_LIB_ECUK_HPP_
#define ECUK_LIB_ECUK_HPP_




class ECUK
{
public:
  // if we're above either of these values then consider it a highload situation
  const int TPS_THRESHOLD = 30;
  const int RPM_THRESHOLD = 3000;

  typedef enum
  {
    ECU_RESET = 0,

    ECU_5_BAUD,
    ECU_5_BAUD_VERIFY,
    ECU_5_BAUD_REPLY,
    ECU_5_BAUD_TX_KW_NOT,

    ECU_SEND_REQUEST,
    ECU_PROCESS_REPLY,

    ECU_DELAY
  }
  ecuState_e;

  typedef enum
  {
    ECU_LOAD_LOW, // log when load is low
    ECU_LOAD_HIGH // always log
  }
  ecuLoad_e;

  typedef struct
  {
    const char name[16];
    const char units[16];
    const uint8_t PID;
    const uint8_t responseLen;
    const float scale, offset;
    const bool inverse;         // 1 / x
    float val;
    int lastTime_ms;
    ecuLoad_e load;
  }
  ecuParam_t;

  typedef enum
  {
    ECU_PARAM_TPS,
    ECU_PARAM_SPEED,
    ECU_PARAM_RPM,
    ECU_PARAM_WB,
    ECU_PARAM_KNOCK,
    ECU_PARAM_TIMING,
    ECU_PARAM_AFR_TARGET,

    ECU_PARAM_FFTL,
    ECU_PARAM_FFTM,
    ECU_PARAM_FFTH,
    ECU_PARAM_RFTL,
    ECU_PARAM_RFTM,
    ECU_PARAM_RFTH,

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
          .load = ECU_LOAD_HIGH,
      },
      {
          .name = "Speed",
          .units = "mph",
          .PID = 0x2F,
          .responseLen = 1,
          .scale = 1.2427424,
          .offset = 0,
          .val = 0,
          .lastTime_ms = -1,
          .load = ECU_LOAD_HIGH,
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
          .load = ECU_LOAD_HIGH,
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
          .load = ECU_LOAD_HIGH,
      },
      {
          .name = "Knock Sum",
          .units = "Count",
          .PID = 0x26,
          .responseLen = 1,
          .scale = 1,
          .offset = 0,
          .val = 0,
          .lastTime_ms = -1,
          .load = ECU_LOAD_HIGH,
      },
      {
          .name = "Timing Adv",
          .units = "°",
          .PID = 0x06,
          .responseLen = 1,
          .scale = 1,
          .offset = -20,
          .val = 0,
          .lastTime_ms = -1,
          .load = ECU_LOAD_HIGH,
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
          .lastTime_ms = -1,
          .load = ECU_LOAD_HIGH,
      },



      {
          .name = "FFTL",
          .units = "%",
          .PID = 0x4c,
          .responseLen = 1,
          .scale = 0.1953125,
          .offset = -25,
          .val = 0,
          .lastTime_ms = -1,
          .load = ECU_LOAD_LOW,
      },
      {
          .name = "FFTM",
          .units = "%",
          .PID = 0x4d,
          .responseLen = 1,
          .scale = 0.1953125,
          .offset = -25,
          .val = 0,
          .lastTime_ms = -1,
          .load = ECU_LOAD_LOW,
      },
      {
          .name = "FFTH",
          .units = "%",
          .PID = 0x4e,
          .responseLen = 1,
          .scale = 0.1953125,
          .offset = -25,
          .val = 0,
          .lastTime_ms = -1,
          .load = ECU_LOAD_LOW,
      },

      {
          .name = "RFTL",
          .units = "%",
          .PID = 0x0c,
          .responseLen = 1,
          .scale = 0.1953125,
          .offset = -25,
          .val = 0,
          .lastTime_ms = -1,
          .load = ECU_LOAD_LOW,
      },
      {
          .name = "RFTM",
          .units = "%",
          .PID = 0x0d,
          .responseLen = 1,
          .scale = 0.1953125,
          .offset = -25,
          .val = 0,
          .lastTime_ms = -1,
          .load = ECU_LOAD_LOW,
      },
      {
          .name = "RFTH",
          .units = "%",
          .PID = 0x0e,
          .responseLen = 1,
          .scale = 0.1953125,
          .offset = -25,
          .val = 0,
          .lastTime_ms = -1,
          .load = ECU_LOAD_LOW,
      },
  };

  constexpr static const int numEcuParams = sizeof(ecuParams) / sizeof(ecuParams[0]);

  ECUK(UART_HandleTypeDef *huart, bool *_txDone, bool *_rxDone);

  void update();

  float getVal(ecuParam_e);

  const char* getStatus();

private:
  int parseEcuParam(ecuParam_t *ecuParam, uint8_t *data);

  UART_HandleTypeDef *_huart;
  int ecuParamInd = 0;
  uint32_t timerECU = 0;
  ecuState_e ecuState = ECU_RESET;
  ecuState_e ecuStateNext = ECU_RESET;
  int ecuDelayFor_ms = 0;
  bool *txDone, *rxDone;
  bool initSuccess = false;

};


#endif /* ECUK_LIB_ECUK_HPP_ */
