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

    ECU_DELAY,

    ECU_DELAY_TX
  }
  ecuState_e;

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
    int priority;
  }
  ecuParam_t;

  ECUK(UART_HandleTypeDef *huart, bool *_txDone, bool *_rxDone);

  void update();

  float getVal(int);

  const char* getStatus();

protected:
  const char INIT_SEQ = 0xC0;
  const int BAUDRATE = 10400;
  const int NUM5BAUDREPLYBYTES = 3;
  const bool HASINITRESPONSE = true;
  const int REQUEST_BYTE_DELAY_MS = 5;
  const int ECU_REQUEST_DELAY_MS = 55;

  virtual int parse5BaudReply(const uint8_t *) = 0;

  /*
   * load PID request into buffer
   *
   * return expected receive length
   */
  virtual void loadRequest( uint8_t *, int &txLen, int &rxLen ) = 0;
  virtual int parseRequest( uint8_t *data) = 0;
  virtual int getNumParams() = 0;
  virtual ecuParam_t getParam(int ind) = 0;

  UART_HandleTypeDef *_huart;
  int ecuParamInd = 0;
  uint32_t timerECU = 0;
  ecuState_e ecuState = ECU_RESET;
  ecuState_e ecuStateNext = ECU_RESET;
  int ecuDelayFor_ms = 0;
  bool *txDone, *rxDone;
  bool initSuccess = false;
  uint8_t buffer_tx[10], buffer_rx[15];
  int delayTxInd = 0, delayTxCnt = 0;

};


#endif /* ECUK_LIB_ECUK_HPP_ */
