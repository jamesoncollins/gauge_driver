

#include <stdio.h>

#include "main.h"

#include "utils.h"

#include "ECUK.hpp"

/*
 * iso driver negates signals on both tx and rx.
 *
 * for usart this is handleded through hardware and you dont need to think
 * about it.
 */
#define HIGH GPIO_PIN_RESET
#define LOW GPIO_PIN_SET

#define WHILE_NOT(ARG) while(ARG){};

#define RESET \
        timerECU = get_us_32(); \
        ecuState = ECU_DELAY;     \
        ecuDelayFor_us = 1000*1000;    \
        ecuStateNext = ECU_RESET;

ECUK::ECUK(UART_HandleTypeDef *huart, bool *_txDone, bool *_rxDone)
{
  _huart = huart;
  timerECU = HAL_GetTick ();
  txDone = _txDone;
  rxDone = _rxDone;
  initSuccess = false;
}

void ECUK::update()
{

  /*
   * ecu interface
   */
  uint32_t elapsed = get_us_32() - timerECU;
  char init_bit;
  bool cont;

  switch(ecuState)
  {

    // start 5-baud init
    case ECU_RESET:
      missedReplyCnt = 0;
      msgCount = 0;
      msgRate = 0;
      initSuccess = false;
      HAL_UART_Abort(_huart);
      My_MX_USART1_UART_DeInit();       //fixme: we shouldnt call a hardware-specific function here
      timerECU = get_us_32();
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, HIGH); // make sure line is high for awhile
      ecuState = ECU_DELAY;
      ecuDelayFor_us = 2e6;
      ecuStateNext = ECU_5_BAUD;
      *txDone = false;
      *rxDone = false;
      init_bit_ind = 0;
      break;

    // perform 5-baud init
    case ECU_5_BAUD:
      init_bit_ind = (elapsed-200000)/200000;
      init_bit = ((~INIT_SEQ)&(1<<init_bit_ind))>>init_bit_ind;
      if(elapsed < 200000*1)
      {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, LOW);      // start bit
      }
      else if(elapsed < 200000*9)
      {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, (GPIO_PinState)init_bit );
      }
      else
      {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, HIGH);        // stop bit
        *txDone = false;
        *rxDone = false;
        timerECU = get_us_32();
        My_MX_USART1_UART_Init(BAUDRATE);
        WHILE_NOT(HAL_UART_Receive_IT( _huart,  &buffer_rx[0], NUM5BAUDREPLYBYTES )); // try to get reply data
        ecuState = ECU_5_BAUD_VERIFY;
      }
      break;

    case ECU_5_BAUD_VERIFY:
      if(elapsed > 1000e3)
      {
        RESET
      }
      else if(!*rxDone)
      {

      }
      else if( parse5BaudReply(buffer_rx)==0 )
      {
        timerECU = get_us_32();
        if(HASINITRESPONSE)
        {
          ecuState = ECU_DELAY;
          ecuDelayFor_us = 50*1000;    // W4 - 25ms
          ecuStateNext = ECU_5_BAUD_TX_KW_NOT;
          buffer_tx[0] = ~buffer_rx[2];
        }
        else
        {
          ecuState = ECU_SEND_REQUEST;
        }
      }
      else
      {
        RESET
      }
      break;

    case ECU_5_BAUD_TX_KW_NOT:
      *txDone = false;
      *rxDone = false;
      WHILE_NOT(HAL_UART_Receive_IT( _huart,  buffer_rx, 2 )); // receive what we are going to send, plus the response from ecu
      WHILE_NOT(HAL_UART_Transmit_IT( _huart,  buffer_tx, 1 ));
      timerECU = get_us_32();
      ecuState = ECU_5_BAUD_REPLY;
      break;

    case ECU_5_BAUD_REPLY:
      if(elapsed > 1000e3)
      {
        RESET
      }
      else if( !(*txDone && *rxDone))
      {

      }
      // we received an init reply, check it
      else if ( (buffer_rx[0] == buffer_tx[0] && buffer_rx[1] == 0xCC))
      {
        ecuStateNext = ECU_SEND_REQUEST;
        ecuDelayFor_us = 70*1000;  // P3 - 55ms
        timerECU = get_us_32();
        ecuState = ECU_DELAY;
      }
      else
      {
        RESET
      }
      break;

    case ECU_SEND_REQUEST:
      if(msgCount == 0)
        msgCount_ms = HAL_GetTick();
      *txDone = false;
      *rxDone = false;
      loadRequest(buffer_tx, txLen, rxLen);
      WHILE_NOT(HAL_UART_Receive_IT( _huart,  buffer_rx, rxLen ));
      timerECU = get_us_32();

      if(REQUEST_BYTE_DELAY_US>0)
      {
        ecuState = ECU_DELAY_TX;
        ecuStateNext = ECU_PROCESS_REPLY;
        delayTxInd = 0;
        delayTxCnt = txLen;
        ecuDelayFor_us = REQUEST_BYTE_DELAY_US;
      }
      else
      {
        WHILE_NOT(HAL_UART_Transmit_IT( _huart,  buffer_tx, txLen ));
        ecuState = ECU_PROCESS_REPLY;
      }
      break;

    case ECU_PROCESS_REPLY:
      if(elapsed > ECU_REPLY_TIMEOUT_US)
      {
        missedReplyCnt++;
        if(missedReplyCnt>=MISSED_REPLY_THRESHOLD)
        {
          /*
           * every time we hit this threshold we increase the delay between a
           * good reply and the next request
           */
          ECU_REQUEST_DELAY_US += (ECU_REQUEST_DELAY_US/2) + 100;
          missedReplyResetCnt++;
          RESET
        }
        else
        {
          // if we didnt hit the thresold then just go back to the
          // state where we send a request.  but we have to abort becuase
          // we have an outstanding read still.
          HAL_UART_Abort(_huart);
          ecuStateNext = ECU_SEND_REQUEST;
          ecuDelayFor_us = ECU_REQUEST_DELAY_US; // P3 - 55ms
          timerECU = get_us_32();
          ecuState = ECU_DELAY;
        }
      }
      else if( !(*txDone && *rxDone))
      {

      }
      else if( parseRequest( buffer_rx ) )
      {
        RESET
      }
      else
      {
        initSuccess = true;
        missedReplyCnt = 0;

        getParam(ecuParamInd)->isNew = true;

        msgCount++;
        if(HAL_GetTick() - msgCount_ms > 1000)
        {
          msgRate = (msgCount*1000) / (HAL_GetTick() - msgCount_ms);
          msgCount = 0;
          msgCount_ms = HAL_GetTick();
        }

        // move to next param, unless the load states dont match
        cont = true;
        while(cont)
        {
          ecuParamInd = (ecuParamInd+1==getNumParams()) ? 0 : ecuParamInd+1;
          if((cont=HAL_GetTick()-getParam(ecuParamInd)->lastTime_ms < getParam(ecuParamInd)->priority))
          {
            /*
             * we recently got this low priority value, skip it
             */
          }
          else
          {
            // we found the next valid param
            ecuStateNext = ECU_SEND_REQUEST;
            ecuDelayFor_us = ECU_REQUEST_DELAY_US; // P3 - 55ms
            timerECU = get_us_32();
            ecuState = ECU_DELAY;
          }
        }
      }
      break;

    case ECU_DELAY:
      /*
       * generic wait case.  used when we need to pause between
       * receiving a value and sending the next one
       */
      if(elapsed < ecuDelayFor_us)
      {

      }
      else
      {
        ecuState = ecuStateNext;
        timerECU = get_us_32();
      }
      break;

    case ECU_DELAY_TX:
      /*
       * send tx data 1 bytes at a time after the timeout period
       */
      if(elapsed > ecuDelayFor_us)
      {
        while(HAL_UART_Transmit( _huart,  &buffer_tx[delayTxInd], 1, 1000 )!=HAL_OK){};
        if(++delayTxInd>=delayTxCnt)
        {
          ecuState = ecuStateNext;
          *txDone = true;
        }
        timerECU = get_us_32();
      }
      break;
  }
}

const char* ECUK::getStatus()
{
  if(initSuccess)
    return "OK";

  switch(ecuState)
  {
    case ECU_RESET:
      return "RST";
      break;

    case ECU_5_BAUD:
      return "5BUD";
      break;

    case ECU_5_BAUD_VERIFY:
    case ECU_5_BAUD_REPLY:
    case ECU_5_BAUD_TX_KW_NOT:
    case ECU_DELAY:
      return "INIT";
      break;

    case ECU_SEND_REQUEST:
    case ECU_PROCESS_REPLY:
      return "OK";

    default:
      return "ERR";
      break;
  }
}

bool ECUK::isConnected()
{
  /*
   * TODO: it shouldnt be an error if we've never even tried connecting yet.
   * this function should have a different name, like isConnectionGood or something.
   */
  if(initSuccess)
    return true;
  else
    return false;
}

float ECUK::getVal(int paramInd)
{
  getParam(paramInd)->isNew = false;
  return getParam(paramInd)->val;
}

const char * ECUK::getValString(int paramInd)
{
  getParam(paramInd)->isNew = false;
  if(
      getParam(paramInd)->lastTime_ms<0
      || (HAL_GetTick()-getParam(paramInd)->lastTime_ms)>1000
      )
  {
    return "--";
  }
  snprintf(get_val_buffer, 10, "%.1f", getParam(paramInd)->val);
  return get_val_buffer;
}

uint32_t ECUK::getMsgRate()
{
  return msgRate;
}

uint32_t ECUK::getMissedReplyResetCnt()
{
  return missedReplyResetCnt;
}
