
#include "main.h"

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
        timerECU = HAL_GetTick(); \
        ecuState = ECU_DELAY;     \
        ecuDelayFor_ms = 1000;    \
        ecuStateNext = ECU_RESET; \

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
  int elapsed = HAL_GetTick() - timerECU;

  switch(ecuState)
  {

    // start 5-baud init
    case ECU_RESET:
      initSuccess = false;
      HAL_UART_Abort(_huart);
      My_MX_USART1_UART_DeInit();       //fixme: we shouldnt call a hardware-specific function here
      timerECU = HAL_GetTick();
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, HIGH); // make sure line is high for awhile
      ecuState = ECU_DELAY;
      ecuDelayFor_ms = 2000;
      ecuStateNext = ECU_5_BAUD;
      *txDone = false;
      *rxDone = false;
      init_bit_ind = 0;
      break;

    // perform 5-baud init
    case ECU_5_BAUD:
    {
      init_bit_ind = (elapsed-200)/200;
      char init_bit = ((~INIT_SEQ)&(1<<init_bit_ind))>>init_bit_ind;
      if(elapsed < 200*1)
      {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, LOW);      // start bit
//        CLEAR_BIT(GPIOA->ODR, GPIO_PIN_9); //
//        SET_BIT(GPIOA->ODR, GPIO_PIN_9);
      }
      else if(elapsed < 200*9)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, (GPIO_PinState)init_bit );
      else
      {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, HIGH);        // stop bit
        *txDone = false;
        *rxDone = false;
        timerECU = HAL_GetTick();
        My_MX_USART1_UART_Init(BAUDRATE);
        WHILE_NOT(HAL_UART_Receive_IT( _huart,  &buffer_rx[0], NUM5BAUDREPLYBYTES )); // try to get reply data
        ecuState = ECU_5_BAUD_VERIFY;
      }
    }
      break;

    case ECU_5_BAUD_VERIFY:
      if(elapsed > 1000)
      {
        RESET
        break;
      }

      if(!*rxDone)
        break;

      if( parse5BaudReply(buffer_rx)==0 )
      {
        timerECU = HAL_GetTick();
        if(HASINITRESPONSE)
        {
          ecuState = ECU_DELAY;
          ecuDelayFor_ms = 50;    // W4 - 25ms
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
        break;
      }
      break;

    case ECU_5_BAUD_TX_KW_NOT:
      *txDone = false;
      *rxDone = false;
      WHILE_NOT(HAL_UART_Receive_IT( _huart,  buffer_rx, 2 )); // receive what we are going to send, plus the response from ecu
      WHILE_NOT(HAL_UART_Transmit_IT( _huart,  buffer_tx, 1 ));
      timerECU = HAL_GetTick();
      ecuState = ECU_5_BAUD_REPLY;
      break;

    case ECU_5_BAUD_REPLY:
      if(elapsed > 1000)
      {
        RESET
        break;
      }

      if( !(*txDone && *rxDone))
        break;

      // we received an init reply, check it
      if ( (buffer_rx[0] == buffer_tx[0] && buffer_rx[1] == 0xCC))
      {
        initSuccess = true;
        ecuStateNext = ECU_SEND_REQUEST;
        ecuDelayFor_ms = 70;  // P3 - 55ms
        timerECU = HAL_GetTick();
        ecuState = ECU_DELAY;
      }
      else
      {
        RESET
        break;
      }
      break;

    case ECU_SEND_REQUEST:
      *txDone = false;
      *rxDone = false;
      {
        int txLen, rxLen;
        loadRequest(buffer_tx, txLen, rxLen);
        WHILE_NOT(HAL_UART_Receive_IT( _huart,  buffer_rx, rxLen ));
        timerECU = HAL_GetTick();

        if(REQUEST_BYTE_DELAY_MS>0)
        {
          ecuState = ECU_DELAY_TX;
          ecuStateNext = ECU_PROCESS_REPLY;
          delayTxInd = 0;
          delayTxCnt = txLen;
          ecuDelayFor_ms = REQUEST_BYTE_DELAY_MS;
        }
        else
        {
          WHILE_NOT(HAL_UART_Transmit_IT( _huart,  buffer_tx, txLen ));
          ecuState = ECU_PROCESS_REPLY;
        }
      }
      break;

    case ECU_PROCESS_REPLY:
      if(elapsed > 1000)
      {
        RESET
        break;
      }

      if( !(*txDone && *rxDone))
        break;

      if( parseRequest( buffer_rx ) )
      {
        RESET
      }
      else
      {
        initSuccess = true;

        // move to next param, unless the load states dont match
        while(1)
        {
          ecuParamInd = (ecuParamInd+1==getNumParams()) ? 0 : ecuParamInd+1;
          if(false) // TODO: priority / load calculation
          {

          }
          else
          {
            // we found the next valid param
            ecuStateNext = ECU_SEND_REQUEST;
            ecuDelayFor_ms = ECU_REQUEST_DELAY_MS; // P3 - 55ms
            timerECU = HAL_GetTick();
            ecuState = ECU_DELAY;
            break;
          }
        }
      }
      break;

    case ECU_DELAY:
      /*
       * generic wait case.  used when we need to pause between
       * receiving a value and sending the next one
       */
      if(elapsed < ecuDelayFor_ms)
      {

      }
      else
      {
        ecuState = ecuStateNext;
        timerECU = HAL_GetTick();
      }
      break;

    case ECU_DELAY_TX:
      /*
       * send tx data 1 bytes at a time after the timeout period
       */
      if(elapsed > ecuDelayFor_ms)
      {
        while(HAL_UART_Transmit( _huart,  &buffer_tx[delayTxInd], 1, 1000 )!=HAL_OK){};
        if(++delayTxInd>=delayTxCnt)
        {
          ecuState = ecuStateNext;
          *txDone = true;
        }
        timerECU = HAL_GetTick();
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

float ECUK::getVal(int paramInd)
{
  return getParam(paramInd).val;
}
