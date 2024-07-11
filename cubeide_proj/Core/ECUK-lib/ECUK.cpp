
#include "main.h"

#include "ECUK.hpp"

ECUK::ECUK(UART_HandleTypeDef *huart, bool *_txDone, bool *_rxDone)
{
  _huart = huart;
  timerECU = HAL_GetTick ();
  txDone = _txDone;
  rxDone = _rxDone;
}

void ECUK::update()
{

  /*
   * ecu interface
   */
  int elapsed = HAL_GetTick() - timerECU;
  ecuLoad_e currentLoad = ECU_LOAD_LOW;
  if(
      ecuParams[ECU_PARAM_TPS].val > TPS_THRESHOLD
      && HAL_GetTick()-ecuParams[ECU_PARAM_TPS].lastTime_ms<1000
      )
  {
    currentLoad = ECU_LOAD_HIGH;
  }

  if(
      ecuParams[ECU_PARAM_RPM].val > RPM_THRESHOLD
      && HAL_GetTick()-ecuParams[ECU_PARAM_RPM].lastTime_ms<1000
      )
  {
    currentLoad = ECU_LOAD_HIGH;
  }

  switch(ecuState)
  {
    uint8_t buffer_tx[10], buffer_rx[10];

    // start 5-baud init
    case ECU_RESET:
      My_MX_USART1_UART_DeInit();       //fixme: we shouldnt call a hardware-specific function here
      timerECU = HAL_GetTick();
      SET_BIT(GPIOA->ODR, GPIO_PIN_9);
      ecuState = ECU_5_BAUD;
      *txDone = false;
      *rxDone = false;
      break;

    // perform 5-baud init
    case ECU_5_BAUD:
      if(elapsed < 200*2)
        CLEAR_BIT(GPIOA->ODR, GPIO_PIN_9);
      else if(elapsed < 200*4)
        SET_BIT(GPIOA->ODR, GPIO_PIN_9);
      else if(elapsed < 200*6)
        CLEAR_BIT(GPIOA->ODR, GPIO_PIN_9);
      else if(elapsed < 200*8)
        SET_BIT(GPIOA->ODR, GPIO_PIN_9);
      else
      {
        My_MX_USART1_UART_Init();
        *txDone = false;
        *rxDone = false;
        timerECU = HAL_GetTick();
        HAL_UART_Receive(_huart, buffer_rx, 1, 0); // clear rx, if theres anything
        HAL_UART_Receive_IT( _huart,  buffer_rx, 3 ); // try to get reply data
        ecuState = ECU_5_BAUD_VERIFY;
      }
      break;

    case ECU_5_BAUD_VERIFY:
      if(elapsed > 1000)
      {
        // if we waited for over 1 second then we didnt receive a reply.
        // abor tthe transfer and start over
        ecuState = ECU_RESET;
        HAL_UART_Abort(_huart);
        break;
      }

      // see if we have received data
      if ( *rxDone &&
          (
               (buffer_rx[0]==0x55 && buffer_rx[1]==0x08 && buffer_rx[2]==0x08)
            || (buffer_rx[0]==0x55 && buffer_rx[1]==0x94 && buffer_rx[2]==0x94)
          )
          )
      {
        ecuState = ECU_DELAY;
        ecuDelayFor_ms = 30;
        ecuStateNext = ECU_5_BAUD_TX_KW_NOT;
        buffer_tx[0] = ~buffer_rx[2];
        timerECU = HAL_GetTick();
        *txDone = false;
        *rxDone = false;
        HAL_UART_Receive_IT( _huart,  buffer_rx, 2 ); // just receives what we sent, dont need it
      }
      break;

    case ECU_5_BAUD_TX_KW_NOT:
      *txDone = false;
      HAL_UART_Transmit_IT( _huart,  buffer_tx, 1 );
      timerECU = HAL_GetTick();
      ecuState = ECU_5_BAUD_REPLY;
      break;


    case ECU_5_BAUD_REPLY:
      if(!*txDone && elapsed > 1000)
      {
        // if we waited for over 1 second then we didnt receive a reply.
        // abort the transfer and start over
        ecuState = ECU_RESET;
        if(!*txDone)
          HAL_UART_Abort(_huart);
        break;
      }

      // we received an init reply, check it
      if (buffer_rx[0] == buffer_tx[0] && buffer_rx[1] == 0xCC)
      {
        ecuStateNext = ECU_SEND_REQUEST;
        ecuDelayFor_ms = 55;
        ecuState = ECU_DELAY;
      }
      break;

    case ECU_SEND_REQUEST:
      buffer_tx[0] = 0x68; // addr
      buffer_tx[1] = 0x6a; // addr
      buffer_tx[2] = 0xf1; // addr
      buffer_tx[3] = 0x01; // mode
      buffer_tx[4] = ecuParams[ecuParamInd].PID; // PID
      buffer_tx[5] = buffer_tx[0] + buffer_tx[1] + buffer_tx[2] + buffer_tx[3] + buffer_tx[4];
      *txDone = false;
      *rxDone = false;
      HAL_UART_Transmit_IT( _huart,  buffer_tx, 6 );
      HAL_UART_Receive_IT( _huart,  buffer_rx, 10+ecuParams[ecuParamInd].responseLen );
      timerECU = HAL_GetTick();
      ecuState = ECU_PROCESS_REPLY;
      break;

    case ECU_PROCESS_REPLY:
      if(elapsed > 1000)
      {
        ecuState = ECU_RESET;
        if(!*txDone || !*rxDone)
          HAL_UART_Abort(_huart);
        break;
      }

      if(!*rxDone)
        break;

      if(parseEcuParam( &ecuParams[ecuParamInd], &buffer_rx[6] ))
        ecuState = ECU_RESET;
      else
      {
        // move to next param, unless the load states dont match
        while(1)
        {
          ecuParamInd = (ecuParamInd+1==numEcuParams) ? 0 : ecuParamInd+1;
          if(currentLoad==ECU_LOAD_HIGH && ecuParams[ecuParamInd].load == ECU_LOAD_LOW)
          {

          }
          else
          {
            // we found the next valid param
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
      }
      break;


  }
}


int ECUK::parseEcuParam(ecuParam_t *ecuParam, uint8_t *data)
{
  int16_t val;

  // test checksum
  uint8_t cs = 0;
  for(int i=0; i<3+ecuParam->responseLen; i++)
  {
    cs += data[i];
  }

  if(cs != data[4+ecuParam->responseLen])
  {
    ecuParam->lastTime_ms = -1;
    return -1;
  }

  if(ecuParam->responseLen==1)
    val = data[0];
  else
    val = data[0] | data[1]<<8;

  if(ecuParam->inverse)
    val = 1./val;

  ecuParam->val = (val*ecuParam->scale) + ecuParam->offset;
  ecuParam->lastTime_ms = HAL_GetTick();

  return 0;

}


float ECUK::getVal(ecuParam_e paramInd)
{
  return ecuParams[paramInd].val;
}
