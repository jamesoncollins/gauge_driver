/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_stm.c
  * @author  MCD Application Team
  * @brief   Custom Example Service.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "common_blesvc.h"
#include "custom_stm.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct{
  uint16_t  CustomUserbuttonserviceHdle;                    /**< UserButtonService handle */
  uint16_t  CustomButtonpressHdle;                  /**< buttonPress handle */
  uint16_t  CustomClientreadserviceHdle;                    /**< ClientReadService handle */
  uint16_t  CustomReadnextHdle;                  /**< readNext handle */
  uint16_t  CustomRebootreqserviceHdle;                    /**< RebootReqService handle */
  uint16_t  CustomBm_Req_CharHdle;                  /**< BM_REQ_CHAR handle */
/* USER CODE BEGIN Context */
  /* Place holder for Characteristic Descriptors Handle*/

/* USER CODE END Context */
}CustomContext_t;

extern uint16_t Connection_Handle;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/
#define UUID_128_SUPPORTED  1

#if (UUID_128_SUPPORTED == 1)
#define BM_UUID_LENGTH  UUID_TYPE_128
#else
#define BM_UUID_LENGTH  UUID_TYPE_16
#endif

#define BM_REQ_CHAR_SIZE    (3)

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros ------------------------------------------------------------*/
#define CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET         2
#define CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET              1
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
uint16_t SizeButtonpress = 1;
uint16_t SizeReadnext = 64;
uint16_t SizeBm_Req_Char = 3;

/**
 * START of Section BLE_DRIVER_CONTEXT
 */
static CustomContext_t CustomContext;

/**
 * END of Section BLE_DRIVER_CONTEXT
 */

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static SVCCTL_EvtAckStatus_t Custom_STM_Event_Handler(void *pckt);

static tBleStatus Generic_STM_App_Update_Char_Ext(uint16_t ConnectionHandle, uint16_t ServiceHandle, uint16_t CharHandle, uint16_t CharValueLen, uint8_t *pPayload);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
/* USER CODE BEGIN PFD */

/* USER CODE END PFD */

/* Private functions ----------------------------------------------------------*/

#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
    uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
    uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
    uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)

#define COPY_USERBUTTONSERVICE_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0xcc,0x7a,0x48,0x2a,0x98,0x4a,0x7f,0x2e,0xd5,0xb3,0xe5,0x8f)
#define COPY_BUTTONPRESS_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x01,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_CLIENTREADSERVICE_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x02,0xcc,0x7a,0x48,0x2a,0x98,0x4a,0x7f,0x2e,0xd5,0xb3,0xe5,0x8f)
#define COPY_READNEXT_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x03,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_REBOOTREQSERVICE_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x00,0x00,0x0a,0x00,0xcc,0x7a,0x48,0x2a,0x98,0x4a,0x7f,0x2e,0xd5,0xb3,0xe5,0x8f)
#define COPY_BM_REQ_CHAR_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0xfe,0x11,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)

/* USER CODE BEGIN PF */

/* USER CODE END PF */

/**
 * @brief  Event handler
 * @param  Event: Address of the buffer holding the Event
 * @retval Ack: Return whether the Event has been managed or not
 */
static SVCCTL_EvtAckStatus_t Custom_STM_Event_Handler(void *Event)
{
  SVCCTL_EvtAckStatus_t return_value;
  hci_event_pckt *event_pckt;
  evt_blecore_aci *blecore_evt;
  aci_gatt_attribute_modified_event_rp0 *attribute_modified;
  aci_gatt_notification_complete_event_rp0    *notification_complete;
  Custom_STM_App_Notification_evt_t     Notification;
  /* USER CODE BEGIN Custom_STM_Event_Handler_1 */

  /* USER CODE END Custom_STM_Event_Handler_1 */

  return_value = SVCCTL_EvtNotAck;
  event_pckt = (hci_event_pckt *)(((hci_uart_pckt*)Event)->data);

  switch (event_pckt->evt)
  {
    case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE:
      blecore_evt = (evt_blecore_aci*)event_pckt->data;
      switch (blecore_evt->ecode)
      {
        case ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE:
          /* USER CODE BEGIN EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_BEGIN */

          /* USER CODE END EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_BEGIN */
          attribute_modified = (aci_gatt_attribute_modified_event_rp0*)blecore_evt->data;
          if (attribute_modified->Attr_Handle == (CustomContext.CustomReadnextHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1 */

            /* USER CODE END CUSTOM_STM_Service_2_Char_1 */
            switch (attribute_modified->Attr_Data[0])
            {
              /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_attribute_modified */

              /* USER CODE END CUSTOM_STM_Service_2_Char_1_attribute_modified */

              /* Disabled Notification management */
              case (!(COMSVC_Notification)):
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_Disabled_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_2_Char_1_Disabled_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_READNEXT_NOTIFY_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_2_Char_1_Disabled_END */
                break;

              /* Enabled Notification management */
              case COMSVC_Notification:
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_COMSVC_Notification_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_2_Char_1_COMSVC_Notification_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_READNEXT_NOTIFY_ENABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_COMSVC_Notification_END */

                /* USER CODE END CUSTOM_STM_Service_2_Char_1_COMSVC_Notification_END */
                break;

              default:
                /* USER CODE BEGIN CUSTOM_STM_Service_2_Char_1_default */

                /* USER CODE END CUSTOM_STM_Service_2_Char_1_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomReadnextHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomButtonpressHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_1_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
            Notification.Custom_Evt_Opcode = CUSTOM_STM_BUTTONPRESS_WRITE_EVT;
            Notification.DataTransfered.Length=attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload=attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
            /* USER CODE END CUSTOM_STM_Service_1_Char_1_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
          } /* if (attribute_modified->Attr_Handle == (CustomContext.CustomButtonpressHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomBm_Req_CharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
            Notification.Custom_Evt_Opcode = CUSTOM_STM_BM_REQ_CHAR_WRITE_EVT;
            Notification.DataTransfered.Length=attribute_modified->Attr_Data_Length;
            Notification.DataTransfered.pPayload=attribute_modified->Attr_Data;
            Custom_STM_App_Notification(&Notification);
            /* USER CODE END CUSTOM_STM_Service_3_Char_1_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
          } /* if (attribute_modified->Attr_Handle == (CustomContext.CustomBm_Req_CharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
          /* USER CODE BEGIN EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_END */

          /* USER CODE END EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_END */
          break;

        case ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE :
          /* USER CODE BEGIN EVT_BLUE_GATT_READ_PERMIT_REQ_BEGIN */

          /* USER CODE END EVT_BLUE_GATT_READ_PERMIT_REQ_BEGIN */
          /* USER CODE BEGIN EVT_BLUE_GATT_READ_PERMIT_REQ_END */

          /* USER CODE END EVT_BLUE_GATT_READ_PERMIT_REQ_END */
          break;

        case ACI_GATT_WRITE_PERMIT_REQ_VSEVT_CODE:
          /* USER CODE BEGIN EVT_BLUE_GATT_WRITE_PERMIT_REQ_BEGIN */

          /* USER CODE END EVT_BLUE_GATT_WRITE_PERMIT_REQ_BEGIN */
          /* USER CODE BEGIN EVT_BLUE_GATT_WRITE_PERMIT_REQ_END */

          /* USER CODE END EVT_BLUE_GATT_WRITE_PERMIT_REQ_END */
          break;

		case ACI_GATT_NOTIFICATION_COMPLETE_VSEVT_CODE:
        {
          /* USER CODE BEGIN EVT_BLUE_GATT_NOTIFICATION_COMPLETE_BEGIN */

          /* USER CODE END EVT_BLUE_GATT_NOTIFICATION_COMPLETE_BEGIN */
          notification_complete = (aci_gatt_notification_complete_event_rp0*)blecore_evt->data;
          Notification.Custom_Evt_Opcode = CUSTOM_STM_NOTIFICATION_COMPLETE_EVT;
          Notification.AttrHandle = notification_complete->Attr_Handle;
          Custom_STM_App_Notification(&Notification);
          /* USER CODE BEGIN EVT_BLUE_GATT_NOTIFICATION_COMPLETE_END */

          /* USER CODE END EVT_BLUE_GATT_NOTIFICATION_COMPLETE_END */
          break;
        }

        /* USER CODE BEGIN BLECORE_EVT */

        /* USER CODE END BLECORE_EVT */
        default:
          /* USER CODE BEGIN EVT_DEFAULT */

          /* USER CODE END EVT_DEFAULT */
          break;
      }
      /* USER CODE BEGIN EVT_VENDOR*/

      /* USER CODE END EVT_VENDOR*/
      break; /* HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE */

      /* USER CODE BEGIN EVENT_PCKT_CASES*/

      /* USER CODE END EVENT_PCKT_CASES*/

    default:
      /* USER CODE BEGIN EVENT_PCKT*/

      /* USER CODE END EVENT_PCKT*/
      break;
  }

  /* USER CODE BEGIN Custom_STM_Event_Handler_2 */

  /* USER CODE END Custom_STM_Event_Handler_2 */

  return(return_value);
}/* end Custom_STM_Event_Handler */

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Service initialization
 * @param  None
 * @retval None
 */
void SVCCTL_InitCustomSvc(void)
{

  Char_UUID_t  uuid;
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  uint8_t max_attr_record;

  /* USER CODE BEGIN SVCCTL_InitCustomSvc_1 */

  /* USER CODE END SVCCTL_InitCustomSvc_1 */

  /**
   *  Register the event handler to the BLE controller
   */
  SVCCTL_RegisterSvcHandler(Custom_STM_Event_Handler);

  /**
   *          UserButtonService
   *
   * Max_Attribute_Records = 1 + 2*1 + 1*no_of_char_with_notify_or_indicate_property + 1*no_of_char_with_broadcast_property
   * service_max_attribute_record = 1 for UserButtonService +
   *                                2 for buttonPress +
   *                              = 3
   *
   * This value doesn't take into account number of descriptors manually added
   * In case of descriptors added, please update the max_attr_record value accordingly in the next SVCCTL_InitService User Section
   */
  max_attr_record = 3;

  /* USER CODE BEGIN SVCCTL_InitService */
  /* max_attr_record to be updated if descriptors have been added */

  /* USER CODE END SVCCTL_InitService */

  COPY_USERBUTTONSERVICE_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             max_attr_record,
                             &(CustomContext.CustomUserbuttonserviceHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_service command: UserButtonService, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_service command: UserButtonService \n\r");
  }

  /**
   *  buttonPress
   */
  COPY_BUTTONPRESS_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomUserbuttonserviceHdle,
                          UUID_TYPE_128, &uuid,
                          SizeButtonpress,
                          CHAR_PROP_WRITE,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomButtonpressHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : BUTTONPRESS, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : BUTTONPRESS \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service1_Char1 */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service1_Char1 */

  /**
   *          ClientReadService
   *
   * Max_Attribute_Records = 1 + 2*1 + 1*no_of_char_with_notify_or_indicate_property + 1*no_of_char_with_broadcast_property
   * service_max_attribute_record = 1 for ClientReadService +
   *                                2 for readNext +
   *                                1 for readNext configuration descriptor +
   *                              = 4
   *
   * This value doesn't take into account number of descriptors manually added
   * In case of descriptors added, please update the max_attr_record value accordingly in the next SVCCTL_InitService User Section
   */
  max_attr_record = 4;

  /* USER CODE BEGIN SVCCTL_InitService */
  /* max_attr_record to be updated if descriptors have been added */

  /* USER CODE END SVCCTL_InitService */

  COPY_CLIENTREADSERVICE_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             max_attr_record,
                             &(CustomContext.CustomClientreadserviceHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_service command: ClientReadService, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_service command: ClientReadService \n\r");
  }

  /**
   *  readNext
   */
  COPY_READNEXT_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomClientreadserviceHdle,
                          UUID_TYPE_128, &uuid,
                          SizeReadnext,
                          CHAR_PROP_READ | CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE,
                          GATT_DONT_NOTIFY_EVENTS,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomReadnextHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : READNEXT, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : READNEXT \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service2_Char1 */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service2_Char1 */

  /**
   *          RebootReqService
   *
   * Max_Attribute_Records = 1 + 2*1 + 1*no_of_char_with_notify_or_indicate_property + 1*no_of_char_with_broadcast_property
   * service_max_attribute_record = 1 for RebootReqService +
   *                                2 for BM_REQ_CHAR +
   *                              = 3
   *
   * This value doesn't take into account number of descriptors manually added
   * In case of descriptors added, please update the max_attr_record value accordingly in the next SVCCTL_InitService User Section
   */
  max_attr_record = 3;

  /* USER CODE BEGIN SVCCTL_InitService */
  /* max_attr_record to be updated if descriptors have been added */

  /* USER CODE END SVCCTL_InitService */

  COPY_REBOOTREQSERVICE_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             max_attr_record,
                             &(CustomContext.CustomRebootreqserviceHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_service command: RebootReqService, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_service command: RebootReqService \n\r");
  }

  /**
   *  BM_REQ_CHAR
   */
  COPY_BM_REQ_CHAR_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomRebootreqserviceHdle,
                          UUID_TYPE_128, &uuid,
                          SizeBm_Req_Char,
                          CHAR_PROP_WRITE,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomBm_Req_CharHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : BM_REQ_CHAR, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : BM_REQ_CHAR \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service3_Char1 */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service3_Char1 */

  /* USER CODE BEGIN SVCCTL_InitCustomSvc_2 */

  /* USER CODE END SVCCTL_InitCustomSvc_2 */

  return;
}

/**
 * @brief  Characteristic update
 * @param  CharOpcode: Characteristic identifier
 * @param  Service_Instance: Instance of the service to which the characteristic belongs
 *
 */
tBleStatus Custom_STM_App_Update_Char(Custom_STM_Char_Opcode_t CharOpcode, uint8_t *pPayload)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  /* USER CODE BEGIN Custom_STM_App_Update_Char_1 */

  /* USER CODE END Custom_STM_App_Update_Char_1 */

  switch (CharOpcode)
  {

    case CUSTOM_STM_BUTTONPRESS:
      ret = aci_gatt_update_char_value(CustomContext.CustomUserbuttonserviceHdle,
                                       CustomContext.CustomButtonpressHdle,
                                       0, /* charValOffset */
                                       SizeButtonpress, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value BUTTONPRESS command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value BUTTONPRESS command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_1_Char_1*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_1_Char_1*/
      break;

    case CUSTOM_STM_READNEXT:
      ret = aci_gatt_update_char_value(CustomContext.CustomClientreadserviceHdle,
                                       CustomContext.CustomReadnextHdle,
                                       0, /* charValOffset */
                                       SizeReadnext, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value READNEXT command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value READNEXT command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_2_Char_1*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_2_Char_1*/
      break;

    case CUSTOM_STM_BM_REQ_CHAR:
      ret = aci_gatt_update_char_value(CustomContext.CustomRebootreqserviceHdle,
                                       CustomContext.CustomBm_Req_CharHdle,
                                       0, /* charValOffset */
                                       SizeBm_Req_Char, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value BM_REQ_CHAR command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value BM_REQ_CHAR command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_3_Char_1*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_3_Char_1*/
      break;

    default:
      break;
  }

  /* USER CODE BEGIN Custom_STM_App_Update_Char_2 */

  /* USER CODE END Custom_STM_App_Update_Char_2 */

  return ret;
}

/**
 * @brief  Characteristic update
 * @param  CharOpcode: Characteristic identifier
 * @param  pPayload: Characteristic value
 * @param  size: Length of the characteristic value in octets
 *
 */
tBleStatus Custom_STM_App_Update_Char_Variable_Length(Custom_STM_Char_Opcode_t CharOpcode, uint8_t *pPayload, uint8_t size)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  /* USER CODE BEGIN Custom_STM_App_Update_Char_Variable_Length_1 */

  /* USER CODE END Custom_STM_App_Update_Char_Variable_Length_1 */

  switch (CharOpcode)
  {

    case CUSTOM_STM_BUTTONPRESS:
      ret = aci_gatt_update_char_value(CustomContext.CustomUserbuttonserviceHdle,
                                       CustomContext.CustomButtonpressHdle,
                                       0, /* charValOffset */
                                       size, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value BUTTONPRESS command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value BUTTONPRESS command\n\r");
      }
      /* USER CODE BEGIN Custom_STM_App_Update_Char_Variable_Length_Service_1_Char_1*/

      /* USER CODE END Custom_STM_App_Update_Char_Variable_Length_Service_1_Char_1*/
      break;

    case CUSTOM_STM_READNEXT:
      ret = aci_gatt_update_char_value(CustomContext.CustomClientreadserviceHdle,
                                       CustomContext.CustomReadnextHdle,
                                       0, /* charValOffset */
                                       size, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value READNEXT command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value READNEXT command\n\r");
      }
      /* USER CODE BEGIN Custom_STM_App_Update_Char_Variable_Length_Service_2_Char_1*/

      /* USER CODE END Custom_STM_App_Update_Char_Variable_Length_Service_2_Char_1*/
      break;

    case CUSTOM_STM_BM_REQ_CHAR:
      ret = aci_gatt_update_char_value(CustomContext.CustomRebootreqserviceHdle,
                                       CustomContext.CustomBm_Req_CharHdle,
                                       0, /* charValOffset */
                                       size, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value BM_REQ_CHAR command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value BM_REQ_CHAR command\n\r");
      }
      /* USER CODE BEGIN Custom_STM_App_Update_Char_Variable_Length_Service_3_Char_1*/

      /* USER CODE END Custom_STM_App_Update_Char_Variable_Length_Service_3_Char_1*/
      break;

    default:
      break;
  }

  /* USER CODE BEGIN Custom_STM_App_Update_Char_Variable_Length_2 */

  /* USER CODE END Custom_STM_App_Update_Char_Variable_Length_2 */

  return ret;
}

/**
 * @brief  Characteristic update
 * @param  Connection_Handle
 * @param  CharOpcode: Characteristic identifier
 * @param  pPayload: Characteristic value
 *
 */
tBleStatus Custom_STM_App_Update_Char_Ext(uint16_t Connection_Handle, Custom_STM_Char_Opcode_t CharOpcode, uint8_t *pPayload)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  /* USER CODE BEGIN Custom_STM_App_Update_Char_Ext_1 */

  /* USER CODE END Custom_STM_App_Update_Char_Ext_1 */

  switch (CharOpcode)
  {

    case CUSTOM_STM_BUTTONPRESS:
      /* USER CODE BEGIN Updated_Length_Service_1_Char_1*/

      /* USER CODE END Updated_Length_Service_1_Char_1*/
	  Generic_STM_App_Update_Char_Ext(Connection_Handle, CustomContext.CustomUserbuttonserviceHdle, CustomContext.CustomButtonpressHdle, SizeButtonpress, pPayload);

      break;

    case CUSTOM_STM_READNEXT:
      /* USER CODE BEGIN Updated_Length_Service_2_Char_1*/

      /* USER CODE END Updated_Length_Service_2_Char_1*/
	  Generic_STM_App_Update_Char_Ext(Connection_Handle, CustomContext.CustomClientreadserviceHdle, CustomContext.CustomReadnextHdle, SizeReadnext, pPayload);

      break;

    case CUSTOM_STM_BM_REQ_CHAR:
      /* USER CODE BEGIN Updated_Length_Service_3_Char_1*/

      /* USER CODE END Updated_Length_Service_3_Char_1*/
	  Generic_STM_App_Update_Char_Ext(Connection_Handle, CustomContext.CustomRebootreqserviceHdle, CustomContext.CustomBm_Req_CharHdle, SizeBm_Req_Char, pPayload);

      break;

    default:
      break;
  }

  /* USER CODE BEGIN Custom_STM_App_Update_Char_Ext_2 */

  /* USER CODE END Custom_STM_App_Update_Char_Ext_2 */

  return ret;
}

static tBleStatus Generic_STM_App_Update_Char_Ext(uint16_t ConnectionHandle, uint16_t ServiceHandle, uint16_t CharHandle, uint16_t CharValueLen, uint8_t *pPayload)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;

  ret = aci_gatt_update_char_value_ext(ConnectionHandle,
                                       ServiceHandle,
                                       CharHandle,
                                       0, /* update type:0 do not notify, 1 notify, 2 indicate */
                                       CharValueLen, /* charValueLen */
                                       0, /* value offset */
                                       243, /* value length */
                                       (uint8_t *)  pPayload);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_update_char_value_ext command, part 1, result : 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_update_char_value_ext command, part 1\n\r");
  }
  /* USER CODE BEGIN Custom_STM_App_Update_Char_Ext_Service_1_Char_1*/

  if (CharValueLen-243<=243)
  {
    ret = aci_gatt_update_char_value_ext(ConnectionHandle,
                                         ServiceHandle,
                                         CharHandle,
                                         1, /* update type:0 do not notify, 1 notify, 2 indicate */
                                         CharValueLen, /* charValueLen */
                                         243, /* value offset */
                                         CharValueLen-243, /* value length */
                                         (uint8_t *)  ((pPayload)+243));
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("  Fail   : aci_gatt_update_char_value_ext command, part 2, result : 0x%x \n\r", ret);
    }
    else
    {
      APP_DBG_MSG("  Success: aci_gatt_update_char_value_ext command, part 2\n\r");
    }
  }
  else
  {
    ret = aci_gatt_update_char_value_ext(ConnectionHandle,
                                         ServiceHandle,
                                         CharHandle,
                                         0, /* update type:0 do not notify, 1 notify, 2 indicate */
                                         CharValueLen, /* charValueLen */
                                         243, /* value offset */
                                         243, /* value length */
                                         (uint8_t *)  ((pPayload)+243));
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("  Fail   : aci_gatt_update_char_value_ext command, part 3, result : 0x%x \n\r", ret);
    }
    else
    {
      APP_DBG_MSG("  Success: aci_gatt_update_char_value_ext command, part 3\n\r");
    }
    ret = aci_gatt_update_char_value_ext(ConnectionHandle,
                                         ServiceHandle,
                                         CharHandle,
                                         1, /* update type:0 do not notify, 1 notify, 2 indicate */
                                         CharValueLen, /* charValueLen */
                                         243+243, /* value offset */
                                         CharValueLen-243-243, /* value length */
                                         (uint8_t *)  ((pPayload)+243+243));
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("  Fail   : aci_gatt_update_char_value_ext command, part 4, result : 0x%x \n\r", ret);
    }
    else
    {
      APP_DBG_MSG("  Success: aci_gatt_update_char_value_ext command, part 4\n\r");
    }
  }
  return ret;
}

