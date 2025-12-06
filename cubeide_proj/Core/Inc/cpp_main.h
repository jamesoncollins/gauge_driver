
#ifndef INC_MAIN_CPP_H_
#define INC_MAIN_CPP_H_



/*
 * function declarations
 */
int get_x12_ticks_speed( float  );
int get_x12_ticks_rpm( float  );
HAL_StatusTypeDef HAL_TIM_Base_Start_DMA_to_SPI(TIM_HandleTypeDef *htim, const uint32_t *pData, uint16_t Length);


/*
 * #defnes and constants that control operation
 */

// main loop timers
#define SAMPLE_TIME_MS_LED       1000
#define SAMPLE_TIME_MS_PRINT     50
#define TARGET_FPS               20
#define SAMPLE_TIME_MS_DRAW     (1000/TARGET_FPS) // it takes 60ms to refresh the screen
                                                   // with -O2 you can draw in about 10.
                                                   // so 70ms seems to be ablout the best you can do here
//#define SWEEP_GAUGES  // sweep needles forever
//#define SIM_GAUGES       // generate simulated rpm and mph


/*
 * USB-based diagnostics
 *
 * Print to USB: what it sounds like, prints text to the  USB port
 * USB_DISPLAY: sends the display buffer over USB, use the script in ./python to view it.
 *
 * Note, you can use both of these at the same time.
 */
//#define PRINT_TO_USB
//#define USB_DISPLAY

/*
 * Optional debug / diagnostic print on the screen
 */
//#define DIAG_SQUARE

/*
 * RPM/Shift alert points
 */
const int RPM_ALERT_RESET = 5500; // soft alert OFF, provides some hysteresis
const int RPM_ALERT_INIT = 5700; // soft alert / early warn
const int RPM_ALERT_FINAL = 6500; // SHIFT


/*
 * dots per inch for this screen
 */
const int DPI = 240. / 1.4456693; //166
const int DPMM = 240. / 36.72; // 6.53 dots per mm

/*
 * gauge measurement settings
 */
const float MPH_PER_HZ = ( 0.8425872f * 1.015625f ); //(1.11746031667) //( 1.07755102 )
const float  RPM_PER_HZ = ( 20. ); // 3 ticks per revolution

/*
 * button type for bluetooth UI
 */
typedef enum
{
  BTN_OK = 0,
  BTN_U, BTN_D, BTN_L, BTN_R,
  BTN_INV = 255
}
button_e;

#endif /* INC_MAIN_CPP_H_ */
