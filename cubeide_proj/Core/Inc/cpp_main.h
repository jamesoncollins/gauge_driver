
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
#define SAMPLE_TIME_MS_PRINT     750
#define TARGET_FPS               20
#define SAMPLE_TIME_MS_DRAW     (1000/TARGET_FPS) // it takes 60ms to refresh the screen
                                                   // with -O2 you can draw in about 10.
                                                   // so 70ms seems to be ablout the best you can do here
//#define PRINT_TO_USB
//#define SWEEP_GAUGES  // sweep needles forever
//#define SIM_GAUGES       // generate simulated rpm and mph

const int RPM_ALERT_INIT = 5000; // soft alert threshold
const int RPM_ALERT_FINAL = 6000; // hard threshold (i.e. red screen, constant buzz)

/*
 * dots per inch for this screen
 */
const int DPI = 240. / 1.4456693; //166
const int DPMM = 240. / 36.72; // 6.53 dots per mm

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
