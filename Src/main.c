/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "hrtim.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "ad7172.h"
#include "stdbool.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef unsigned char uint8;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCD_BUF_SIZE 32
#define VOUT_SENSE_DIV    22624
#define VIN_SENSE_DIV     23624
#define SUPPLY_SENSE_DIV  23.62443439
#define TIMER_PERIOD      0x2FFF

/* timer period in ns */
#define TIMER_PERIOD_NS   2666

/* inductance in nano-Henries */
#define L_NH              6800

/* how to know when we should be in discontinuous mode */
#define DCM_MARGIN        1.0

/* allowable devation from setpoint, in millivolts */
#define VOUT_ERROR_MARGIN 5

#define I_SENSE_CH        0
#define VOUT_SENSE_CH     1
#define VIN_SENSE_CH      2

#define VSET_MAX          49.0

#define VIN_SAMPLE_RATE   250
#define I_SAMPLE_RATE     20
#define V_SAMPLE_DEPTH    64
#define I_SAMPLE_DEPTH    16

/* counts of deadtime to insert between high side and low side switching (~55ns) */
#define DEADTIME_COUNTS   256

/* gain x 1000 */
#define I_SHUNT_GAIN      21100
/* Current shunt value in milliohms */
#define I_SHUNT           4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* converts millivolts read by ADC to true voltage in millivolts */
#define V_SENSE_MV(mv)    (mv*VOUT_SENSE_DIV)/1000
#define VIN_SENSE_MV(mv)  (mv*VIN_SENSE_DIV)/1000

/* converts from millivolts read by ADC to output current in milliamps */
#define I_OUT_MA(mv_adc)  (mv_adc*1000)/(I_SHUNT*I_SHUNT_GAIN)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8 debug_buf[512];
float voltage_12v = 12.0;
float voltage_24v = 24.0;
float voltage_36v = 36.0;
float voltage_48v = 48.0;

uint8_t   input_relay   = 0; // selects which relay should be active (1 - 4)
uint8_t   last_relay    = 0; // tracks the last value of input_relay

uint32_t  vout_sense_div= VOUT_SENSE_DIV;
uint32_t  vout          = 0; // output voltage in millivolts
uint32_t  vout_meas_raw = 0; // voltage at the ADC input in millivolts
uint32_t  iout_meas_raw = 0; // amplified shunt voltage at the ADC input in millivolts
uint32_t  iout          = 0; // output current in milliamps (mA)
uint32_t  vin_sense_div = VIN_SENSE_DIV;
uint32_t  vin_meas_raw  = 0; // voltage at the ADC input in millivolts
uint32_t  vin           = 0; // input voltage in millivolts
int       kp            = 16;
int       ki            = 2;
int       kd            = 1;
int       verr          = 0;
int       last_verr     = 0;
int       verr_integ    = 0;
int       n_duty_max    = 0;

bool      update_lcd_flag = false;
float     user_vset     = 5.0; // initial voltage set: 5.0V
float     user_ilim     = 5.0; // initial current limit: 5.0A
uint32_t  vout_setpt    = 0; // output setpoint in millivolts
uint32_t  iout_limit    = 0; // output current limit in milliamps
uint32_t  vout_set_last = 0; // to store the last setpoint
uint16_t  duty_max      = 0; // max total duty cycle that would be required for this vout_setpt
uint16_t  duty_phase_1  = 0; // hi-side duty cycle for phase 1
uint16_t  rect_phase_1  = 0; // rectifier duty cycle for phase 1
uint16_t  duty_phase_2  = 0; // hi-side duty cycle for phase 2
uint16_t  rect_phase_2  = 0; // rectifier duty cycle for phase 2

int       v_sample_ind  = 0; // position of most recently added sample
uint32_t  v_samples[V_SAMPLE_DEPTH]; // voltage sample memory
int       i_sample_ind  = 0;
uint32_t  i_samples[I_SAMPLE_DEPTH];

bool      dcm           = false; // flag to indicate if we're in discontinuous conduction mode
int       i_pk          = 0; // peak inductor current

bool output_en          = false; // flag to indicate that user has enabled the output
bool last_output_en     = false; // stores last state of output_en to detect state change
bool phase_1_en         = false; // flag to enable phase 1
bool phase_2_en         = false; // flag to enable phase 2

int     cursor_ind          = 0; 
int     last_cursor_ind     = -1;
float   input_powers[]      = {10.0, 1.0, 0.1, 0.01, 10.0, 1.0, 0.1, 0.01};
uint8_t input_line[8];
char    lcd_buf[128];             // give ourselves space for goofing up
char    lcd_line_0[16];           // page 1 line 0
char    lcd_line_1[16];           // page 1 line 1
char    lcd_line_2[16];           // page 2 line 0
char    lcd_line_3[16];           // page 2 line 1
uint8_t enc_a_state         = 0;  // 
uint8_t enc_b_state         = 0;
bool    btn_a_pressed       = false;
bool    btn_b_pressed       = false;
bool    btn_c_pressed       = false;
bool    poll_adc            = false; // flag to tell the main loop to poll the ADC
int     poll_num            = 0; // tracks how many polls we've done
int 		encoder_counts 		  = 0;
int 		last_encoder_counts = 0;

AD7172_TypeDef adc = {
  .hspi = &hspi1,
  .cs_port = ADC_CS_GPIO_Port,
  .cs_pin = ADC_CS_Pin
};

// configs for setup 0
uint16_t adc_filt_0   = AD7172_ODR_15625; // 15.625 kSPS
uint16_t adc_if_mode  = AD7172_IOSTRENGTH | AD7172_DATA_STAT;
uint16_t adc_mode     = AD7172_REF_EN | AD7172_MODE_CONT_CONV;
uint32_t adc_offset_0 = 0; // will be set in calibration
uint32_t adc_gain_0   = 0; // will be set during calibration

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int clamp(int value, int lower, int upper);
uint16_t clamp16(uint16_t value, uint16_t lower, uint16_t upper);
float fclamp(float value, float lower, float upper);
int map(int x, int in_min, int in_max, int out_min, int out_max);
void debug(char* str);
void status_1_on();
void status_1_off();
void status_2_on();
void status_2_off();
void status_3_on();
void status_3_off();
bool poll_ad7172();
void increment_input_line();
void decrement_input_line();
void update_input_line();
void update_lcd();
void add_v_sample(uint32_t v_sample);
void add_i_sample(uint32_t i_sample);
float get_avg_voltage(int depth);
float get_avg_current(int depth);
float read_supply_channel();
float sense_12v_supply();
float sense_24v_supply();
float sense_36v_supply();
float sense_48v_supply();
float sense_vin();
void set_supply_mux(uint8_t input);
void enable_supply();
void disable_supply();
bool check_tolerance(float value, float target, float tolerance);
bool check_for_dcm(uint8_t phase);
void refactor_duty_cycle();
void enable_phase(uint8_t phase);
void disable_phase(uint8_t phase);
void update_phase_duty_cycle(uint8_t phase, uint16_t duty, uint16_t rect);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_HRTIM1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  htim2.Instance->CCR1 = 5000;
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim3);

  // ==== SETUP LCD ====
  LCD_Init();
  strcpy(lcd_line_0, "CWRUbotix!");
  LCD_Puts(0, 0, lcd_line_0);
  sprintf(lcd_line_2, " VSET | ILIM");
  sprintf(lcd_line_0, " VOUT | IOUT");

  // ==== SETUP ADC ====
  bool setup_success = true; // start true
  HAL_Delay(500); // delay for whatever
  
  debug("AD7172 reset ");
  if(setup_success = setup_success && ad7172_reset(&adc)){
    debug("SUCCEEDED\r\n");
  }else{
    debug("FAILED\r\n");
  }

  HAL_Delay(10); // short delay cuz why not
  debug("AD7172 ID read... ");
  if(setup_success = setup_success && ad7172_check_id(&adc)){
    debug("SUCCEEDED\r\n");
  }else{
    debug("FAILED\r\n");
  }
  sprintf(debug_buf, "ID: 0x%x\r\n", adc.id);
  debug(debug_buf);

  
  // CALIBRATE
  setup_success = setup_success && ad7172_calibrate(&adc); 
  

  // configure each ADC channel
  debug("AD7172 configure channels...\r\n");
  /* current sense channel */  
  setup_success = setup_success && ad7172_config_channel(&adc, I_SENSE_CH, AD7172_SETUP_SEL_0 | AD7172_AINPOS_AIN0 | AD7172_AINNEG_AIN1);
  /* voltage sense channel */
  setup_success = setup_success && ad7172_config_channel(&adc, VOUT_SENSE_CH, AD7172_SETUP_SEL_0 | AD7172_AINPOS_AIN2 | AD7172_AINNEG_AIN4);
  /* supply sense channel */
  setup_success = setup_success && ad7172_config_channel(&adc, VIN_SENSE_CH, AD7172_SETUP_SEL_0 | AD7172_AINPOS_AIN3 | AD7172_AINNEG_AIN4);

  // setup configuration
  debug("AD7172 configure setups...\r\n");
  setup_success = setup_success && ad7172_config_setup(&adc, 0, AD7172_UNIPOLAR | AD7172_REF_SEL_INT_2V5 | AD7172_AINBUF_P_EN);
  debug("AD7172 configure filters...\r\n");
  setup_success = setup_success && ad7172_config_filter(&adc, 0, AD7172_ODR_31250);

  // adc mode and interface mode configuration
  debug("AD7172 configure mode...\r\n");
  setup_success = setup_success && ad7172_config_adc_mode(&adc, AD7172_REF_EN | AD7172_MODE_CONT_CONV);
  setup_success = setup_success && ad7172_config_if_mode(&adc, AD7172_IOSTRENGTH | AD7172_DATA_STAT);
  
  if(setup_success){
    debug("SUCCEEDED\r\n");
    // status_1_on();
  }else{
    debug("FAILED\r\n");
    status_1_off();
    // hang
    while(!setup_success){
      status_1_on();status_2_on();status_3_on();
      HAL_Delay(500);
      status_1_off();status_2_off();status_3_off();
      HAL_Delay(500);
      debug("Setup failed. Press reset button...\r\n");
    }
  }
  voltage_12v = sense_12v_supply();
  voltage_24v = sense_24v_supply();
  voltage_36v = sense_36v_supply();
  voltage_48v = sense_48v_supply();
  sense_vin(); // sets the analog mux to VIN

  sprintf(debug_buf, "12V: %.2fV ", voltage_12v);
  debug(debug_buf);
  if(check_tolerance(voltage_12v, 12.5, 0.5)){ debug("PASSED\r\n"); 
  }else{ debug("FAILED\r\n"); 
  }

  sprintf(debug_buf, "24V: %.2fV ", voltage_24v);
  debug(debug_buf);
  if(check_tolerance(voltage_24v, 25.0, 1.0)){ debug("PASSED\r\n"); 
  }else{ debug("FAILED\r\n"); 
  }

  sprintf(debug_buf, "36V: %.2fV ", voltage_36v);
  debug(debug_buf);
  if(check_tolerance(voltage_36v, 37.5, 1.5)){ debug("PASSED\r\n"); 
  }else{ debug("FAILED\r\n"); 
  }

  sprintf(debug_buf, "48V: %.2fV ", voltage_48v);
  debug(debug_buf);
  if(check_tolerance(voltage_48v, 50.0, 2.0)){ debug("PASSED\r\n"); 
  }else{ debug("FAILED\r\n"); 
  }

  ad7172_enable_channel(&adc, VOUT_SENSE_CH);
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // ==== ADC THINGS ====
    if(poll_adc){
      poll_adc = !poll_ad7172(); // if poll was successful, flag will be reset
    }

    // ==== SETPOINT THINGS ====
    if(vout_setpt != vout_set_last){
      // check if we need switch relays
      last_relay = input_relay;
      if( vout_setpt <= (uint32_t)((voltage_12v - 1.0)*1000.0) ){
        input_relay = 1;
      }else if( vout_setpt <= (uint32_t)((voltage_24v - 1.0)*1000.0) ){
        input_relay = 2;
      }else if( vout_setpt <= (uint32_t)((voltage_36v - 1.0)*1000.0) ){
        input_relay = 3;
      }else{
        input_relay = 4;
      }
      if(input_relay > last_relay){
        disable_supply();
        HAL_Delay(8);
        set_supply_mux(input_relay);
        HAL_Delay(8);
        enable_supply();
        ad7172_enable_channel(&adc, VIN_SENSE_CH); // enable the VIN sense channel to be sure this gets updated soon
      }else if(input_relay < last_relay){
        disable_supply();
        HAL_Delay(100 * (last_relay - input_relay)); // wait for vin rail to drop
        set_supply_mux(input_relay);
        enable_supply();
        ad7172_enable_channel(&adc, VIN_SENSE_CH); // enable the VIN sense channel to be sure this gets updated soon
      }
    }

    // ==== OUTPUT ENABLE THINGS ====
    if(output_en != last_output_en){
      if(output_en){    
        // start up sequence
        status_3_on();
        enable_phase(1);
        enable_phase(2);
      }else{
        // we've just been shut down!
        status_3_off();
        disable_phase(1);
        disable_phase(2);
      }
    }
    last_output_en = output_en;

    // ==== LCD THINGS ====
    if(update_lcd_flag){
      update_lcd_flag = false;
      update_lcd();
      // status_3_off();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_HRTIM1|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Hrtim1ClockSelection = RCC_HRTIM1CLK_PLLCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  // encoder and button things
  // HAL_GPIO_TogglePin(STATUS_2_GPIO_Port, STATUS_2_Pin);
  
  if(GPIO_Pin == ENC_A_Pin){
    if(!HAL_GPIO_ReadPin(ENC_B_GPIO_Port, ENC_B_Pin)){
      increment_input_line();
    }
  }else if(GPIO_Pin == ENC_B_Pin){
    if(!HAL_GPIO_ReadPin(ENC_A_GPIO_Port, ENC_A_Pin)){
      decrement_input_line();
    }
  }else if(GPIO_Pin == BTN_A_Pin){
    // if knob button (enter) was pressed
    if(cursor_ind < 0){
      // we're on home screen, so toggle enabled
      output_en = !output_en;
    }else if(cursor_ind < 4){
      vout_set_last = vout_setpt;
      vout_setpt = (uint32_t)(user_vset*1000.0);
      cursor_ind = -1;
      output_en = true;
    }else{
      iout_limit = (uint32_t)(user_ilim*1000.0);
      cursor_ind = -1;
      output_en = true;
    }
    
  }else if(GPIO_Pin == BTN_B_Pin){
    btn_b_pressed = true;
    cursor_ind--;
  }else if(GPIO_Pin == BTN_C_Pin){
    btn_c_pressed = true;
    cursor_ind++;
  }else{
    // undefined case, do nothing
  }
}

/**
 * @brief callback to trigger polling from the ADC or trigger update of LCD
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if(htim->Instance == TIM3){
    // time to read from the ADC
    poll_adc = true;
      
  }else if(htim->Instance == TIM6){
    // HAL_GPIO_TogglePin(STATUS_3_GPIO_Port, STATUS_3_Pin);
    // sprintf(debug_buf, "Duty cycle: %d\r\n", duty_phase_1);
    // debug(debug_buf);
    update_lcd_flag = true;
  }
}

/**
 * @brief clamps value between upper and lower
 */
uint16_t clamp16(uint16_t value, uint16_t lower, uint16_t upper){
  return (value > upper ? upper : (value < lower ? lower : value));
}

/**
 * @brief clamps value between lower and upper 
 */
int clamp(int value, int lower, int upper){
  return (value > upper ? upper : (value < lower ? lower : value));
}

/**
 * @brief clamps value between lower and upper
 */
float fclamp(float value, float lower, float upper){
  return (value > upper ? upper : (value < lower ? lower : value));
}

int map(int x, int in_min, int in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void debug(char* str){
  HAL_UART_Transmit(&huart1, str, strlen(str), 10);
}

void status_1_on(){
  HAL_GPIO_WritePin(STATUS_1_GPIO_Port, STATUS_1_Pin, GPIO_PIN_SET);
}
void status_1_off(){
  HAL_GPIO_WritePin(STATUS_1_GPIO_Port, STATUS_1_Pin, GPIO_PIN_RESET);
}
void status_2_on(){
  HAL_GPIO_WritePin(STATUS_2_GPIO_Port, STATUS_2_Pin, GPIO_PIN_SET);
}
void status_2_off(){
  HAL_GPIO_WritePin(STATUS_2_GPIO_Port, STATUS_2_Pin, GPIO_PIN_RESET);
}
void status_3_on(){
  HAL_GPIO_WritePin(STATUS_3_GPIO_Port, STATUS_3_Pin, GPIO_PIN_SET);
}
void status_3_off(){
  HAL_GPIO_WritePin(STATUS_3_GPIO_Port, STATUS_3_Pin, GPIO_PIN_RESET);
}

bool poll_ad7172(){
  if(ad7172_read_conv(&adc, 10)){
    switch(adc.status & 0x03){
    case VOUT_SENSE_CH:{
      // HAL_GPIO_TogglePin(STATUS_1_GPIO_Port, STATUS_1_Pin);
      vout_meas_raw = ad7172_conv_to_mv(adc.conversions[VOUT_SENSE_CH]);
      vout = V_SENSE_MV(vout_meas_raw);
      add_v_sample(vout);
      if(output_en){
        refactor_duty_cycle(); // this should be synchronous with ADC updates
        update_phase_duty_cycle(1, duty_phase_1, rect_phase_1);
        update_phase_duty_cycle(2, duty_phase_2, rect_phase_2);
      }
      break;}
    case I_SENSE_CH:{
      iout_meas_raw = ad7172_conv_to_mv(adc.conversions[I_SENSE_CH]);
      iout = I_OUT_MA(iout_meas_raw);
      add_i_sample(iout);
      ad7172_disable_channel(&adc, I_SENSE_CH); // disable channel until another I_SAMPLE_RATE voltage samples have passed
      break;}
    case VIN_SENSE_CH:{
      vin_meas_raw = ad7172_conv_to_mv(adc.conversions[VIN_SENSE_CH]);
      vin = VIN_SENSE_MV(vin_meas_raw);
      ad7172_disable_channel(&adc, VIN_SENSE_CH); // disable channel until another VIN_SAMPLE_RATE voltage samples have passed
      break;}
    default:{
      break;}
    }
    poll_num++;
    if(poll_num % I_SAMPLE_RATE == 0){
      ad7172_enable_channel(&adc, I_SENSE_CH); // enable current sense channel every so often
    }
    if(poll_num % VIN_SAMPLE_RATE == 0){
      ad7172_enable_channel(&adc, VIN_SENSE_CH); // enable supply sense channel every now and then
    }
  }
}

/**
 * @brief to be called to increment voltage or current based on LCD cursor position
 */
void increment_input_line(){
  if(cursor_ind < 0){
    return;
  }else if(cursor_ind < 4){
    // voltage
    user_vset += input_powers[cursor_ind];
    user_vset = fclamp(user_vset, 0.0, VSET_MAX);
  }else if(cursor_ind < 8){
    // current
    user_ilim += input_powers[cursor_ind];
    user_ilim = fclamp(user_ilim, 0.0, 30.0);
  }else{
    return;
  }
}

/**
 * @brief to be called to decrement voltage or current based on LCD cursor position
 */
void decrement_input_line(){
  if(cursor_ind < 0){
    return;
  }else if(cursor_ind < 4){
    // voltage
    user_vset -= input_powers[cursor_ind];
    user_vset = fclamp(user_vset, 0.0, VSET_MAX);
  }else if(cursor_ind < 8){
    // current
    user_ilim -= input_powers[cursor_ind];
    user_ilim = fclamp(user_ilim, 0.0, 30.0);
  }else{
    return;
  }
}

/**
 * @brief Call periodically to display info on LCD
 */
void update_lcd(){
  cursor_ind = clamp(cursor_ind, -1, 7);

  if(cursor_ind < 0){
    // display status page
    float v_avg = get_avg_voltage(5)/1000.0;
    float i_avg = get_avg_current(3)/1000.0;

    LCD_CursorOff();

    int offset = 0;
    if(v_avg < 10.0){
      offset = sprintf(lcd_buf, " %3.2f |", v_avg);
    }else{
      offset = sprintf(lcd_buf, "%4.2f |", v_avg);
    }if(i_avg < 10.0){
      offset += sprintf(lcd_buf + offset, " %3.2f ", i_avg);
    }else{
      offset += sprintf(lcd_buf + offset, "%4.2f ", i_avg);
    }

    if(output_en){
      sprintf(lcd_buf + offset, " ON");
    }else{
      sprintf(lcd_buf + offset, "OFF");
    }
    if(last_cursor_ind >= 0){
      LCD_Puts(0,0,lcd_line_0);
    }
    LCD_Puts(0,1,lcd_buf);
    LCD_CursorOn();
    LCD_CursorSet(13, 1);

  }else{
    // display settings page
    
    LCD_CursorOff();
    int offset = 0;
    if(user_vset < 10.0){
      offset = sprintf(lcd_buf, "0%3.2f | ", user_vset);
    }else{
      offset = sprintf(lcd_buf, "%4.2f | ", user_vset);
    }if(user_ilim < 10.0){
      sprintf(lcd_buf + offset, "0%3.2f   ", user_ilim);
    }else{
      sprintf(lcd_buf + offset, "%4.2f   ", user_ilim);
    }

    if(last_cursor_ind < 0){
      LCD_Puts(0,0,lcd_line_2);
    }
    
    LCD_Puts(0,1,lcd_buf);
    uint8_t cols[] = {0, 1, 3, 4, 8, 9, 11, 12};
    LCD_CursorOn();
    LCD_CursorSet(cols[cursor_ind], 1);

  }
  last_cursor_ind = cursor_ind;
  
}

/**
 * @brief adds v_sample to v_samples. increments v_sample_ind & wraps once v_sample_ind >= V_SAMPLE_DEPTH
 * @param sample to add to v_samples at next v_sample_ind
 */
void add_v_sample(uint32_t v_sample){
  v_sample_ind++;
  if(v_sample_ind >= V_SAMPLE_DEPTH){
    v_sample_ind = 0;
  }
  v_samples[v_sample_ind] = v_sample;
}


void add_i_sample(uint32_t i_sample){
  i_sample_ind++;
  if(i_sample_ind >= I_SAMPLE_DEPTH){
    i_sample_ind = 0;
  }
  i_samples[i_sample_ind] = i_sample;
}

/**
 * @brief computes the average of v_samples at the specified sample depth
 * @param depth number of previous samples over which to compute the average
 * @return arithmatic mean of "depth" many samples from v_samples
 */
float get_avg_voltage(int depth){
  if(depth > V_SAMPLE_DEPTH){
    depth = V_SAMPLE_DEPTH;
  }
  float weight = 1.0/depth;
  float avg = 0.0;
  bool done = false;
  int i = v_sample_ind;
  int count = 0;
  while(count < depth){
    avg += weight * ((float)v_samples[i]);
    count++;
    i--;
    if(i < 0){
      i = V_SAMPLE_DEPTH - 1;
    }
  }
  return avg;
}

/**
 * @brief computes the average of i_samples at the specified sample depth
 * @param depth number of previous samples over which to compute the average
 * @return arithmatic mean of "depth" many samples from i_samples
 */
float get_avg_current(int depth){
  if(depth > I_SAMPLE_DEPTH){
    depth = I_SAMPLE_DEPTH;
  }
  float weight = 1.0/depth;
  float avg = 0.0;

  // start at the most recent sample and work backwards
  int i = i_sample_ind;
  int count = 0;
  while(count < depth){
    avg += weight * ((float)i_samples[i]);
    count++;
    i--;
    if(i < 0){ // if we've hit the beginning
      i = I_SAMPLE_DEPTH - 1; // wrap around to the end
    }
  }
  return avg;
}

/**
 * @brief reads from the ADC the channel for supply voltage
 * @return the voltage read from the supply channel or -1.0 in case of failure
 */
float read_supply_channel(){
  ad7172_enable_channel(&adc, 2);
  HAL_Delay(1); // wait for a few conversions to occur
  int readings = 0;
  int timeout = 0;
  uint32_t results[4];

  while(readings < 4 && timeout < 100){
    if(ad7172_read_conv(&adc, 161) && (adc.status & 0x03) == 2 ){
      results[readings] = adc.conversions[2];
      readings++;
    }
    HAL_Delay(1);
    timeout++;
  }
  ad7172_disable_channel(&adc, 2);

  if (timeout < 100){
    // we gucci
    float adc_reading = 0;
    for(int i = 0; i < 4; i++){
      adc_reading += 0.25*results[i];
    }
    return SUPPLY_SENSE_DIV * ad7172_conv_to_voltage(adc_reading);
  }else{
    // hmmm suspicious
    return -1.0;
  }
}

float sense_12v_supply(){
  HAL_GPIO_WritePin(ANLG_MUX_0_GPIO_Port, ANLG_MUX_0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ANLG_MUX_1_GPIO_Port, ANLG_MUX_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ANLG_MUX_2_GPIO_Port, ANLG_MUX_2_Pin, GPIO_PIN_RESET);
  HAL_Delay(5); // wait for the output of the mux stabilizes
  return read_supply_channel();
}
float sense_24v_supply(){
  HAL_GPIO_WritePin(ANLG_MUX_0_GPIO_Port, ANLG_MUX_0_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ANLG_MUX_1_GPIO_Port, ANLG_MUX_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ANLG_MUX_2_GPIO_Port, ANLG_MUX_2_Pin, GPIO_PIN_RESET);
  HAL_Delay(5); // wait for the output of the mux stabilizes
  return read_supply_channel();
}
float sense_36v_supply(){
  HAL_GPIO_WritePin(ANLG_MUX_0_GPIO_Port, ANLG_MUX_0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ANLG_MUX_1_GPIO_Port, ANLG_MUX_1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ANLG_MUX_2_GPIO_Port, ANLG_MUX_2_Pin, GPIO_PIN_RESET);
  HAL_Delay(5); // wait for the output of the mux stabilizes
  return read_supply_channel();
}
float sense_48v_supply(){
  HAL_GPIO_WritePin(ANLG_MUX_0_GPIO_Port, ANLG_MUX_0_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ANLG_MUX_1_GPIO_Port, ANLG_MUX_1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ANLG_MUX_2_GPIO_Port, ANLG_MUX_2_Pin, GPIO_PIN_RESET);
  HAL_Delay(5); // wait for the output of the mux stabilizes
  return read_supply_channel();
}
float sense_vin(){
  HAL_GPIO_WritePin(ANLG_MUX_0_GPIO_Port, ANLG_MUX_0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ANLG_MUX_1_GPIO_Port, ANLG_MUX_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ANLG_MUX_2_GPIO_Port, ANLG_MUX_2_Pin, GPIO_PIN_SET);
  HAL_Delay(5); // wait for the output of the mux stabilizes
  return read_supply_channel();
}


/**
 * @brief writes value of input to the relay mux select inputs. Doesn't enable mux
 */
void set_supply_mux(uint8_t input){
  HAL_GPIO_WritePin(MUX_A_GPIO_Port, MUX_A_Pin, input & 1);
  HAL_GPIO_WritePin(MUX_B_GPIO_Port, MUX_B_Pin, (input >> 1) & 1);
  HAL_GPIO_WritePin(MUX_C_GPIO_Port, MUX_C_Pin, (input >> 2) & 1);
}

/**
 * @brief enables input relay mux
 */
void enable_supply(){
  // HAL_GPIO_WritePin(MUX_G1_GPIO_Port, MUX_G1_Pin, GPIO_PIN_SET);
  // HAL_GPIO_WritePin(MUX_G2A_GPIO_Port, MUX_G2A_Pin, GPIO_PIN_RESET);
  // HAL_GPIO_WritePin(MUX_G2B_GPIO_Port, MUX_G2B_Pin, GPIO_PIN_RESET);
}

/**
 * @brief disables input relay mux
 */
void disable_supply(){
  HAL_GPIO_WritePin(MUX_G1_GPIO_Port, MUX_G1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MUX_G2A_GPIO_Port, MUX_G2A_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MUX_G2B_GPIO_Port, MUX_G2B_Pin, GPIO_PIN_SET);
}


bool check_tolerance(float value, float target, float tolerance){
  return value < (target + tolerance) && value > (target - tolerance);
}

/**
 * @brief calculate new duty cycle based on vout_setpt and vout
 */
void refactor_duty_cycle(){
  last_verr = verr;
  verr = (int)vout_setpt - (int)vout;

  // if we're close enough to the setpoint, don't try to correct it
  if(abs(verr) < VOUT_ERROR_MARGIN){return;}

  n_duty_max = (int)((TIMER_PERIOD*vout_setpt*100)/(90*vin));
  verr_integ = clamp(verr_integ + verr, 0, n_duty_max/ki);
  int p = kp*verr;
  int i = ki*verr_integ;
  int d = kd*(verr - last_verr);
  int duty = p + i + d;
  duty = clamp(duty, 0, n_duty_max);

  duty_phase_1 = (uint16_t)duty/2;
  duty_phase_2 = duty_phase_1;

  // uint32_t delta_i = ((vin - vout)*duty_cycle)/L_NH;
  // uint32_t rect_cts = (L_NH * delta_i)/vout;
  rect_phase_1 = (uint16_t)((vin-vout)*duty_phase_1)/vout;
  rect_phase_2 = rect_phase_1;

}

void enable_phase(uint8_t phase){
  phase &= 0x03;
  if(phase == 1){
    debug("Enabling phase 1\r\n");
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
    HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_TIMER_A);
  }else if(phase == 2){
    debug("Enabling phase 2\r\n");
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
    HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_TIMER_B);
  }
}

void disable_phase(uint8_t phase){
  phase &= 0x03;
  if(phase == 1){
    debug("Disabling phase 1\r\n");
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
    HAL_HRTIM_WaveformCountStop(&hhrtim1, HRTIM_TIMERID_TIMER_A);
  }else if(phase == 2){
    debug("Disabling phase 2\r\n");
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
    HAL_HRTIM_WaveformCountStop(&hhrtim1, HRTIM_TIMERID_TIMER_B);
  }
}

/**
 * @brief 
 * @param phase either 1 or 2
 * @param duty counts of time the high side should be active for
 * @param rect counts of time the rectifier side should be active for
 */
void update_phase_duty_cycle(uint8_t phase, uint16_t duty, uint16_t rect){
  /* Set the compare value */
  phase &= 0x03;
  hhrtim1.Instance->sTimerxRegs[phase].CMP1xR = duty;
  hhrtim1.Instance->sTimerxRegs[phase].CMP2xR = duty + DEADTIME_COUNTS; // insert deadtime
  hhrtim1.Instance->sTimerxRegs[phase].CMP3xR = clamp16(duty + DEADTIME_COUNTS + rect, duty + DEADTIME_COUNTS + 1, TIMER_PERIOD-DEADTIME_COUNTS);
  if(phase == 1){
    // set master timer compare 1 which triggers phase 2 if it's enabled
    hhrtim1.Instance->sMasterRegs.MCMP1R = duty;
    // hhrtim1.Instance->sTimerxRegs[0].CMP1xR = duty;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
