/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "adc.h"
#include "communication.h"
#include "parsing.h"
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define RX_BUF_SIZE 1
#define BUF_SIZE 512

Communication comms; // Defined Globally so that it can be used in the UART ISR

uint8_t SERIAL_RX_BUF[RX_BUF_SIZE];
uint8_t WIFI_RX_BUF[RX_BUF_SIZE];
uint8_t UART_BUFFER[BUF_SIZE] = {'\0'};
uint32_t bufferIndex = 0;
uint32_t bufferCount = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

  if ((huart == &huart1)) {

    if (bufferCount < BUF_SIZE) {

      if (bufferCount == 0) {
        bufferIndex = 0;
      }

      UART_BUFFER[bufferIndex] = WIFI_RX_BUF[0];
      bufferIndex = (bufferIndex + 1) % BUF_SIZE;
      bufferCount++;
    }

    if (WIFI_RX_BUF[0] == '\n') {

      comms.receivedMsgs++; // Increment new message counter
    }

    WIFI_RX_BUF[0] = '\0';

  } else if ((huart == &huart3)) {

    if (bufferCount < BUF_SIZE) {

      if (bufferCount == 0) {
        bufferIndex = 0;
      }

      UART_BUFFER[bufferIndex] = SERIAL_RX_BUF[0];
      bufferIndex = (bufferIndex + 1) % BUF_SIZE;
      bufferCount++;
    }

    if (SERIAL_RX_BUF[0] == '\n') {

      comms.receivedMsgs = 1; // Set new message flag
    }

    SERIAL_RX_BUF[0] = '\0';
  }

  if (bufferCount >=
      BUF_SIZE) { // If the buffer is full clear it and reset variables

    memset(UART_BUFFER, '\0', BUF_SIZE);
    bufferCount = 0;
    bufferIndex = 0;
  }

  if (huart ==
      &huart1) { // Calls the interrupt for the huart current instance of huart

    HAL_UART_Receive_IT(comms.wifi, WIFI_RX_BUF, RX_BUF_SIZE);

  } else {

    HAL_UART_Receive_IT(comms.serial, SERIAL_RX_BUF, RX_BUF_SIZE);
  }
}

/**
 * Reads up to '\n' UART_BUFFER and extracts the ID and data from the line.
 * A line is a string of characters up to and including a '\n'. This function
 * clears UART_BUFFER as it goes.
 *
 * Parameters:
 * 	id <char*>: the character array that will hold the ID read from the
 * line. data <char*>: the character array that will hold the data read from the
 * the line.
 */
void read_line_serial(char *id, char *data) {
  char c = '\0';
  int buffIndex = 0;
  int lineIndex = 0;
  uint8_t colon = 0;
  while (buffIndex < BUF_SIZE) {

    c = UART_BUFFER[buffIndex];
    UART_BUFFER[buffIndex] = '\0';

    if (c != '\0') {

      if (c == ':') {
        colon = 1;
        id[lineIndex] = '\0';
        lineIndex = 0;
        buffIndex++;
        continue;

      } else if (c == '\n') {
        data[lineIndex] = '\0';
        data[++lineIndex] = '\n';

        break;
      }

      if (colon == 0) {
        id[lineIndex] = c;
        lineIndex++;

      } else if (colon == 1) {
        data[lineIndex] = c;
        lineIndex++;
      }
    }
    buffIndex++;
    bufferCount--;
  }
}

/**
 * Reads up to a '\n' in UART_BUFFER and saves the line to 'line' clearing the
 * UART_BUFFER as it goes. All '\0' are ignored when they are read.
 *
 * Parameters:
 * 	line <char*>: the character array that will hold the read line.
 */
void read_line_wifi(char *line) {
  char c = '\0';
  int buffIndex = 0;
  int lineIndex = 0;

  while (buffIndex < BUF_SIZE && bufferCount > 0) {

    c = UART_BUFFER[buffIndex];
    UART_BUFFER[buffIndex] = '\0';

    if (c == '\r') {

      UART_BUFFER[buffIndex + 1] = '\0';
      bufferCount--;
      line[lineIndex] = '\0';
      break;

    } else if (c == '\n') {

      line[lineIndex] = '\0';
      bufferCount--;
      break;
    }

    if (c != '\0') {

      line[lineIndex] = c;
      lineIndex++;
      bufferCount--;
    }

    buffIndex++;
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_CRC_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* Pulsing power LED */
  for (int i = 1; i < 13; i++) {
    if (i % 2 == 0) {

      HAL_GPIO_WritePin(PWR_LED_GPIO_Port, PWR_LED_Pin, 1);

    } else {

      HAL_GPIO_WritePin(PWR_LED_GPIO_Port, PWR_LED_Pin, 0);
    }

    HAL_Delay(125);
  }

  // initiate landscape in landscape orientation and init backlight
  Displ_Init(Displ_Orientat_270);
  Displ_CLS(BLACK);
  Displ_BackLight('I');
  Displ_BackLight('1');

  // 5 different modes of operation
  const char *mode[5];
  mode[0] = "Mode: Weigh";
  mode[1] = "Mode: Tare";
  mode[2] = "Mode: Pass/Fail";
  mode[3] = "Mode: Calibration";
  mode[4] = "Mode: Counting";

  // Draw Buttons
  Displ_FillArea(0, 0, 160, 43, CYAN);
  Displ_FillArea(0, 43, 160, 85, YELLOW);
  Displ_Border(4, 4, 30, 14, 1, BLACK);
  Displ_WString(5, 5, "Tare", Font12, 1, BLUE, RED);
  Displ_Border(44, 4, 37, 14, 1, BLACK);
  Displ_WString(45, 5, "Weigh", Font12, 1, BLUE, WHITE);
  Displ_Border(94, 4, 65, 14, 1, BLACK);
  Displ_WString(95, 5, "Pass/Fail", Font12, 1, BLUE, WHITE);
  Displ_Border(94, 24, 58, 14, 1, BLACK);
  Displ_WString(95, 25, "Counting", Font12, 1, BLUE, WHITE);
  Displ_Border(4, 24, 65, 14, 1, BLACK);
  Displ_WString(5, 25, "Calibrate", Font12, 1, BLUE, WHITE);

  // Draw Main Area
  Displ_Border(4, 48, 130, 61, 1, BLACK);
  Displ_FillArea(5, 50, 128, 58, WHITE);

  // Draw Brightness Buttons
  Displ_WString(141, 68, "}", Font20, 1, D_GREEN,
                YELLOW); // increase brightness button
  Displ_WString(141, 48, "}", Font20, 1, RED,
                YELLOW); // decrease brightness button

  // Draw Wifi and Serial Buttons
  Displ_WString(141, 108, "{", Font20, 1, RED, YELLOW);    // wifi symbol
  Displ_WString(141, 88, "~", Font20, 1, D_GREEN, YELLOW); // serial symbol
  Displ_Border(133, 68, 30, 20, 1, BLACK);
  Displ_Border(133, 48, 30, 21, 1, BLACK);
  Displ_Border(133, 87, 30, 22, 1, BLACK);
  Displ_Border(133, 108, 30, 22, 1, BLACK);

  // Loading at start up
  Displ_WString(12, 64, "Loading...", Font16, 1, BLACK, WHITE);

  // Variables for general weigh mode
  char weightStr[10];
  float weight = 0.0;
  int weightOverLim = 0;

  // Variables for pass fail mode
  int lowerThreshSet = 0;
  char lowerThreshStr[6];
  int lowerIndex = 0;
  char higherThreshStr[6];
  int higherIndex = 0;
  int lowerHigherSet = 0; // Lower and Upper values not set

  // Variables for touch screen
  int index = 0;      // used to remove noisy touches
  int touchIndex = 0; // determines when the touch is registered
  int previousX;
  int previousY;
  int previousXX;
  int previousYY;
  int x_coord;
  int y_coord;

  // Variables for counting mode
  float countingRefNum = 0.0;
  char countingRefNumStr[5];
  char countingRefNumStrTwo[5];
  int countingIndex = 0;
  float countInt = 0;
  char countStr[4];
  int countScreenSet = 0;
  int reset = 0;
  float refWeightInt = 0.0;

  // Variables for calibration mode
  int calibrationStep = 0;
  int calibrateFirstIndex = 0;
  char calibrateFirstStr[4];
  int calibrateFirstNum = 0; // weight of first calibration object in grams
  int calibrateSecondIndex = 0;
  char calibrateSecondStr[4];
  int calibrateSecondNum = 0; // weight of second calibration object in grams
  int calibrationSet = 0;
  char adcValStr[6];

  // Used to clear up screen in case of error
  int iconReset = 0;

  comms.wifi = &huart1;
  comms.serial = &huart3;
  comms.i2c = &hi2c1;
  comms.receivedMsgs = 0;
  comms.commsMedium = 0;

  /* ADC inits and setup */

  init_adc(&hi2c1);
  config_adc(&hi2c1);
  start_adc(&hi2c1);
  calibrate_adc(&hi2c1);

  init_wifi(comms.wifi);
  listen_wifi(comms.wifi);

  HAL_UART_Receive_IT(comms.wifi, WIFI_RX_BUF, RX_BUF_SIZE);
  HAL_UART_Receive_IT(comms.serial, SERIAL_RX_BUF, RX_BUF_SIZE);

  int32_t adcVal;
  uint32_t prevTime = 0;
  float massVal;
  float countVal;
  char adc[10] = {'\0'};
  char mass[15] = {'\0'};
  char count[15] = {'\0'};
  char id[20] = {'\0'};
  char line[60] = {'\0'};
  char data[50] = {'\0'};
  char msg[60] = {'\0'};

  MassCalcVals formula;

  formula.mode = 0;
  formula.tareOn = 0;
  formula.tareAdc = 0;
  formula.tareMass = 0.0;
  formula.grad = 0.0;
  formula.yInt = 0;
  formula.countingSet = 0;

  HAL_FLASH_Unlock();
  EE_Init(EE_FORCED_ERASE);
  HAL_FLASH_Lock();

  uint32_t memData = 0;
  HAL_FLASH_Unlock();
  EE_ReadVariable16bits(LOWER_THRESH, &(formula.lowerThresh));
  EE_ReadVariable16bits(UPPER_THRESH, &(formula.upperThresh));

  EE_ReadVariable32bits(GRAD_UPPER, &memData);
  formula.grad = (((float)memData) / 1000000000);

  HAL_FLASH_Lock();

  uint16_t highThresh = formula.lowerThresh;
  uint16_t lowThresh = formula.upperThresh;

  formula.zeroValue = mean_adc(&hi2c1, 32);
  HAL_Delay(500);
  formula.zeroValue = mean_adc(&hi2c1, 32);
  formula.yInt =
      formula.grad *
      formula.zeroValue; // Converting ADC value to mass using gradient

  HAL_GPIO_WritePin(PWR_LED_GPIO_Port, PWR_LED_Pin, 1);

  int referenceNumber = 0;

  send_mode(&comms, &formula); // Tell the GUI the current mode of the system.
  send_tare(&comms,
            &formula); // Tell the GUI if a tare is active and what it is.
  send_pass_fail(
      &comms, &formula); // Tell the GUI the current lower and upper thresholds
  send_zero_value(
      &comms, &formula); // Tell the GUI the ADC value corresponding to 0 grams.

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {

    // Weight readings from scale
    adcVal = mean_adc(&hi2c1, 1);
    massVal = calc_mass(&formula, adcVal);

    int_to_string(adc, adcVal);
    float_to_string(mass, massVal);

    /* RX GUI communication */
    if (comms.receivedMsgs == 1 && comms.commsMedium == 0) {
      read_line_serial(id, data);

      parse_msg(&comms, id, data, &hi2c1, &formula);

      memset(id, '\0', strlen(id));
      memset(data, '\0', strlen(data));

      bufferIndex = 0;
      bufferCount = 0;

      comms.receivedMsgs = 0;

    } else if (comms.receivedMsgs > 0 && comms.commsMedium == 1) {

      while (comms.receivedMsgs > 0) {

        read_line_wifi(line);

        if (get_msg(line, msg, id, data) == HAL_OK) {

          parse_msg(&comms, id, data, &hi2c1, &formula);
        }

        memset(line, '\0', strlen(line));
        memset(msg, '\0', strlen(msg));
        memset(id, '\0', strlen(id));
        memset(data, '\0', strlen(data));

        comms.receivedMsgs--;
      }
    }

    /* TX GUI communication */
    /* This check decreases the frequency of outgoing WiFi messages so that the
     * ESP32-C3 has enough time to handle each message.
     */
    if (((HAL_GetTick() - prevTime > 100) && comms.commsMedium == WIFI) ||
        comms.commsMedium == WIRED_SERIAL) {

      if ((massVal + formula.tareMass) >
          2100) { // Checking if weight on scale is over 2.1kg

        send_msg(&comms, "OL", "1");

      } else if (formula.mode == 2) { // Pass Fail Mode

        if (formula.lowerThresh < massVal && massVal < formula.upperThresh) {

          send_msg(&comms, PASS_VALUE, "1");

        } else {

          send_msg(&comms, PASS_VALUE, "0");
        }

        send_msg(&comms, ADC, adc);
        send_msg(&comms, MASS, mass);

      } else if (formula.mode == 4 &&
                 formula.countingSet == 2) { // Counting Mode

        countVal = massVal / formula.unitMass;
        float_to_string(count, countVal);

        send_msg(&comms, COUNT_VALUE, count);
        send_msg(&comms, ADC, adc);
        send_msg(&comms, MASS, mass);

      } else { // General Weighing Mode

        send_msg(&comms, ADC, adc);
        send_msg(&comms, MASS, mass);
      }

      prevTime = HAL_GetTick();
    }

    // Always Display Current Mode
    Displ_WString(5, 110, mode[formula.mode], Font12, 1, BLUE, YELLOW);

    // Turn ints and floats into strings to display on screen
    sprintf(weightStr, "%0.1fg", weight);
    sprintf(lowerThreshStr, "%dg", formula.lowerThresh);
    sprintf(higherThreshStr, "%dg", formula.upperThresh);
    referenceNumber = countingRefNum; // convert count number from float to int
    sprintf(countingRefNumStr, "%d", referenceNumber);
    sprintf(countingRefNumStrTwo, "%d", referenceNumber);
    sprintf(countStr, "%0.1f", countInt);
    sprintf(calibrateFirstStr, "%ig", calibrateFirstNum);
    sprintf(calibrateSecondStr, "%ig", calibrateSecondNum);
    sprintf(adcValStr, "%ld", adcVal);

    weight = massVal;

    // Checking if there is an active tare
    if (formula.tareAdc > 0) {
      Displ_WString(5, 5, "Tare", Font12, 1, BLUE, D_GREEN); // turn button
                                                             // green
    } else {
      Displ_WString(5, 5, "Tare", Font12, 1, BLUE, RED); // turn button green
    }

    // Reset icons in case of bug
    if (iconReset == 10) {
      Displ_FillArea(133, 48, 27, 41, YELLOW);
      Displ_WString(141, 68, "}", Font20, 1, D_GREEN,
                    YELLOW); // increase brightness button
      Displ_WString(141, 48, "}", Font20, 1, RED,
                    YELLOW); // decrease brightness button
      Displ_Border(133, 68, 30, 20, 1, BLACK);
      Displ_Border(133, 48, 30, 21, 1, BLACK);
      Displ_Border(4, 48, 130, 61, 1, BLACK);
      iconReset = 0;
    }

    // update tare weight to be set
    if (formula.mode == 1) {

      // 2 digits e.g (8.4g)
      if (weight > 0 && weight < 10) {
        Displ_FillArea(55, 50, 22, 25, WHITE);
        Displ_WString(10, 55, weightStr, Font16, 1, BLACK, WHITE);
      }

      // 3 digits e.g (50.3g, -9.3g)
      if ((weight < 0 && weight > -10) || (weight > 9 && weight < 100)) {
        Displ_FillArea(65, 50, 22, 25, WHITE);
        Displ_WString(10, 55, weightStr, Font16, 1, BLACK, WHITE);
      }

      // 4 digits e.g (100.5g, -55.3g)
      if ((weight > 99 && weight < 1000) || (weight < -9 && weight > -100)) {
        Displ_FillArea(75, 50, 12, 25, WHITE);
        Displ_WString(10, 55, weightStr, Font16, 1, BLACK, WHITE);
      }

      // 5 digits e.g (1500.4g, -950.3g)
      if ((weight > 999 && weight < 2100) || (weight < -99 && weight > -1000)) {
        Displ_WString(10, 55, weightStr, Font16, 1, BLACK, WHITE);
      }

      if (weight > 2100) {
        Displ_FillArea(33, 50, 55, 25, WHITE);
        Displ_WString(10, 55, "OL", Font16, 1, BLACK, WHITE);
      }
    }

    if (comms.commsMedium == 0) {                           // Serial
      Displ_WString(141, 108, "{", Font20, 1, RED, YELLOW); // wifi symbol red
      Displ_WString(141, 88, "~", Font20, 1, D_GREEN,
                    YELLOW); // serial symbol green
      Displ_Border(133, 108, 30, 22, 1, BLACK);
    }

    if (comms.commsMedium == 1) { // Wifi
      Displ_WString(141, 108, "{", Font20, 1, D_GREEN,
                    YELLOW);                               // wifi symbol green
      Displ_WString(141, 88, "~", Font20, 1, RED, YELLOW); // serial symbol red
      Displ_Border(133, 108, 30, 22, 1, BLACK);
    }

    // Check if weight reading is over limit
    if (massVal + formula.tareMass > 2100) {
      weightOverLim = 1;
    } else {
      weightOverLim = 0;
    }

    // General weigh mode, no over limit reading
    if (formula.mode == 0 && weightOverLim == 0) {

      // 2 digits wide e.g (1.1g)
      if (weight > 0 && weight < 10) {
        Displ_FillArea(79, 50, 51, 55, WHITE);
        Displ_WString(12, 60, weightStr, Font24, 1, BLACK, WHITE);
      }

      // 3 digits wide (e.g 10.5g, -9.4g)
      if ((weight > 9 && weight < 100) || (weight < 0 && weight > -10)) {
        Displ_FillArea(97, 50, 33, 55, WHITE);
        Displ_WString(12, 60, weightStr, Font24, 1, BLACK, WHITE);
      }

      // 4 digits wide (e.g 100.9g, -15.3g)
      if ((weight > 99 && weight < 1000) || (weight < -9 && weight > -100)) {
        Displ_FillArea(115, 50, 15, 55, WHITE);
        Displ_WString(12, 60, weightStr, Font24, 1, BLACK, WHITE);
      }

      // 5 digits wide (e.g 1250.3g, -567.4g)
      if (((weight > 999) && (weight < 2100)) ||
          ((weight > -1000) && (weight < -99))) {
        Displ_WString(12, 60, weightStr, Font24, 1, BLACK, WHITE);
      }

      if ((weight < -999) && (weight > -2100)) { // 6 digits wide (e.g -1500.3g)
        Displ_FillArea(100, 50, 30, 55, WHITE);
        Displ_WString(12, 67, weightStr, Font16, 1, BLACK, WHITE);
      }
    }

    // General weigh mode, over limit detected
    if (formula.mode == 0 && weightOverLim == 1) {
      Displ_FillArea(5, 80, 110, 20, WHITE);
      Displ_FillArea(110, 50, 20, 14, WHITE);
      Displ_FillArea(120, 63, 10, 20, WHITE);
      Displ_WString(12, 64, "Over Limit", Font16, 1, BLACK, WHITE);
    }

    // Pass fail mode, setting lower threshold
    if (formula.mode == 2 && lowerThreshSet == 1 && lowerHigherSet == 0) {
      Displ_WString(90, 85, lowerThreshStr, Font12, 1, BLACK, WHITE);
    }

    // Pass fail mode, setting upper threshold
    if (formula.mode == 2 && lowerThreshSet == 2 && lowerHigherSet == 0) {
      Displ_WString(90, 85, higherThreshStr, Font12, 1, BLACK, WHITE);
    }

    // If values already set, go straight to pass fail screen
    if (lowThresh > 0 && highThresh > 0) {
      lowerHigherSet = 1;
    }

    // Pass fail mode, once both thresholds set
    if (formula.mode == 2 && lowerHigherSet == 1) {

      Displ_WString(14, 82, "Low Thresh:", Font12, 1, BLACK, WHITE);
      Displ_WString(90, 82, lowerThreshStr, Font12, 1, BLACK, WHITE);
      Displ_WString(7, 95, "High Thresh:", Font12, 1, BLACK, WHITE);
      Displ_WString(90, 95, higherThreshStr, Font12, 1, BLACK, WHITE);

      Displ_WString(5, 50, "RESET", Font12, 1, BLACK, RED);

      if (weightOverLim == 0) {

        // 2 digits e.g (8.4g)
        if (weight > 0 && weight < 10) {
          Displ_FillArea(55, 60, 22, 25, WHITE);
          Displ_WString(10, 65, weightStr, Font16, 1, BLACK, WHITE);
        }

        // 3 digits e.g (50.3g, -9.3g)
        if ((weight < 0 && weight > -10) || (weight > 9 && weight < 100)) {
          Displ_FillArea(65, 60, 22, 25, WHITE);
          Displ_WString(10, 65, weightStr, Font16, 1, BLACK, WHITE);
        }

        // 4 digits e.g (100.5g, -55.3g)
        if ((weight > 99 && weight < 1000) || (weight < -9 && weight > -100)) {
          Displ_FillArea(75, 60, 12, 25, WHITE);
          Displ_WString(10, 65, weightStr, Font16, 1, BLACK, WHITE);
        }

        // 5 digits e.g (1500.4g, -950.3g)
        if ((weight > 999 && weight < 2100) ||
            (weight < -99 && weight > -1000)) {
          Displ_WString(10, 65, weightStr, Font16, 1, BLACK, WHITE);
        }
      }

      if (weightOverLim == 1) {
        Displ_FillArea(33, 65, 55, 15, WHITE);
        Displ_WString(10, 65, "OL", Font16, 1, BLACK, WHITE);
      }

      // Checking if weight is passing or failing
      if (weight > formula.lowerThresh && weight < formula.upperThresh) {
        Displ_WString(100, 55, "+", Font24, 1, GREEN, WHITE);
      } else {
        Displ_WString(100, 55, "X", Font24, 1, RED, WHITE);
      }
    }

    // calibration mode
    if (formula.mode == 3 && (calibrationStep == 1 || calibrationStep == 2 ||
                              calibrationStep == 4)) {

      if (adcVal > 999) {
        Displ_FillArea(30, 55, 25, 12, WHITE);
        Displ_WString(10, 55, adcValStr, Font12, 1, RED, WHITE);
      }

      else {
        Displ_WString(10, 55, adcValStr, Font12, 1, RED, WHITE);
      }
    }

    if (formula.mode == 3 && calibrationStep == 3) {
      // Displ_WString(10, 55, adcValStr, Font12, 1, RED, WHITE);
      Displ_WString(65, 83, calibrateFirstStr, Font12, 1, BLACK, WHITE);
    }

    if (formula.mode == 3 && calibrationStep == 5) {

      Displ_WString(65, 83, calibrateSecondStr, Font12, 1, BLACK, WHITE);
    }

    // calibration set
    if (calibrationSet == 1) {
      // calibration logic
      formula.tareAdc = 0;
      formula.tareMass = 0;
      send_tare(&comms, &formula);
      calc_formula(&formula);
      calibrationSet = 0;
    }

    // counting mode
    if (formula.mode == 4 && countScreenSet == 1 && formula.countingSet != 2) {

      if (formula.countingSet == 0 && weightOverLim == 0) {

        // 2 digits e.g (8.4g)
        if (weight > 0 && weight < 10) {
          Displ_FillArea(55, 50, 22, 25, WHITE);
          Displ_WString(10, 55, weightStr, Font16, 1, BLACK, WHITE);
        }

        // 3 digits e.g (50.3g, -9.3g)
        if ((weight < 0 && weight > -10) || (weight > 9 && weight < 100)) {
          Displ_FillArea(65, 50, 22, 25, WHITE);
          Displ_WString(10, 55, weightStr, Font16, 1, BLACK, WHITE);
        }

        // 4 digits e.g (100.5g, -55.3g)
        if ((weight > 99 && weight < 1000) || (weight < -9 && weight > -100)) {
          Displ_FillArea(75, 50, 12, 25, WHITE);
          Displ_WString(10, 55, weightStr, Font16, 1, BLACK, WHITE);
        }

        // 5 digits e.g (1500.4g, -950.3g)
        if ((weight > 999 && weight < 2100) ||
            (weight < -99 && weight > -1000)) {
          Displ_WString(10, 55, weightStr, Font16, 1, BLACK, WHITE);
        }
      }

      if (formula.countingSet == 0 && weightOverLim == 1) {
        Displ_FillArea(33, 50, 55, 25, WHITE);
        Displ_WString(10, 55, "OL", Font16, 1, BLACK, WHITE);
      }

      if (formula.countingSet == 1 && reset == 0) {
        Displ_WString(110, 84, countingRefNumStr, Font12, 1, BLACK, WHITE);
      }

      // If a reset has been requested
      if (formula.countingSet == 1 && reset == 1) {
        Displ_WString(110, 84, countingRefNumStrTwo, Font12, 1, BLACK, WHITE);
      }
    }

    // Show counting screen when variables have been set
    if (formula.mode == 4 && formula.countingSet == 2) {

      countInt = weight / formula.unitMass;
      Displ_WString(14, 82, "Count:", Font12, 1, BLACK, WHITE);
      Displ_WString(60, 82, countStr, Font16, 1, BLACK, WHITE);

      if (weight < 100) {
        Displ_FillArea(65, 50, 22, 25, WHITE);
        Displ_WString(10, 55, weightStr, Font16, 1, BLACK, WHITE);
      }

      if (weight > 99 && weight < 1000) {
        Displ_FillArea(75, 50, 12, 25, WHITE);
        Displ_WString(10, 55, weightStr, Font16, 1, BLACK, WHITE);
      }

      else {
        Displ_WString(10, 55, weightStr, Font16, 1, BLACK, WHITE);
      }
    }

    // All touch logic below
    if (Touch_GetXYtouch().isTouch == 1) {

      // Check if a touch is held down to avoid random touches
      if (index % 2 == 1) {
        previousX = Touch_GetXYtouch().Xpos;
        previousY = Touch_GetXYtouch().Ypos;
      }

      if (index % 2 == 0) {
        previousXX = Touch_GetXYtouch().Xpos;
        previousYY = Touch_GetXYtouch().Ypos;
      }

      // Checking if the touch has been held down for a split second
      if (previousXX == previousX && previousYY == previousY) {
        touchIndex++;
      }

      // If touch is valid (held down) execute touch response
      if (touchIndex == 2) {
        x_coord = Touch_GetXYtouch().Xpos - 10;
        y_coord = Touch_GetXYtouch().Ypos - 5;

        // increase brightness
        if (x_coord > 130 && x_coord < 150 && y_coord > 40 && y_coord < 55) {
          Displ_BackLight('+');
        }

        // decrease brightness
        if (x_coord > 130 && x_coord < 150 && y_coord > 60 && y_coord < 80) {
          Displ_BackLight('-');
        }

        // change to serial communication
        if (x_coord > 130 && x_coord < 150 && y_coord > 20 && y_coord < 35) {
          Displ_WString(141, 108, "{", Font20, 1, RED,
                        YELLOW); // wifi symbol red
          Displ_WString(141, 88, "~", Font20, 1, D_GREEN,
                        YELLOW); // serial symbol green
          Displ_Border(133, 108, 30, 22, 1, BLACK);
          toggle_coms(&comms, "0");
          send_msg_serial(comms.serial, COMMS_MEDIUM, "0");
          send_msg_wifi(comms.wifi, COMMS_MEDIUM, "0");
        }

        // change to wifi communication
        if (x_coord > 130 && x_coord < 150 && y_coord > 0 && y_coord < 15) {
          Displ_WString(141, 108, "{", Font20, 1, D_GREEN,
                        YELLOW); // wifi symbol green
          Displ_WString(141, 88, "~", Font20, 1, RED,
                        YELLOW); // serial symbol red
          Displ_Border(133, 108, 30, 22, 1, BLACK);
          toggle_coms(&comms, "1");
          send_msg_serial(comms.serial, COMMS_MEDIUM, "1");
          send_msg_wifi(comms.wifi, COMMS_MEDIUM, "1");
        }

        // enter screen to tare mode
        if (formula.tareOn == 0 && x_coord > 0 && x_coord < 35 &&
            y_coord > 105 && y_coord < 125) {

          Displ_FillArea(0, 110, 130, 12, YELLOW);
          Displ_FillArea(5, 50, 128, 58, WHITE);
          Displ_WString(10, 85, "Click to set", Font12, 1, BLACK, WHITE);
          Displ_WString(10, 75, "tare weight:", Font12, 1, BLACK, WHITE);
          Displ_WString(10, 55, weightStr, Font16, 1, BLACK, WHITE);
          Displ_fillCircle(100, 65, 10, D_GREEN);
          formula.mode = 1;
        }

        // enter tare mode
        if (formula.mode == 1 && x_coord > 85 && x_coord < 115 &&
            y_coord > 45 && y_coord < 75) {

          Displ_WString(5, 5, "Tare", Font12, 1, BLUE,
                        D_GREEN); // turn button green
          formula.tareMass = weight;
          formula.tareAdc = adcVal;
          formula.tareOn = 1; // tare mode on
          // tareOnce = 1;
          formula.mode = 0; // tare set, go back to general weigh mode
          Displ_FillArea(5, 50, 128, 55, WHITE);

          send_tare(&comms, &formula);
        }

        // turn off tare mode
        if (formula.tareOn == 1 && x_coord > 0 && x_coord < 35 &&
            y_coord > 105 && y_coord < 125) {
          Displ_WString(5, 5, "Tare", Font12, 1, BLUE, RED);

          formula.tareMass = 0.0;
          formula.tareAdc = 0;
          formula.tareOn = 0; // tare mode off

          send_tare(&comms, &formula);

          if (formula.mode == 2) {
            Displ_FillArea(90, 82, 42, 20, WHITE);
          }
        }

        // enter weigh mode
        if (x_coord > 40 && x_coord < 80 && y_coord > 105 && y_coord < 125) {

          Displ_FillArea(0, 110, 130, 12, YELLOW);
          Displ_FillArea(5, 50, 128, 58, WHITE);
          formula.mode = 0;
          send_msg(&comms, MODE, "0");
        }

        // enter pass fail mode
        if (x_coord > 85 && x_coord < 160 && y_coord > 105 && y_coord < 125) {

          Displ_FillArea(0, 110, 130, 12, YELLOW);
          Displ_FillArea(5, 50, 128, 58, WHITE);
          formula.mode = 2;

          // If values are already set
          if (lowerHigherSet == 1) {
            send_msg(&comms, MODE, "2");
            send_pass_fail(&comms, &formula);
          }

          // Setting lower and higher thresholds
          if (lowerHigherSet != 1) {

            Displ_WString(12, 65, "1 2 3 4 5", Font16, 1, BLACK, WHITE);
            Displ_WString(12, 50, "6 7 8 9 0", Font16, 1, BLACK, WHITE);
            Displ_WString(10, 94, "Enter Lower", Font12, 1, BLACK, WHITE);
            Displ_WString(10, 84, "Threshold:", Font12, 1, BLACK, WHITE);
            Displ_FillArea(114, 52, 17, 15, RED);
            Displ_FillArea(114, 70, 17, 15, GREEN);
            lowerThreshSet = 1; // let lower threshold be set
          }
        }

        // pass fail mode active, selecting thresholds
        if (formula.mode == 2) {

          // KEYBOARD LOGIC
          // 1
          if (x_coord > 5 && x_coord < 18 && y_coord > 45 && y_coord < 60) {

            if (lowerThreshSet == 1) {

              if (lowerIndex == 0) {
                formula.lowerThresh = (formula.lowerThresh + 1000 / 1000);
              }
              if (lowerIndex == 1) {
                formula.lowerThresh =
                    ((formula.lowerThresh * 1000 + 100) / 100);
              }
              if (lowerIndex == 2) {
                formula.lowerThresh = ((formula.lowerThresh * 100 + 10) / 10);
              }
              if (lowerIndex == 3) {
                formula.lowerThresh = (formula.lowerThresh * 10 + 1);
              }
              lowerIndex++;
            }

            if (lowerThreshSet == 2) {

              if (higherIndex == 0) {
                formula.upperThresh = (formula.upperThresh + 1000 / 1000);
              }
              if (higherIndex == 1) {
                formula.upperThresh =
                    ((formula.upperThresh * 1000 + 100) / 100);
              }
              if (higherIndex == 2) {
                formula.upperThresh = ((formula.upperThresh * 100 + 10) / 10);
              }
              if (higherIndex == 3) {
                formula.upperThresh = (formula.upperThresh * 10 + 1);
              }
              higherIndex++;
            }
          }

          // 2
          if (x_coord > 20 && x_coord < 40 && y_coord > 45 && y_coord < 60) {

            if (lowerThreshSet == 1) {

              if (lowerIndex == 0) {
                formula.lowerThresh = (formula.lowerThresh + 2000 / 1000);
              }
              if (lowerIndex == 1) {
                formula.lowerThresh =
                    ((formula.lowerThresh * 1000 + 200) / 100);
              }
              if (lowerIndex == 2) {
                formula.lowerThresh = ((formula.lowerThresh * 100 + 20) / 10);
              }
              if (lowerIndex == 3) {
                formula.lowerThresh = (formula.lowerThresh * 10 + 2);
              }
              lowerIndex++;
            }

            if (lowerThreshSet == 2) {

              if (higherIndex == 0) {
                formula.upperThresh = (formula.upperThresh + 2000 / 1000);
              }
              if (higherIndex == 1) {
                formula.upperThresh =
                    ((formula.upperThresh * 1000 + 200) / 100);
              }
              if (higherIndex == 2) {
                formula.upperThresh = ((formula.upperThresh * 100 + 20) / 10);
              }
              if (higherIndex == 3) {
                formula.upperThresh = (formula.upperThresh * 10 + 2);
              }
              higherIndex++;
            }
          }

          // 3
          if (x_coord > 40 && x_coord < 70 && y_coord > 45 && y_coord < 60) {

            if (lowerThreshSet == 1) {

              if (lowerIndex == 0) {
                formula.lowerThresh = (formula.lowerThresh + 3000 / 1000);
              }
              if (lowerIndex == 1) {
                formula.lowerThresh =
                    ((formula.lowerThresh * 1000 + 300) / 100);
              }
              if (lowerIndex == 2) {
                formula.lowerThresh = ((formula.lowerThresh * 100 + 30) / 10);
              }
              if (lowerIndex == 3) {
                formula.lowerThresh = (formula.lowerThresh * 10 + 3);
              }
              lowerIndex++;
            }

            if (lowerThreshSet == 2) {

              if (higherIndex == 0) {
                formula.upperThresh = (formula.upperThresh + 3000 / 1000);
              }
              if (higherIndex == 1) {
                formula.upperThresh =
                    ((formula.upperThresh * 1000 + 300) / 100);
              }
              if (higherIndex == 2) {
                formula.upperThresh = ((formula.upperThresh * 100 + 30) / 10);
              }
              if (higherIndex == 3) {
                formula.upperThresh = (formula.upperThresh * 10 + 3);
              }
              higherIndex++;
            }
          }

          // 4
          if (x_coord > 70 && x_coord < 95 && y_coord > 45 && y_coord < 60) {

            if (lowerThreshSet == 1) {

              if (lowerIndex == 0) {
                formula.lowerThresh = (formula.lowerThresh + 4000 / 1000);
              }
              if (lowerIndex == 1) {
                formula.lowerThresh =
                    ((formula.lowerThresh * 1000 + 400) / 100);
              }
              if (lowerIndex == 2) {
                formula.lowerThresh = ((formula.lowerThresh * 100 + 40) / 10);
              }
              if (lowerIndex == 3) {
                formula.lowerThresh = (formula.lowerThresh * 10 + 4);
              }
              lowerIndex++;
            }

            if (lowerThreshSet == 2) {

              if (higherIndex == 0) {
                formula.upperThresh = (formula.upperThresh + 4000 / 1000);
              }
              if (higherIndex == 1) {
                formula.upperThresh =
                    ((formula.upperThresh * 1000 + 400) / 100);
              }
              if (higherIndex == 2) {
                formula.upperThresh = ((formula.upperThresh * 100 + 40) / 10);
              }
              if (higherIndex == 3) {
                formula.upperThresh = (formula.upperThresh * 10 + 4);
              }
              higherIndex++;
            }
          }

          // 5
          if (x_coord > 95 && x_coord < 110 && y_coord > 45 && y_coord < 60) {

            if (lowerThreshSet == 1) {

              if (lowerIndex == 0) {
                formula.lowerThresh = (formula.lowerThresh + 5000 / 1000);
              }
              if (lowerIndex == 1) {
                formula.lowerThresh =
                    ((formula.lowerThresh * 1000 + 500) / 100);
              }
              if (lowerIndex == 2) {
                formula.lowerThresh = ((formula.lowerThresh * 100 + 50) / 10);
              }
              if (lowerIndex == 3) {
                formula.lowerThresh = (formula.lowerThresh * 10 + 5);
              }
              lowerIndex++;
            }

            if (lowerThreshSet == 2) {

              if (higherIndex == 0) {
                formula.upperThresh = (formula.upperThresh + 5000 / 1000);
              }
              if (higherIndex == 1) {
                formula.upperThresh =
                    ((formula.upperThresh * 1000 + 500) / 100);
              }
              if (higherIndex == 2) {
                formula.upperThresh = ((formula.upperThresh * 100 + 50) / 10);
              }
              if (higherIndex == 3) {
                formula.upperThresh = (formula.upperThresh * 10 + 5);
              }
              higherIndex++;
            }
          }

          // 6
          if (x_coord > 5 && x_coord < 18 && y_coord > 60 && y_coord < 75) {

            if (lowerThreshSet == 1) {

              if (lowerIndex == 0) {
                formula.lowerThresh = (formula.lowerThresh + 6000 / 1000);
              }
              if (lowerIndex == 1) {
                formula.lowerThresh =
                    ((formula.lowerThresh * 1000 + 600) / 100);
              }
              if (lowerIndex == 2) {
                formula.lowerThresh = ((formula.lowerThresh * 100 + 60) / 10);
              }
              if (lowerIndex == 3) {
                formula.lowerThresh = (formula.lowerThresh * 10 + 6);
              }
              lowerIndex++;
            }

            if (lowerThreshSet == 2) {

              if (higherIndex == 0) {
                formula.upperThresh = (formula.upperThresh + 6000 / 1000);
              }
              if (higherIndex == 1) {
                formula.upperThresh =
                    ((formula.upperThresh * 1000 + 600) / 100);
              }
              if (higherIndex == 2) {
                formula.upperThresh = ((formula.upperThresh * 100 + 60) / 10);
              }
              if (higherIndex == 3) {
                formula.upperThresh = (formula.upperThresh * 10 + 6);
              }
              higherIndex++;
            }
          }

          // 7
          if (x_coord > 20 && x_coord < 40 && y_coord > 60 && y_coord < 75) {

            if (lowerThreshSet == 1) {

              if (lowerIndex == 0) {
                formula.lowerThresh = (formula.lowerThresh + 7000 / 1000);
              }
              if (lowerIndex == 1) {
                formula.lowerThresh =
                    ((formula.lowerThresh * 1000 + 700) / 100);
              }
              if (lowerIndex == 2) {
                formula.lowerThresh = ((formula.lowerThresh * 100 + 70) / 10);
              }
              if (lowerIndex == 3) {
                formula.lowerThresh = (formula.lowerThresh * 10 + 7);
              }
              lowerIndex++;
            }

            if (lowerThreshSet == 2) {

              if (higherIndex == 0) {
                formula.upperThresh = (formula.upperThresh + 7000 / 1000);
              }
              if (higherIndex == 1) {
                formula.upperThresh =
                    ((formula.upperThresh * 1000 + 700) / 100);
              }
              if (higherIndex == 2) {
                formula.upperThresh = ((formula.upperThresh * 100 + 70) / 10);
              }
              if (higherIndex == 3) {
                formula.upperThresh = (formula.upperThresh * 10 + 7);
              }
              higherIndex++;
            }
          }

          // 8
          if (x_coord > 40 && x_coord < 70 && y_coord > 60 && y_coord < 75) {

            if (lowerThreshSet == 1) {

              if (lowerIndex == 0) {
                formula.lowerThresh = (formula.lowerThresh + 8000 / 1000);
              }
              if (lowerIndex == 1) {
                formula.lowerThresh =
                    ((formula.lowerThresh * 1000 + 800) / 100);
              }
              if (lowerIndex == 2) {
                formula.lowerThresh = ((formula.lowerThresh * 100 + 80) / 10);
              }
              if (lowerIndex == 3) {
                formula.lowerThresh = (formula.lowerThresh * 10 + 8);
              }
              lowerIndex++;
            }

            if (lowerThreshSet == 2) {

              if (higherIndex == 0) {
                formula.upperThresh = (formula.upperThresh + 8000 / 1000);
              }
              if (higherIndex == 1) {
                formula.upperThresh =
                    ((formula.upperThresh * 1000 + 800) / 100);
              }
              if (higherIndex == 2) {
                formula.upperThresh = ((formula.upperThresh * 100 + 80) / 10);
              }
              if (higherIndex == 3) {
                formula.upperThresh = (formula.upperThresh * 10 + 8);
              }
              higherIndex++;
            }
          }

          // 9
          if (x_coord > 70 && x_coord < 95 && y_coord > 60 && y_coord < 75) {

            if (lowerThreshSet == 1) {

              if (lowerIndex == 0) {
                formula.lowerThresh = (formula.lowerThresh + 9000 / 1000);
              }
              if (lowerIndex == 1) {
                formula.lowerThresh =
                    ((formula.lowerThresh * 1000 + 900) / 100);
              }
              if (lowerIndex == 2) {
                formula.lowerThresh = ((formula.lowerThresh * 100 + 90) / 10);
              }
              if (lowerIndex == 3) {
                formula.lowerThresh = (formula.lowerThresh * 10 + 9);
              }
              lowerIndex++;
            }

            if (lowerThreshSet == 2) {

              if (higherIndex == 0) {
                formula.upperThresh = (formula.upperThresh + 9000 / 1000);
              }
              if (higherIndex == 1) {
                formula.upperThresh =
                    ((formula.upperThresh * 1000 + 900) / 100);
              }
              if (higherIndex == 2) {
                formula.upperThresh = ((formula.upperThresh * 100 + 90) / 10);
              }
              if (higherIndex == 3) {
                formula.upperThresh = (formula.upperThresh * 10 + 9);
              }
              higherIndex++;
            }
          }

          // 0
          if (x_coord > 95 && x_coord < 110 && y_coord > 60 && y_coord < 75) {

            if (lowerThreshSet == 1) {

              if (lowerIndex == 0) {
                formula.lowerThresh = (formula.lowerThresh * 10);
              }
              if (lowerIndex == 1) {
                formula.lowerThresh = (formula.lowerThresh * 10);
              }
              if (lowerIndex == 2) {
                formula.lowerThresh = (formula.lowerThresh * 10);
              }
              if (lowerIndex == 3) {
                formula.lowerThresh = (formula.lowerThresh * 10);
              }
              lowerIndex++;
            }

            if (lowerThreshSet == 2) {

              if (higherIndex == 0) {
                formula.upperThresh = (formula.upperThresh * 10);
              }
              if (higherIndex == 1) {
                formula.upperThresh = (formula.upperThresh * 10);
              }
              if (higherIndex == 2) {
                formula.upperThresh = (formula.upperThresh * 10);
              }
              if (higherIndex == 3) {
                formula.upperThresh = (formula.upperThresh * 10);
              }
              higherIndex++;
            }
          }

          // CLR
          if (x_coord > 110 && x_coord < 125 && y_coord > 60 && y_coord < 75) {
            Displ_FillArea(90, 85, 40, 15, WHITE);

            if (lowerThreshSet == 1) {
              formula.lowerThresh = 0;
              lowerIndex = 0;
            }

            if (lowerThreshSet == 2) {
              formula.upperThresh = 0;
              higherIndex = 0;
            }
          }

          // ENTER
          if (x_coord > 110 && x_coord < 125 && y_coord > 45 && y_coord < 60) {

            // If higher threshold was entered, goto pass fail screen
            if (lowerThreshSet == 2) {
              Displ_FillArea(5, 50, 128, 58, WHITE);
              lowerHigherSet = 1;
              highThresh = formula.upperThresh;
              save_pass_fail(&formula);
              send_pass_fail(&comms, &formula);
              send_msg(&comms, MODE, "2");
            }

            // If lower threshold entered, prompt user to enter higher threshold
            if (lowerThreshSet == 1) {
              lowerThreshSet = 2;
              Displ_WString(10, 94, "Enter Higher", Font12, 1, BLACK, WHITE);
              Displ_WString(10, 84, "Threshold:", Font12, 1, BLACK, WHITE);
              Displ_FillArea(90, 85, 40, 15, WHITE);
              lowThresh = formula.lowerThresh;
            }
          }
        }

        // counting mode
        if (x_coord > 90 && x_coord < 150 && y_coord > 85 && y_coord < 105) {
          Displ_FillArea(0, 110, 130, 12, YELLOW);
          Displ_FillArea(5, 50, 128, 58, WHITE);
          formula.mode = 4;
          countScreenSet = 1;

          // If counting mode variables already set
          if (formula.countingSet == 2) {
            send_msg(&comms, MODE, "4");
          }

          // Set counting variables through screen
          if (formula.countingSet != 2) {
            Displ_WString(10, 85, "Place reference", Font12, 1, BLACK, WHITE);
            Displ_WString(10, 75, "weight on scale:", Font12, 1, BLACK, WHITE);
            Displ_WString(10, 55, weightStr, Font16, 1, BLACK, WHITE);
            Displ_fillCircle(100, 65, 10, D_GREEN);
            formula.countingSet = 0;
          }
        }

        // User enters counting mode on screen to set variables
        if (formula.mode == 4 && x_coord > 85 && x_coord < 115 &&
            y_coord > 45 && y_coord < 75 && formula.countingSet == 0) {

          Displ_FillArea(5, 50, 128, 58, WHITE);
          Displ_WString(10, 94, "Select a number", Font12, 1, BLACK, WHITE);
          Displ_WString(10, 84, "between 1-100:", Font12, 1, BLACK, WHITE);
          Displ_WString(12, 65, "1 2 3 4 5", Font16, 1, BLACK, WHITE);
          Displ_WString(12, 50, "6 7 8 9 0", Font16, 1, BLACK, WHITE);
          Displ_FillArea(114, 52, 17, 15, RED);
          Displ_FillArea(114, 70, 17, 15, GREEN);
          refWeightInt = weight;
          formula.countingSet = 1;
        }

        // Entering reference count number
        if (formula.mode == 4 && formula.countingSet == 1) {

          // 1
          if (x_coord > 5 && x_coord < 18 && y_coord > 45 && y_coord < 60) {

            if (countingIndex == 0) {
              countingRefNum = countingRefNum + 1;
            }
            if (countingIndex == 1) {
              countingRefNum = countingRefNum * 10 + 1;
            }
            if (countingIndex == 2) {
              countingRefNum = countingRefNum * 10 + 1;
            }

            countingIndex++;
          }

          // 2
          if (x_coord > 20 && x_coord < 40 && y_coord > 45 && y_coord < 60) {

            if (countingIndex == 0) {
              countingRefNum = countingRefNum + 2;
            }
            if (countingIndex == 1) {
              countingRefNum = countingRefNum * 10 + 2;
            }
            if (countingIndex == 2) {
              countingRefNum = countingRefNum * 10 + 2;
            }

            countingIndex++;
          }

          // 3
          if (x_coord > 40 && x_coord < 70 && y_coord > 45 && y_coord < 60) {

            if (countingIndex == 0) {
              countingRefNum = countingRefNum + 3;
            }
            if (countingIndex == 1) {
              countingRefNum = countingRefNum * 10 + 3;
            }
            if (countingIndex == 2) {
              countingRefNum = countingRefNum * 10 + 3;
            }

            countingIndex++;
          }

          // 4
          if (x_coord > 70 && x_coord < 95 && y_coord > 45 && y_coord < 60) {

            if (countingIndex == 0) {
              countingRefNum = countingRefNum + 4;
            }
            if (countingIndex == 1) {
              countingRefNum = countingRefNum * 10 + 4;
            }
            if (countingIndex == 2) {
              countingRefNum = countingRefNum * 10 + 4;
            }

            countingIndex++;
          }

          // 5
          if (x_coord > 95 && x_coord < 110 && y_coord > 45 && y_coord < 60) {

            if (countingIndex == 0) {
              countingRefNum = countingRefNum + 5;
            }
            if (countingIndex == 1) {
              countingRefNum = countingRefNum * 10 + 5;
            }
            if (countingIndex == 2) {
              countingRefNum = countingRefNum * 10 + 5;
            }

            countingIndex++;
          }

          // 6
          if (x_coord > 5 && x_coord < 18 && y_coord > 60 && y_coord < 75) {

            if (countingIndex == 0) {
              countingRefNum = countingRefNum + 6;
            }
            if (countingIndex == 1) {
              countingRefNum = countingRefNum * 10 + 6;
            }
            if (countingIndex == 2) {
              countingRefNum = countingRefNum * 10 + 6;
            }

            countingIndex++;
          }

          // 7
          if (x_coord > 20 && x_coord < 40 && y_coord > 60 && y_coord < 75) {

            if (countingIndex == 0) {
              countingRefNum = countingRefNum + 7;
            }
            if (countingIndex == 1) {
              countingRefNum = countingRefNum * 10 + 7;
            }
            if (countingIndex == 2) {
              countingRefNum = countingRefNum * 10 + 7;
            }

            countingIndex++;
          }

          // 8
          if (x_coord > 40 && x_coord < 70 && y_coord > 60 && y_coord < 75) {

            if (countingIndex == 0) {
              countingRefNum = countingRefNum + 8;
            }
            if (countingIndex == 1) {
              countingRefNum = countingRefNum * 10 + 8;
            }
            if (countingIndex == 2) {
              countingRefNum = countingRefNum * 10 + 8;
            }

            countingIndex++;
          }

          // 9
          if (x_coord > 70 && x_coord < 95 && y_coord > 60 && y_coord < 75) {

            if (countingIndex == 0) {
              countingRefNum = countingRefNum + 9;
            }
            if (countingIndex == 1) {
              countingRefNum = countingRefNum * 10 + 9;
            }
            if (countingIndex == 2) {
              countingRefNum = countingRefNum * 10 + 9;
            }

            countingIndex++;
          }

          // 0
          if (x_coord > 95 && x_coord < 110 && y_coord > 60 && y_coord < 75) {

            if (countingIndex == 0) {
              // countingRefNum = countingRefNum * 10;
              countingIndex--;
            }
            if (countingIndex == 1) {
              countingRefNum = countingRefNum * 10;
              // countingIndex++;
            }
            if (countingIndex == 2) {
              countingRefNum = countingRefNum * 10;
              // countingIndex++;
            }

            countingIndex++;
          }

          // CLR
          if (x_coord > 110 && x_coord < 125 && y_coord > 60 && y_coord < 75) {

            Displ_FillArea(110, 84, 22, 12, WHITE);
            countingRefNum = 0;
            countingIndex = 0;
          }

          // ENTER
          if (x_coord > 110 && x_coord < 125 && y_coord > 45 && y_coord < 60) {

            // Counting mode variables entered
            Displ_FillArea(5, 50, 128, 58, WHITE);
            formula.countingSet = 2;
            set_count_vars(&comms, &formula, countingRefNum, refWeightInt);
            send_msg(&comms, MODE, "4");

            // If reference number was invalid, request another number
            if (countingRefNum < 1 || countingRefNum > 100) {
              Displ_WString(10, 90, "Number is outside", Font12, 1, BLACK,
                            WHITE);
              Displ_WString(6, 78, "the range of 1-100", Font12, 1, BLACK,
                            WHITE);
              Displ_WString(10, 66, "Try Again", Font12, 1, BLACK, RED);
              countingRefNum = 0;
              countingIndex = 0;
              formula.countingSet = 3;
            }
          }
        }

        // If a reset is requested after counting mode is set
        if (formula.mode == 4 && formula.countingSet == 2) {
          Displ_WString(98, 50, "RESET", Font12, 1, BLACK, RED);

          if (x_coord > 90 && x_coord < 120 && y_coord > 60 && y_coord < 80) {
            Displ_FillArea(5, 49, 128, 58, WHITE);
            Displ_WString(10, 85, "Place reference", Font12, 1, BLACK, WHITE);
            Displ_WString(10, 75, "weight on scale:", Font12, 1, BLACK, WHITE);
            Displ_WString(10, 55, weightStr, Font16, 1, BLACK, WHITE);
            Displ_fillCircle(100, 65, 10, D_GREEN);
            countingRefNum = 0;
            countingIndex = 0;
            formula.countingSet = 0;
            reset = 1;
          }
        }

        // invalid counting reference number entered, reset
        if (formula.mode == 4 && formula.countingSet == 3) {

          if (x_coord > 5 && x_coord < 60 && y_coord > 45 && y_coord < 60) {

            Displ_FillArea(5, 50, 128, 58, WHITE);
            Displ_WString(10, 94, "Select a number", Font12, 1, BLACK, WHITE);
            Displ_WString(10, 84, "between 1-100:", Font12, 1, BLACK, WHITE);
            Displ_WString(12, 65, "1 2 3 4 5", Font16, 1, BLACK, WHITE);
            Displ_WString(12, 50, "6 7 8 9 0", Font16, 1, BLACK, WHITE);
            Displ_FillArea(114, 52, 17, 15, RED);
            Displ_FillArea(114, 70, 17, 15, GREEN);
            formula.countingSet = 1;
          }
        }

        // calibration mode
        if (x_coord > 0 && x_coord < 70 && y_coord > 85 && y_coord < 105) {
          Displ_FillArea(0, 110, 130, 12, YELLOW);
          Displ_FillArea(5, 50, 128, 58, WHITE);
          formula.mode = 3;
          calibrateFirstIndex = 0;
          calibrateFirstNum = 0; // weight of first calibration object in grams
          calibrateSecondIndex = 0;
          calibrateSecondNum = 0; // weight of second calibration object in
                                  // grams
          calibrationSet = 0;

          Displ_WString(10, 95, "Take all weights", Font12, 1, BLACK, WHITE);
          Displ_WString(10, 82, "off scale and", Font12, 1, BLACK, WHITE);
          Displ_WString(10, 69, "hit Next.", Font12, 1, BLACK, WHITE);
          Displ_WString(85, 50, "NEXT", Font16, 1, BLACK, WHITE);

          calibrationStep = 1;
        }

        // calibration mode, setting 2nd weight
        if (formula.mode == 3 && calibrationStep == 4) {

          if (x_coord > 90 && x_coord < 120 && y_coord > 60 && y_coord < 80) {
            Displ_FillArea(5, 50, 128, 58, WHITE);
            Displ_WString(12, 65, "1 2 3 4 5", Font16, 1, BLACK, WHITE);
            Displ_WString(12, 50, "6 7 8 9 0", Font16, 1, BLACK, WHITE);
            Displ_WString(10, 94, "Enter 2nd Object", Font12, 1, BLACK, WHITE);
            Displ_WString(10, 84, "Weight:", Font12, 1, BLACK, WHITE);
            Displ_FillArea(114, 52, 17, 15, RED);
            Displ_FillArea(114, 70, 17, 15, GREEN);

            formula.adcVal2 = adcVal;

            calibrationStep = 5;
          }
        }

        // calibration mode, setting 1st weight
        if (formula.mode == 3 && calibrationStep == 2) {

          if (x_coord > 90 && x_coord < 120 && y_coord > 60 && y_coord < 80) {
            Displ_FillArea(5, 50, 128, 58, WHITE);
            Displ_WString(12, 65, "1 2 3 4 5", Font16, 1, BLACK, WHITE);
            Displ_WString(12, 50, "6 7 8 9 0", Font16, 1, BLACK, WHITE);
            Displ_WString(10, 94, "Enter 1st Object", Font12, 1, BLACK, WHITE);
            Displ_WString(10, 84, "Weight:", Font12, 1, BLACK, WHITE);
            Displ_FillArea(114, 52, 17, 15, RED);
            Displ_FillArea(114, 70, 17, 15, GREEN);

            formula.adcVal1 = adcVal;

            calibrationStep = 3;
          }
        }

        // calibration mode, setting 0 weight
        if (formula.mode == 3 && calibrationStep == 1) {

          if (x_coord > 90 && x_coord < 120 && y_coord > 60 && y_coord < 80) {
            Displ_FillArea(5, 49, 128, 58, WHITE);
            Displ_WString(10, 95, "Place single", Font12, 1, BLACK, WHITE);
            Displ_WString(10, 82, "weight on scale", Font12, 1, BLACK, WHITE);
            Displ_WString(10, 69, "and hit Next.", Font12, 1, BLACK, WHITE);
            Displ_WString(85, 50, "NEXT", Font16, 1, BLACK, WHITE);

            formula.adcVal0 = adcVal;

            calibrationStep = 2;
          }
        }

        // calibration mode, entering 1st object weight
        if (formula.mode == 3 && calibrationStep == 3) {

          // 1
          if (x_coord > 5 && x_coord < 18 && y_coord > 45 && y_coord < 60) {

            if (calibrateFirstIndex == 0) {
              calibrateFirstNum = calibrateFirstNum + 1;
            }
            if (calibrateFirstIndex == 1) {
              calibrateFirstNum = calibrateFirstNum * 10 + 1;
            }
            if (calibrateFirstIndex == 2) {
              calibrateFirstNum = calibrateFirstNum * 10 + 1;
            }
            if (calibrateFirstIndex == 3) {
              calibrateFirstNum = calibrateFirstNum * 10 + 1;
            }

            calibrateFirstIndex++;
          }

          // 2
          if (x_coord > 20 && x_coord < 40 && y_coord > 45 && y_coord < 60) {

            if (calibrateFirstIndex == 0) {
              calibrateFirstNum = calibrateFirstNum + 2;
            }
            if (calibrateFirstIndex == 1) {
              calibrateFirstNum = calibrateFirstNum * 10 + 2;
            }
            if (calibrateFirstIndex == 2) {
              calibrateFirstNum = calibrateFirstNum * 10 + 2;
            }
            if (calibrateFirstIndex == 3) {
              calibrateFirstNum = calibrateFirstNum * 10 + 2;
            }

            calibrateFirstIndex++;
          }

          // 3
          if (x_coord > 40 && x_coord < 70 && y_coord > 45 && y_coord < 60) {

            if (calibrateFirstIndex == 0) {
              calibrateFirstNum = calibrateFirstNum + 3;
            }
            if (calibrateFirstIndex == 1) {
              calibrateFirstNum = calibrateFirstNum * 10 + 3;
            }
            if (calibrateFirstIndex == 2) {
              calibrateFirstNum = calibrateFirstNum * 10 + 3;
            }
            if (calibrateFirstIndex == 3) {
              calibrateFirstNum = calibrateFirstNum * 10 + 3;
            }

            calibrateFirstIndex++;
          }

          // 4
          if (x_coord > 70 && x_coord < 95 && y_coord > 45 && y_coord < 60) {

            if (calibrateFirstIndex == 0) {
              calibrateFirstNum = calibrateFirstNum + 4;
            }
            if (calibrateFirstIndex == 1) {
              calibrateFirstNum = calibrateFirstNum * 10 + 4;
            }
            if (calibrateFirstIndex == 2) {
              calibrateFirstNum = calibrateFirstNum * 10 + 4;
            }
            if (calibrateFirstIndex == 3) {
              calibrateFirstNum = calibrateFirstNum * 10 + 4;
            }

            calibrateFirstIndex++;
          }

          // 5
          if (x_coord > 95 && x_coord < 110 && y_coord > 45 && y_coord < 60) {

            if (calibrateFirstIndex == 0) {
              calibrateFirstNum = calibrateFirstNum + 5;
            }
            if (calibrateFirstIndex == 1) {
              calibrateFirstNum = calibrateFirstNum * 10 + 5;
            }
            if (calibrateFirstIndex == 2) {
              calibrateFirstNum = calibrateFirstNum * 10 + 5;
            }
            if (calibrateFirstIndex == 3) {
              calibrateFirstNum = calibrateFirstNum * 10 + 5;
            }

            calibrateFirstIndex++;
          }

          // 6
          if (x_coord > 5 && x_coord < 18 && y_coord > 60 && y_coord < 75) {

            if (calibrateFirstIndex == 0) {
              calibrateFirstNum = calibrateFirstNum + 6;
            }
            if (calibrateFirstIndex == 1) {
              calibrateFirstNum = calibrateFirstNum * 10 + 6;
            }
            if (calibrateFirstIndex == 2) {
              calibrateFirstNum = calibrateFirstNum * 10 + 6;
            }
            if (calibrateFirstIndex == 3) {
              calibrateFirstNum = calibrateFirstNum * 10 + 6;
            }

            calibrateFirstIndex++;
          }

          // 7
          if (x_coord > 20 && x_coord < 40 && y_coord > 60 && y_coord < 75) {

            if (calibrateFirstIndex == 0) {
              calibrateFirstNum = calibrateFirstNum + 7;
            }
            if (calibrateFirstIndex == 1) {
              calibrateFirstNum = calibrateFirstNum * 10 + 7;
            }
            if (calibrateFirstIndex == 2) {
              calibrateFirstNum = calibrateFirstNum * 10 + 7;
            }
            if (calibrateFirstIndex == 3) {
              calibrateFirstNum = calibrateFirstNum * 10 + 7;
            }

            calibrateFirstIndex++;
          }

          // 8
          if (x_coord > 40 && x_coord < 70 && y_coord > 60 && y_coord < 75) {

            if (calibrateFirstIndex == 0) {
              calibrateFirstNum = calibrateFirstNum + 8;
            }
            if (calibrateFirstIndex == 1) {
              calibrateFirstNum = calibrateFirstNum * 10 + 8;
            }
            if (calibrateFirstIndex == 2) {
              calibrateFirstNum = calibrateFirstNum * 10 + 8;
            }
            if (calibrateFirstIndex == 3) {
              calibrateFirstNum = calibrateFirstNum * 10 + 8;
            }

            calibrateFirstIndex++;
          }

          // 9
          if (x_coord > 70 && x_coord < 95 && y_coord > 60 && y_coord < 75) {

            if (calibrateFirstIndex == 0) {
              calibrateFirstNum = calibrateFirstNum + 9;
            }
            if (calibrateFirstIndex == 1) {
              calibrateFirstNum = calibrateFirstNum * 10 + 9;
            }
            if (calibrateFirstIndex == 2) {
              calibrateFirstNum = calibrateFirstNum * 10 + 9;
            }
            if (calibrateFirstIndex == 3) {
              calibrateFirstNum = calibrateFirstNum * 10 + 9;
            }

            calibrateFirstIndex++;
          }

          // 0
          if (x_coord > 95 && x_coord < 110 && y_coord > 60 && y_coord < 75) {

            if (calibrateFirstIndex == 0) {
              // calibrateFirstNum = calibrateFirstNum * 10;
              calibrateFirstIndex--;
            }
            if (calibrateFirstIndex == 1) {
              calibrateFirstNum = calibrateFirstNum * 10;
              // calibrateFirstIndex++;
            }
            if (calibrateFirstIndex == 2) {
              calibrateFirstNum = calibrateFirstNum * 10;
              // calibrateFirstIndex++;
            }
            if (calibrateFirstIndex == 3) {
              calibrateFirstNum = calibrateFirstNum * 10;
              // calibrateFirstIndex++;
            }

            calibrateFirstIndex++;
          }

          // CLR
          if (x_coord > 110 && x_coord < 125 && y_coord > 60 && y_coord < 75) {

            Displ_FillArea(80, 83, 22, 12, WHITE);
            calibrateFirstNum = 0;
            calibrateFirstIndex = 0;
          }

          // ENTER
          if (x_coord > 110 && x_coord < 125 && y_coord > 45 && y_coord < 60) {

            Displ_FillArea(5, 50, 128, 58, WHITE);
            Displ_FillArea(5, 49, 128, 58, WHITE);
            Displ_WString(10, 95, "Place single", Font12, 1, BLACK, WHITE);
            Displ_WString(10, 82, "weight on scale", Font12, 1, BLACK, WHITE);
            Displ_WString(10, 69, "and hit Next.", Font12, 1, BLACK, WHITE);
            Displ_WString(85, 50, "NEXT", Font16, 1, BLACK, WHITE);

            formula.massVal1 = calibrateFirstNum;
            calibrationStep = 4;
          }
        }

        // calibration mode, entering 1st object weight
        if (formula.mode == 3 && calibrationStep == 5) {

          // 1
          if (x_coord > 5 && x_coord < 18 && y_coord > 45 && y_coord < 60) {

            if (calibrateSecondIndex == 0) {
              calibrateSecondNum = calibrateSecondNum + 1;
            }
            if (calibrateSecondIndex == 1) {
              calibrateSecondNum = calibrateSecondNum * 10 + 1;
            }
            if (calibrateSecondIndex == 2) {
              calibrateSecondNum = calibrateSecondNum * 10 + 1;
            }
            if (calibrateSecondIndex == 3) {
              calibrateSecondNum = calibrateSecondNum * 10 + 1;
            }

            calibrateSecondIndex++;
          }

          // 2
          if (x_coord > 20 && x_coord < 40 && y_coord > 45 && y_coord < 60) {

            if (calibrateSecondIndex == 0) {
              calibrateSecondNum = calibrateSecondNum + 2;
            }
            if (calibrateSecondIndex == 1) {
              calibrateSecondNum = calibrateSecondNum * 10 + 2;
            }
            if (calibrateSecondIndex == 2) {
              calibrateSecondNum = calibrateSecondNum * 10 + 2;
            }
            if (calibrateSecondIndex == 3) {
              calibrateSecondNum = calibrateSecondNum * 10 + 2;
            }

            calibrateSecondIndex++;
          }

          // 3
          if (x_coord > 40 && x_coord < 70 && y_coord > 45 && y_coord < 60) {

            if (calibrateSecondIndex == 0) {
              calibrateSecondNum = calibrateSecondNum + 3;
            }
            if (calibrateSecondIndex == 1) {
              calibrateSecondNum = calibrateSecondNum * 10 + 3;
            }
            if (calibrateSecondIndex == 2) {
              calibrateSecondNum = calibrateSecondNum * 10 + 3;
            }
            if (calibrateSecondIndex == 3) {
              calibrateSecondNum = calibrateSecondNum * 10 + 3;
            }

            calibrateSecondIndex++;
          }

          // 4
          if (x_coord > 70 && x_coord < 95 && y_coord > 45 && y_coord < 60) {

            if (calibrateSecondIndex == 0) {
              calibrateSecondNum = calibrateSecondNum + 4;
            }
            if (calibrateSecondIndex == 1) {
              calibrateSecondNum = calibrateSecondNum * 10 + 4;
            }
            if (calibrateSecondIndex == 2) {
              calibrateSecondNum = calibrateSecondNum * 10 + 4;
            }
            if (calibrateSecondIndex == 3) {
              calibrateSecondNum = calibrateSecondNum * 10 + 4;
            }

            calibrateSecondIndex++;
          }

          // 5
          if (x_coord > 95 && x_coord < 110 && y_coord > 45 && y_coord < 60) {

            if (calibrateSecondIndex == 0) {
              calibrateSecondNum = calibrateSecondNum + 5;
            }
            if (calibrateSecondIndex == 1) {
              calibrateSecondNum = calibrateSecondNum * 10 + 5;
            }
            if (calibrateSecondIndex == 2) {
              calibrateSecondNum = calibrateSecondNum * 10 + 5;
            }
            if (calibrateSecondIndex == 3) {
              calibrateSecondNum = calibrateSecondNum * 10 + 5;
            }

            calibrateSecondIndex++;
          }

          // 6
          if (x_coord > 5 && x_coord < 18 && y_coord > 60 && y_coord < 75) {

            if (calibrateSecondIndex == 0) {
              calibrateSecondNum = calibrateSecondNum + 6;
            }
            if (calibrateSecondIndex == 1) {
              calibrateSecondNum = calibrateSecondNum * 10 + 6;
            }
            if (calibrateSecondIndex == 2) {
              calibrateSecondNum = calibrateSecondNum * 10 + 6;
            }
            if (calibrateSecondIndex == 3) {
              calibrateSecondNum = calibrateSecondNum * 10 + 6;
            }

            calibrateSecondIndex++;
          }

          // 7
          if (x_coord > 20 && x_coord < 40 && y_coord > 60 && y_coord < 75) {

            if (calibrateSecondIndex == 0) {
              calibrateSecondNum = calibrateSecondNum + 7;
            }
            if (calibrateSecondIndex == 1) {
              calibrateSecondNum = calibrateSecondNum * 10 + 7;
            }
            if (calibrateSecondIndex == 2) {
              calibrateSecondNum = calibrateSecondNum * 10 + 7;
            }
            if (calibrateSecondIndex == 3) {
              calibrateSecondNum = calibrateSecondNum * 10 + 7;
            }

            calibrateSecondIndex++;
          }

          // 8
          if (x_coord > 40 && x_coord < 70 && y_coord > 60 && y_coord < 75) {

            if (calibrateSecondIndex == 0) {
              calibrateSecondNum = calibrateSecondNum + 8;
            }
            if (calibrateSecondIndex == 1) {
              calibrateSecondNum = calibrateSecondNum * 10 + 8;
            }
            if (calibrateSecondIndex == 2) {
              calibrateSecondNum = calibrateSecondNum * 10 + 8;
            }
            if (calibrateSecondIndex == 3) {
              calibrateSecondNum = calibrateSecondNum * 10 + 8;
            }

            calibrateSecondIndex++;
          }

          // 9
          if (x_coord > 70 && x_coord < 95 && y_coord > 60 && y_coord < 75) {

            if (calibrateSecondIndex == 0) {
              calibrateSecondNum = calibrateSecondNum + 9;
            }
            if (calibrateSecondIndex == 1) {
              calibrateSecondNum = calibrateSecondNum * 10 + 9;
            }
            if (calibrateSecondIndex == 2) {
              calibrateSecondNum = calibrateSecondNum * 10 + 9;
            }
            if (calibrateSecondIndex == 3) {
              calibrateSecondNum = calibrateSecondNum * 10 + 9;
            }

            calibrateSecondIndex++;
          }

          // 0
          if (x_coord > 95 && x_coord < 110 && y_coord > 60 && y_coord < 75) {

            if (calibrateSecondIndex == 0) {
              calibrateSecondIndex--;
            }
            if (calibrateSecondIndex == 1) {
              calibrateSecondNum = calibrateSecondNum * 10;
              // calibrateSecondIndex++;
            }
            if (calibrateSecondIndex == 2) {
              calibrateSecondNum = calibrateSecondNum * 10;
              // calibrateSecondIndex++;
            }
            if (calibrateSecondIndex == 3) {
              calibrateSecondNum = calibrateSecondNum * 10;
              // calibrateSecondIndex++;
            }

            calibrateSecondIndex++;
          }

          // CLR
          if (x_coord > 110 && x_coord < 125 && y_coord > 60 && y_coord < 75) {

            Displ_FillArea(80, 83, 22, 12, WHITE);
            calibrateSecondNum = 0;
            calibrateSecondIndex = 0;
          }

          // ENTER
          if (x_coord > 110 && x_coord < 125 && y_coord > 45 && y_coord < 60) {

            Displ_FillArea(5, 50, 128, 58, WHITE);
            Displ_FillArea(0, 110, 130, 12, YELLOW);
            formula.massVal2 = calibrateSecondNum;
            calibrationSet = 1;
            formula.mode = 0;
          }
        }

        // Requesting to reset pass fail mode thresholds
        if (formula.mode == 2 && lowerHigherSet == 1) {

          if (x_coord > 5 && x_coord < 45 && y_coord > 60 && y_coord < 80) {
            Displ_FillArea(5, 49, 128, 58, WHITE);
            Displ_WString(12, 65, "1 2 3 4 5", Font16, 1, BLACK, WHITE);
            Displ_WString(12, 50, "6 7 8 9 0", Font16, 1, BLACK, WHITE);
            Displ_WString(10, 94, "Enter Lower", Font12, 1, BLACK, WHITE);
            Displ_WString(10, 84, "Threshold:", Font12, 1, BLACK, WHITE);
            Displ_FillArea(114, 52, 17, 15, RED);
            Displ_FillArea(114, 70, 17, 15, GREEN);
            lowerThreshSet = 1; // let lower threshold be set
            formula.lowerThresh = 0;
            formula.upperThresh = 0;
            lowThresh = 0;
            highThresh = 0;
            lowerIndex = 0;
            higherIndex = 0;
            lowerHigherSet = 0;
          }
        }

        touchIndex = 0;
      }
      index++;
    }

    /* USER CODE END WHILE */
    iconReset++;

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void) {

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1100;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DISPL_CS_GPIO_Port, DISPL_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TOUCH_CS_GPIO_Port, TOUCH_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PWR_LED_GPIO_Port, PWR_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DISPL_DC_Pin | DISPL_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DISPL_CS_Pin DISPL_DC_Pin DISPL_RST_Pin */
  GPIO_InitStruct.Pin = DISPL_CS_Pin | DISPL_DC_Pin | DISPL_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : TOUCH_CS_Pin PWR_LED_Pin */
  GPIO_InitStruct.Pin = TOUCH_CS_Pin | PWR_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TOUCH_INT_Pin */
  GPIO_InitStruct.Pin = TOUCH_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TOUCH_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
