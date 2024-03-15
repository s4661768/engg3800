/*
 * adc_1.c
 *
 *  Created on: 2 Sep. 2022
 *      Author: prism
 */

#include "adc.h"
#include "communication.h"
#include "main.h"
#include "parsing.h"
#include <communication.h>
#include <stdio.h>
#include <stdlib.h>

/* Device and register addresses for I2C */
uint8_t ADDR = (0x2A << 1); // ADC Device address

uint8_t PU_CTRL = 0x00;
uint8_t CTRL_1 = 0x01;
uint8_t CTRL_2 = 0x02;

uint8_t HIGH_WORD = 0x12;
uint8_t MIDDLE_WORD = 0x13;
uint8_t LOW_WORD = 0x14;

void init_adc(I2C_HandleTypeDef *i2c) {
  uint8_t dataIn = 0x00;
  uint8_t dataOut = 0x00;

  dataOut = 0x01; // Reset all registers
  HAL_I2C_Mem_Write(i2c, ADDR, PU_CTRL, sizeof(PU_CTRL), &dataOut,
                    sizeof(dataOut), 10);

  dataOut = 0x02; // Set PUD -> Power on digital circuits
  HAL_I2C_Mem_Write(i2c, ADDR, PU_CTRL, sizeof(PU_CTRL), &dataOut,
                    sizeof(dataOut), 10);

  while (1) {
    HAL_I2C_Mem_Read(i2c, ADDR, PU_CTRL, sizeof(PU_CTRL), &dataIn,
                     sizeof(dataIn), 10);

    if (dataIn & (1 << 3)) { // Wait for PUR (Power Up Ready) to be set
      break;
    }
  }
}

void config_adc(I2C_HandleTypeDef *i2c) {
  uint8_t dataIn = 0x00;
  uint8_t dataOut = 0x00;
  uint8_t memAddr = 0x00;

  /* Default ADC set up */
  dataOut = 0xAE;
  HAL_I2C_Mem_Write(i2c, ADDR, PU_CTRL, sizeof(PU_CTRL), &dataOut,
                    sizeof(dataOut), 10);

  memAddr = 0x15;
  dataOut = 0x30;
  HAL_I2C_Mem_Write(i2c, ADDR, memAddr, sizeof(memAddr), &dataOut,
                    sizeof(dataOut), 10);

  /* Setting LDO voltage to 3.3v  and gain to 8*/
  dataOut = 0b00100011;
  HAL_I2C_Mem_Write(i2c, ADDR, CTRL_1, sizeof(CTRL_1), &dataOut,
                    sizeof(dataOut), 10);

  /* Increasing ADC sample rate */
  HAL_I2C_Mem_Read(i2c, ADDR, CTRL_2, sizeof(CTRL_2), &dataIn, sizeof(dataIn),
                   10);

  /* Setting Cfilter enable bit */
  memAddr = 0x1C;
  HAL_I2C_Mem_Read(i2c, ADDR, memAddr, sizeof(memAddr), &dataIn, sizeof(dataIn),
                   10);

  dataOut = dataIn | (1 << 7);
  HAL_I2C_Mem_Write(i2c, ADDR, memAddr, sizeof(memAddr), &dataOut,
                    sizeof(dataOut), 10);
}

void start_adc(I2C_HandleTypeDef *i2c) {
  uint8_t dataIn = 0x00;
  uint8_t dataOut = 0x00;

  HAL_I2C_Mem_Read(i2c, ADDR, PU_CTRL, sizeof(PU_CTRL), &dataIn, sizeof(dataIn),
                   10);

  dataOut = dataIn | (1 << 5); // Setting CS bit (cycle start bit) in control
                               // register 0 to start the ADC
  HAL_I2C_Mem_Write(i2c, ADDR, PU_CTRL, sizeof(PU_CTRL), &dataOut,
                    sizeof(dataOut), 10);

  /* Checking CS bit is actually set */
  while (1) {
    HAL_I2C_Mem_Read(i2c, ADDR, PU_CTRL, sizeof(PU_CTRL), &dataIn,
                     sizeof(dataIn), 10);

    if (dataIn & (0b00100000)) { // ADC has be started
      break;
    }
  }
}

void calibrate_adc(I2C_HandleTypeDef *i2c) {

  uint8_t dataOut = 0x00;
  uint8_t memAddr = 0x03;

  /* Setting the offset such that the data sits in a usable range */
  dataOut = 0b00000001;
  HAL_I2C_Mem_Write(i2c, ADDR, memAddr, sizeof(memAddr), &dataOut,
                    sizeof(dataOut), 10);

  memAddr = 0x04;
  dataOut = 0b00100111;
  HAL_I2C_Mem_Write(i2c, ADDR, memAddr, sizeof(memAddr), &dataOut,
                    sizeof(dataOut), 10);

  memAddr = 0x05;
  dataOut = 0xFF;
  HAL_I2C_Mem_Write(i2c, ADDR, memAddr, sizeof(memAddr), &dataOut,
                    sizeof(dataOut), 10);
}

HAL_StatusTypeDef conv_ready(I2C_HandleTypeDef *i2c) {
  uint8_t dataIn = 0x00;
  HAL_StatusTypeDef err = HAL_OK;

  HAL_I2C_Mem_Read(i2c, ADDR, PU_CTRL, sizeof(PU_CTRL), &dataIn, sizeof(dataIn),
                   10);

  if (!(dataIn & (1 << 5))) { // If there is no conversion ready return error
    err = HAL_ERROR;
  }

  return err;
}

int32_t read_adc(I2C_HandleTypeDef *i2c) {

  if (conv_ready(i2c) ==
      HAL_ERROR) { // If there is no conversion ready return error code
    return -1;
  }

  uint8_t high = 0x00;
  uint8_t middle = 0x00;
  uint8_t low = 0x00;

  int32_t result = 0;

  /* Read 3 8 bit words to make the 24 bit number */
  HAL_I2C_Mem_Read(i2c, ADDR, HIGH_WORD, sizeof(HIGH_WORD), &high, sizeof(high),
                   10);
  HAL_I2C_Mem_Read(i2c, ADDR, MIDDLE_WORD, sizeof(MIDDLE_WORD), &middle,
                   sizeof(middle), 10);
  HAL_I2C_Mem_Read(i2c, ADDR, LOW_WORD, sizeof(LOW_WORD), &low, sizeof(low),
                   10);

  result = ((high << 16) | (middle << 8) |
            (low)); // Bit shift the 3 words into an int32_t

  return result & 0xFFFFFFC0; // Mask of usable bits according to data sheet
}

int32_t mean_adc(I2C_HandleTypeDef *i2c, uint8_t num) {
  uint8_t reads = 0;
  int32_t adcVal = 0;
  int32_t sum = 0;
  int32_t mean = 0;

  while (reads < num) {
    adcVal = read_adc(i2c);

    if (adcVal < 0) {
      continue;
    } // Ensuring a conversion was ready (ignoring -1)

    sum += adcVal;
    reads++;
  }

  mean = sum / num;

  return mean;
}

float calc_mass(MassCalcVals *formula, int32_t adcVal) {

  /* y = mx + c */
  float mass = (adcVal * formula->grad) - formula->yInt - formula->tareMass;

  return mass;
}

void calibrate_scale(Communication *comms, char *data, MassCalcVals *formula) {
  char elements[5][15];
  for (uint8_t index = 0; index < 5; index++) {

    memset(elements[index], '\0', 15);
  }

  cal_split_string(data, elements, '-');

  /* ADC values are being cast to floats for float arithmetic happening later */
  formula->adcVal0 = (float)atoi(elements[0]);
  formula->adcVal1 = (float)atoi(elements[1]);
  formula->adcVal2 = (float)atoi(elements[2]);

  formula->massVal1 = atof(elements[3]);
  formula->massVal2 = atof(elements[4]);

  calc_formula(formula); // Calculate the gradient and y-intercept and save them
                         // to the struct.

  send_msg(comms, CALIBRATE, "1"); // Telling GUI calibration has finished
  tare(comms, comms->i2c, formula, 0);
}

void calc_formula(MassCalcVals *formula) {

  float m = (((formula->massVal2 - formula->massVal1) /
              (formula->adcVal2 - formula->adcVal1)));

  formula->zeroValue = formula->adcVal0;
  formula->grad = m;
  formula->yInt =
      formula->grad *
      formula->adcVal0; // Calculate the new zeroAdcVal with old gradient

  /* Can't store floats so the gradient is multiplied by 1 billion and stored as
   * an uint32_t */
  uint32_t x = (uint32_t)(m * 1000000000);

  HAL_FLASH_Unlock();
  EE_WriteVariable32bits(GRAD_UPPER, x);
  HAL_FLASH_Lock();
}

void tare(Communication *comms, I2C_HandleTypeDef *i2c, MassCalcVals *formula,
          uint8_t tare) {

  if (tare) { // User wants to set a tare

    /* Get tareADC and tareMass values */
    int32_t adcVal = mean_adc(i2c, 7);
    float massVal = calc_mass(formula, adcVal);

    formula->tareMass = massVal;
    formula->tareAdc = adcVal;

    /* Convert the ADC value to a string to that it can be sent back to the GUI
     */
    char adc[15] = {'\0'};
    char data[40] = {'\0'};
    int_to_string(adc, adcVal);
    sprintf(data, "1-%s", adc);
    send_msg(comms, TARE, data);

    formula->tareOn = 1; // Toggle on the tare flag

  } else if (!tare) { // User wants to clear the current tare

    /* Clear tare variables */
    formula->tareMass = 0.0;
    formula->tareAdc = 0;

    send_msg(comms, TARE, "0-0");

    formula->tareOn = 0; // Toggle off the tare flag
  }
}

void send_tare(Communication *comms, MassCalcVals *formula) {

  if (formula->tareOn == 1) { // If tare is on

    char adc[15] = {'\0'};
    char data[40] = {'\0'};

    /* Convert ADC value to string to send to GUI */
    int_to_string(adc, formula->tareAdc);
    sprintf(data, "1-%lu", formula->tareAdc);

    send_msg(comms, TARE, data);

  } else {

    send_msg(comms, TARE, "0-0");
  }
}

void pass_fail(Communication *comms, MassCalcVals *formula, char *data) {
  // data is of the format <123>-<456>
  char elements[2][10];
  memset(elements[0], '\0', 10);
  memset(elements[1], '\0', 10);

  pf_split_string(data, elements, '-');

  formula->lowerThresh = atof(elements[0]);
  formula->upperThresh = atof(elements[1]);
  formula->mode = 2;
  Displ_FillArea(5, 50, 128, 58, WHITE);

  save_pass_fail(formula);

  sprintf(data, "%s-%s", elements[0], elements[1]);
  send_msg(comms, PASS_FAIL, data);
}

void send_pass_fail(Communication *comms, MassCalcVals *formula) {
  char lower[15] = {'\0'};
  char upper[15] = {'\0'};
  char data[35] = {'\0'};

  float_to_string(lower, formula->lowerThresh);
  float_to_string(upper, formula->upperThresh);

  sprintf(data, "%s-%s", lower, upper);
  send_msg(comms, PASS_FAIL, data);
}

void save_pass_fail(MassCalcVals *formula) {

  HAL_FLASH_Unlock();
  EE_WriteVariable16bits(1, (formula->lowerThresh));
  EE_WriteVariable16bits(3, (formula->upperThresh));
  HAL_FLASH_Lock();
}

void counting(Communication *comms, MassCalcVals *formula, char *data) {

  float unitNum = 0.0;
  float refMassVal = 0.0;

  char elements[2][10];
  memset(elements[0], '\0', 10);
  memset(elements[1], '\0', 10);

  pf_split_string(data, elements, '-');

  unitNum = atof(elements[0]);
  refMassVal = atof(elements[1]);

  set_count_vars(comms, formula, unitNum, refMassVal);
  formula->countingSet = 2;
  Displ_FillArea(5, 50, 128, 58, WHITE);
  Displ_WString(98, 50, "RESET", Font12, 1, BLACK, RED);
}

void set_count_vars(Communication *comms, MassCalcVals *formula,
                    float unitNumVal, float refMassVal) {
  // “CO:<refCnt>-<refMass>-<unitMass>”
  formula->unitMass = (refMassVal / unitNumVal);
  formula->unitNumVal = unitNumVal;
  formula->refMassVal = refMassVal;

  char refMass[10] = {'\0'};
  char unitMass[10] = {'\0'};
  char unitNum[10] = {'\0'};

  float_to_string(refMass, refMassVal);
  float_to_string(unitMass, formula->unitMass);
  int_to_string(unitNum, unitNumVal);

  char msg[35] = {'\0'};
  sprintf(msg, "%s-%s-%s", unitNum, refMass, unitMass);

  send_msg(comms, COUNT, msg);
  send_msg(comms, MODE, "4");
}

void send_counting(Communication *comms, MassCalcVals *formula) {

  if (formula->mode == 4) {
    char refMass[20] = {'\0'};
    char unitMass[20] = {'\0'};
    char unitNumVal[20] = {'\0'};

    char data[70] = {'\0'};

    float_to_string(refMass, formula->refMassVal);
    float_to_string(unitMass, formula->unitMass);
    float_to_string(unitNumVal, formula->unitNumVal);

    // “CO:<refCnt>-<refMass>-<unitMass>”
    sprintf(data, "%s-%s-%s", unitNumVal, refMass, unitMass);

    send_msg(comms, COUNT, data);
  }
}

void send_zero_value(Communication *comms, MassCalcVals *formula) {

  char zeroValue[10] = {'\0'};

  int_to_string(zeroValue, formula->zeroValue);

  send_msg(comms, ZERO_VALUE, zeroValue);
}

void send_mode(Communication *comms, MassCalcVals *formula) {

  switch (formula->mode) {

  case 2: // Pass Fail mode

    send_msg(comms, MODE, "2");
    break;

  case 4: // Counting mode

    send_msg(comms, MODE, "4");
    break;

  default: // General weighing mode an intermediate state

    send_msg(comms, MODE, "0");
    break;
  }
}
