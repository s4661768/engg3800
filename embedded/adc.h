/*
 * adc_1.h
 *
 *  Created on: 2 Sep. 2022
 *      Author: prism
 */

#ifndef __ADC_1_H__
#define __ADC_1_H__

#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"

#include "communication.h"

/**
 * Holds several flags and variables used in multiple functions across the
 * firmware.
 */
typedef struct MassCalcVals {
  /* Zero value for ADC set on start up or calibration */
  int32_t zeroValue;

  /* The gradient and y-intercept used for the mass calculation formula */
  float grad, yInt;

  /* Flag showing what mode the scale is in */
  uint8_t mode;

  uint8_t countingSet;

  /* Flag, and lower and upper thresholds used for pass fail*/
  uint16_t upperThresh, lowerThresh;

  /* The flag, and ADC and mass values used when a tare is applied  */
  uint8_t tareOn;
  float tareMass; // The mass value the scale was tared at. (is to be subtracted
                  // from the averaged ADC value in mass computation
  uint32_t tareAdc;

  /* ADC and mass values used for calibration */
  float adcVal0, adcVal1, adcVal2;
  float massVal1, massVal2;

  /* Masses and counts used for counting mode */
  float unitMass;
  float refMassVal;
  float unitNumVal;

} MassCalcVals;

/**
 * Initialises the ADC by resetting all registers, turning on analogue and
 * digital power, and waiting for ADC to be 'ready' for further configuration.
 *
 * This and all functions that communication with the ADC use I2C to communicate
 * with it.
 *
 * Parameters:
 *  i2c <I2C_HandleTypeDef*>: the I2C handle.
 */
void init_adc(I2C_HandleTypeDef *i2c);

/**
 * Configures the ADC for default strain gauge usage as well as setting internal
 * LDO to 3.3V, gain to 8x, and enables the Cfilter.
 *
 * Parameters:
 *  i2c <I2C_HandleTypeDef*>: the I2C handle.
 */
void config_adc(I2C_HandleTypeDef *i2c);

/**
 * Tells the ADC to start making conversions by setting the cycle start bit and
 * then checking that it was set.
 *
 * Parameters:
 *  i2c <I2C_HandleTypeDef*>: the I2C handle.
 */
void start_adc(I2C_HandleTypeDef *i2c);

/**
 * Calibrates the offset of the ADC by using the PGA offset registers.
 *
 * The offset is used to shift the output range of the ADC to a more favourable
 * range.
 *
 * Parameters:
 *  i2c <I2C_HandleTypeDef*>: the I2C handle.
 */
void calibrate_adc(I2C_HandleTypeDef *i2c);

/**
 * Checks whether the ADC has a conversion that is ready to be read.
 *
 * Parameters:
 *  i2c <I2C_HandleTypeDef*>: the I2C handle.
 *
 * Returns:
 * 	err <HAL_StatusTypeDef>: HAL_OK if a conversion is ready, HAL_ERROR
 * otherwise
 */
HAL_StatusTypeDef conv_ready(I2C_HandleTypeDef *i2c);

/**
 * Checks if a conversion is ready using conv_ready() then reads the value from
 * the ADC conversion registers.
 *
 * Parameters:
 *  i2c <I2C_HandleTypeDef*>: the I2C handle.
 *
 * Returns:
 * 	result <int32_t>: -1 if no conversion was ready, the ADC value
 * otherwise.
 */
int32_t read_adc(I2C_HandleTypeDef *i2c);

/**
 * Reads and averages 'num' number of ADC values using read_adc(). If read_adc()
 * returns -1 this mean_adc() ignores it and attempts to read again not counting
 * the -1 towards the average.
 *
 * Parameters:
 *  i2c <I2C_HandleTypeDef*>: the I2C handle.
 *  num <uint8_t>: the number of values to be read for the average.
 *
 * Returns:
 * 	mean <int32_t>: the average of the number of ADC reads.
 */
int32_t mean_adc(I2C_HandleTypeDef *i2c, uint8_t num);

/**
 * Takes the given ADC value and converts it to its corresponding mass using the
 * gradient, y-intercept, and tare values found in 'formula'.
 *
 * Parameters:
 * 	formula <MassCalcVals*>: a struct holding the gradient, y-intercept, and
 * tare values. adcVal <int32_t>: the ADC value to be converted.
 *
 * Returns:
 * 	mass <float>: the mass corresponding to the ADC value according to the
 * variables in the formula struct
 */
float calc_mass(MassCalcVals *formula, int32_t adcVal);

/**
 * Recalibrates the scale by converting the mass and ADC values sent by the GUI
 * to floats and integers appropriately. The values are then saved to 'formula'
 * and calc_formula is called to recalculate the gradient and y-intercept used
 * for the ADC value to mass calculation.
 *
 * Parameters:
 * 	formula <MassCalcVals*>: a struct holding the gradient, y-intercept,
 * lower and upper thresholds , and tare values. comms <Communication*>: a
 * struct holding, uart handles and communication flags. data <char*>: a string
 * holding the 3 ADC and 2 mass values of the following format
 * 		<adc1>-<adc2>-<adc3>-<mass2>-<mass3>.
 */
void calibrate_scale(Communication *comms, char *data, MassCalcVals *formula);

/**
 * Once called by calibrate_scale it calculates the new gradient and saves it to
 * 'formula' and non-volatile memory as well as setting the y-intercept to the
 * new zero value.
 *
 * Parameters:
 * 	formula <MassCalcVals*>: a struct holding the gradient, y-intercept, and
 * tare values.
 */
void calc_formula(MassCalcVals *formula);

/**
 * Sets or clears the current tare based on the 'tare' variable passed in. If
 * 'tare' is 1, the current ADC value corresponding to the mass on the scale is
 * read in and converted to a mass value. Both the adc and mass value are saved
 * to the 'formula' struct so the tare can be accounted for in proceeding mass
 * calculations. The new tare value is sent back to the GUI and finally, the
 * 'formula.tareOn' flag is set. If 'tare' is 0 the tare mass and ADC variables
 * in 'formula' are cleared, the new tare of 0 is sent back to the GUI, and the
 * 'formula.tareOn' flag is set to 0.
 *
 * Parameters:
 *  comms <Communication*>: a struct holding, uart handles and communicvation
 * flags. i2c <I2C_HandleTypeDef*>: i2c handle formula <MassCalcVals*>: a struct
 * holding the gradient, y-intercept, and tare values. tare <uint8_t>: 1 if a
 * tare is supposed to be set, 0 if current tare is suppused to be cleared.
 */
void tare(Communication *comms, I2C_HandleTypeDef *i2c1, MassCalcVals *formula,
          uint8_t tare);

/**
 * Sends the current tare ADC value to the GUI. If the scale isn't in tare mode
 * (formula.tareOn == 0) the tare value sent will be 0.
 *
 * Parameters:
 * 	formula <MassCalcVals*>: a struct holding the gradient, y-intercept,
 * lower and upper thresholds , and tare values. comms <Communication*>: a
 * struct holding, uart handles and communication flags.
 */
void send_tare(Communication *comms, MassCalcVals *formula);

/**
 * Tells the device to work in pass fail mode as well as saving the lower and
 * upper bounds to the 'formula' struct and non-volatile memory. Once all of the
 * variables are saved and set the lower and upper bounds are sent back to the
 * GUI to be displayed.
 *
 * Parameters:
 * 	comms <Communication*>: a struct holding, uart handles and communication
 * flags. formula <MassCalcVals*>: a struct holding the gradient, y-intercept,
 * and tare values. data <char*>: the data part of the pass fail message sent
 * from the GUI containing the lower and upper bounds in the following format
 * "<lowerAdcVal>-<upperAdcVal>".
 */
void pass_fail(Communication *comms, MassCalcVals *formula, char *data);

/**
 * Sends the current lower and upper thresholds for pass fail mode to the GUI.
 *
 * Parameters:
 * 	formula <MassCalcVals*>: a struct holding the gradient, y-intercept,
 * lower and upper thresholds , and tare values. comms <Communication*>: a
 * struct holding, uart handles and communication flags.
 */
void send_pass_fail(Communication *comms, MassCalcVals *formula);

/**
 * When called by pass_fail it saves the new lower and upper thresholds to
 * non-volatile memory using STM's X-Cube EEPROM Emulation library.
 *
 * Parameters:
 * 	formula <MassCalcVals*>: a struct holding the gradient, y-intercept,
 * lower and upper thresholds, and tare values.
 */
void save_pass_fail(MassCalcVals *formula);

/**
 * Changes the scales mode to counting mode. This function splits 'data' int its
 * 2 elements and converts the strings to floats. From here set_count_vars is
 * used to finally set the appropriate variables in the 'formula' struct.
 *
 * Parameters:
 * 	formula <MassCalcVals*>: a struct holding the gradient, y-intercept,
 * lower and upper thresholds , and tare values. comms <Communication*>: a
 * struct holding, uart handles and communication flags. data <char*>: a string
 * holding the information the GUI sent for counting mode. The string is of the
 * form "<refCnt>-<refMass>".
 */
void counting(Communication *comms, MassCalcVals *formula, char *data);

/**
 * Sets the struct variables needed for counting mode and tells the GUI to
 * change mode to counting mode. This function uses 'unitNumVal' and
 * 'refMassVal' to calculate the mass of one unit called 'unitMass'. This float
 * is then saved to the 'formula' struct. After this the numbers the numbers the
 * GUI is meant to display are sent back in the format
 * “CO:<refCnt>-<refMass>-<unitMass>”.
 *
 * Parameters:
 * 	formula <MassCalcVals*>: a struct holding the gradient, y-intercept,
 * lower and upper thresholds , and tare values. comms <Communication*>: a
 * struct holding, uart handles and communication flags. unitNumVal <float>: the
 * number of units the user entered in the GUI. refMassVal <float>: the mass of
 * the number of units the user put on the scale.
 */
void set_count_vars(Communication *comms, MassCalcVals *formula, float unitNum,
                    float refMassVal);

/**
 * Sends the current count variables to the GUI.
 *
 * Parameters:
 * 	formula <MassCalcVals*>: a struct holding the gradient, y-intercept,
 * lower and upper thresholds , and tare values. comms <Communication*>: a
 * struct holding, uart handles and communication flags.
 */
void send_counting(Communication *comms, MassCalcVals *formula);

/**
 * Sends the ADC value corresponding to 0 grams to the GUI.
 *
 * When the controller receives the "PO:1" ("PO:1" means the GUI has just been
 * booted up and is requesting start up information) from the GUI this function
 * is called to send the value stored in 'formula.zeroValue'.
 *
 * Parameters:
 * 	formula <MassCalcVals*>: a struct holding the gradient, y-intercept,
 * lower and upper thresholds , and tare values. comms <Communication*>: a
 * struct holding, uart handles and communication flags.
 */
void send_zero_value(Communication *comms, MassCalcVals *formula);

/**
 * Sends the current mode to the GUI.
 *
 * Parameters:
 * 	formula <MassCalcVals*>: a struct holding the gradient, y-intercept,
 * lower and upper thresholds , and tare values. comms <Communication*>: a
 * struct holding, uart handles and communication flags.
 */
void send_mode(Communication *comms, MassCalcVals *formula);

#endif
