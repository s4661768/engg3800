/*
 * parsing.h
 *
 *  Created on: Sep 14, 2022
 *      Author: prism
 */

#ifndef __PARSING_H__
#define __PARSING_H__

#include "adc.h"

/* Message Identifiers */
#define MODE                                                                   \
  "MD" // Followed by '0', '2', '4' to representing desired mode of operation
#define TARE "TA"  // Followed by '1' or '0' to set or clear a tare
#define COUNT "CO" // Followed by count mode variables
#define COUNT_VALUE                                                            \
  "CV" // Followed by the number of 'units' measured on the scale
#define PASS_FAIL "PF" // Followed by pass fail mode lowerThresh and upperThresh
#define PASS_VALUE                                                             \
  "PV" // Followed by '1' or '0' depending on whether the current mass passes
#define CALIBRATE "CA"    // Followed by a zero to request calibration
#define COMMS_MEDIUM "CM" // Followed by a 1 for wifi or 0 for wired serial
#define POWER_ON                                                               \
  "PO" // Followed by a '1' to request startup information like pass fail
       // thresholds
#define ADC "ADC"       //  Followed by an ADC value
#define MASS "MASS"     // Followed by a mass value (MASS:1234.5678)
#define ZERO_VALUE "ZV" // Followed by the zeroValue of the scale
#define BACK_LIGHT                                                             \
  "BL" // Followed by '1' to increase or '0' to decrease the brightness of the
       // LCD

#define WIFI 1
#define WIRED_SERIAL 0

/**
 * Takes the ID part of a received GUI message and calls the appropriate
 * functions based on it. If the string contained in 'id' isn't a valid ID (as
 * described in the communication documentation) HAL_ERROR will be returned,
 * otherwise HAL_OK is returned.
 *
 * Parameters:
 * 	comms <Communication*>: a struct holding UART handles and various
 * communication flags used throughout the code. id <char*>: the ID from the GUI
 * message. data <char*>: the data from the GUI message. i2c
 * <I2C_HandleTypeDef*>: I2C handle.
 *
 * 	Returns:
 * 		err <HAL_StatusTypeDef>: HAL_OK if the id given is a valid ID,
 * HAL_ERROR otherwise.
 */
HAL_StatusTypeDef parse_msg(Communication *comms, char *id, char *data,
                            I2C_HandleTypeDef *i2c1, MassCalcVals *formula);

/**
 * This function splits the given string using the given delimiter to find two
 * elements. In the case of the pass_fail these elements are two mass values.
 *
 * Ensure the elements of the string must be less than 10 characters long.
 *
 * Parameters:
 * 	str <char*>: the string to be split.
 * 	arr <char**>: 2x10 (arr[2][10]) character array that will hold the two
 * elements found delimiter <char>: the character used to delimit the strings.
 */
void pf_split_string(char *str, char arr[2][10], char delimiter);

/**
 * This function splits the given 'str' using the given delimiter 'delimiter' to
 *find 5 elements. This function is called by calibrate_scale to split the
 *'data' the GUI sends into usable strings.
 *
 * Parameters:
 * 	str <char*> the string to be split.
 * 	arr <char**>: 5x15 (arr[5][15]) character array that will hold the 5
 *elements found. delimiter <char>: the character used to delimit the strings.
 */
void cal_split_string(char *str, char arr[5][15], char delimiter);

#endif
