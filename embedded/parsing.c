/*
 * parsing.c
 *
 *  Created on: Sep 14, 2022
 *      Author: prism
 */

#include "parsing.h"
#include "adc.h"
#include "main.h"
#include <stdlib.h>
#include <string.h>

HAL_StatusTypeDef parse_msg(Communication *comms, char *id, char *data,
                            I2C_HandleTypeDef *i2c1, MassCalcVals *formula) {
  HAL_StatusTypeDef err = HAL_OK;

  if (!strcmp(id,
              (char *)MODE)) { // Update the mode flag then echo the request.

    if (!strcmp(data, "0")) { // General Weighing Mode

      Displ_FillArea(0, 110, 130, 12, YELLOW);
      Displ_FillArea(5, 50, 128, 58,
                     WHITE); // Clearing main screen before mode change
      formula->mode = 0;
      send_msg(comms, MODE, "0");

    } else if (!strcmp(data, "2")) { // Pass Fail mode

      Displ_FillArea(5, 50, 128, 58, WHITE);
      formula->mode = 2;
      send_msg(comms, MODE, "2");

    } else if (!strcmp(data, "4")) { // Counting mode

      Displ_FillArea(5, 50, 128, 58, WHITE);
      formula->mode = 4;
      send_msg(comms, MODE, "4");
    }

  } else if (!strcmp(id, ((char *)TARE))) {

    if (!strcmp(data, "1")) { // Set a tare

      tare(comms, i2c1, formula, 1);
      err = HAL_OK;

    } else if (!strcmp(data, "0")) { // Clear current tare

      tare(comms, i2c1, formula, 0);
    }

  } else if (!strcmp(id, ((char *)COUNT))) { // Enter counting mode

    Displ_FillArea(5, 50, 128, 58, WHITE);

    counting(comms, formula, data);

    /* Change to counting mode and tell the GUI about change*/
    formula->mode = 4;
    send_msg(comms, MODE, "4");

  } else if (!strcmp(id, ((char *)PASS_FAIL))) { // Enter pass fail mode

    Displ_FillArea(5, 50, 128, 58, WHITE);
    pass_fail(comms, formula, data);

  } else if (!strcmp(id, ((char *)CALIBRATE))) { // Recalibrate the scale

    calibrate_scale(comms, data, formula);

  } else if (!strcmp(id,
                     ((char *)COMMS_MEDIUM))) { // Switch communication mediums

    toggle_coms(comms, data);

  } else if (!strcmp(id, ((char *)POWER_ON))) { // Send GUI startup information

    /* Tell the GUI the current communication medium */
    if (comms->commsMedium == 1) {

      send_msg_wifi(comms->wifi, COMMS_MEDIUM, "1");

    } else {

      send_msg_serial(comms->serial, COMMS_MEDIUM, "0");
    }

    send_msg(comms, POWER_ON, "1"); // Echo POWER_ON to show we are connected.
    send_mode(comms, formula); // Tell the GUI the current mode of the system.
    send_tare(comms,
              formula); // Tell the GUI if a tare is active and what it is.
    send_pass_fail(
        comms, formula); // Tell the GUI the current lower and upper thresholds
    send_zero_value(
        comms, formula); // Tell the GUI the ADC value corresponding to 0 grams.
    send_counting(comms, formula); // Send the counting variables to the GUI

  } else if (!strcmp(id, ((char *)BACK_LIGHT))) {

    if (!strcmp(data, "1")) { // Increase display back light

      Displ_BackLight('+');

    } else if (!strcmp(data, "0")) { // Decrease display back light

      Displ_BackLight('-');
    }

  } else { // Invalid message ID

    err = HAL_ERROR;
  }

  return err;
}

void pf_split_string(char *str, char arr[2][10], char delimiter) {

  uint8_t strIndex = 0;
  uint8_t elementIndex = 0;
  uint8_t element = 0;

  char c = '\0';

  while (strIndex < strlen(str)) {
    c = str[strIndex];

    if (c == delimiter) {

      // append '\0'
      arr[element][elementIndex] = '\0';
      elementIndex = 0;
      element++;

    } else if (c == '\r') {

      // append '\0'
      arr[element][elementIndex] = '\0';
      break;

    } else {
      arr[element][elementIndex] = c;
      elementIndex++;
    }
    strIndex++;
  }
}

void cal_split_string(char *str, char arr[5][15], char delimiter) {

  uint8_t strIndex = 0;
  uint8_t elementIndex = 0;
  uint8_t element = 0;

  char c = '\0';

  while (strIndex < strlen(str)) {
    c = str[strIndex];

    if (c == delimiter) {

      // append '\0'
      arr[element][elementIndex] = '\0';
      elementIndex = 0;
      element++;

    } else if (c == '\r') {

      // append '\0'
      arr[element][elementIndex] = '\0';
      break;

    } else {
      arr[element][elementIndex] = c;
      elementIndex++;
    }
    strIndex++;
  }
}
