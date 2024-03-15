/*
 * wifi.h
 *
 *  Created on: 9 Sep. 2022
 *      Author: prism
 */

#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__

#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"

#define WIFI 1
#define WIRED_SERIAL 0

/**
 * Holds the UART handles as well as several flags used throughout the program.
 */
typedef struct Communication {

  /* Flag showing which communication medium is being used. commsMedium == 0 is
   * for wired serial and commsMedium == 1 is for WiFi
   */
  uint8_t commsMedium;

  /* HUART and HI2C handles */
  UART_HandleTypeDef *wifi;
  UART_HandleTypeDef *serial;
  I2C_HandleTypeDef *i2c;

  /* Flag / counter of for new messages. When commsMedium == 0 receivedMsgs is
   * used as a flag. when commsMedium == 1 receivedMsgs is used as a counter for
   * the number of new message waiting to be processed. receivedMsgs is set or
   * incremented when the huart ISR receives a
   * '\n'.
   */
  uint8_t
      receivedMsgs; // Counter for the number of messages (newlines) received.
} Communication;

/**
 * Initialises the ESP32-C3 by resetting it, turning of echo, and then sending
 * team 26's network information. This function takes about 18 seconds to run as
 * each commands takes a variable amount of time to be processed by the
 * ESP32-C3.
 *
 * Paramters:
 *  wifi <UART_HandleTypeDef*>: a uart handle
 */
void init_wifi(UART_HandleTypeDef *wifi);

/**
 * Tells the ESP32-C3 to listen to the on 'team26/downlink'. This function has
 * a 3.5 second delay.
 *
 * Paramters:
 *  wifi <UART_HandleTypeDef*>: a uart handle
 */
void listen_wifi(UART_HandleTypeDef *wifi);

/**
 * Sends the UART message '<id>:<data>' to the ESP32-C3. A delay is used so that
 if this function
 * is called multiple times in a row, there will be no buffering issues with the
 ESP32-C3.
 *
 * The communication protocol specifies all the possible ID's and data formats
 that are permitted for
 * use.
 *
 * Paramters:
 *  wifi <UART_HandleTypeDef*>: a uart handle
 *  id <char*>: a string containing the identifier for the message (e.g. "ADC",
 "CA" etc.). (refer to parsing.h for full list of ID's)
 *  data <char*>: a string containing the data to be sent along with the
 identifier.
 */
void send_msg_wifi(UART_HandleTypeDef *wifi, char *id, char *data);

/**
 * Send the UART message '<id>:<data>' to the Seeeduino Turtle.
 *
 * The communication protocol specifies all the possible ID's and data formats
 that are permitted for
 * use.
 *
 * Paramters:
 *  serial <UART_HandleTypeDef*>: a uart handle
 *  id <char*>: a string containing the identifier for the message (e.g. "ADC",
 "CA" etc.).(refer to parsing.h for full list of ID's)
 *  data <char*>: a string containing the data to be sent along with the
 identifier.
 */
void send_msg_serial(UART_HandleTypeDef *serial, char *id, char *data);

/**
 * Sends the UART message '<id>:<data>' to the ESP32-C3 or the Seeeduino Turtle
 based on the
 * comms.commsMedium flag. If comms.commsMedium == 1 the message is sent to the
 ESP32-C3 using
 * send_msg_wifi. If comms.commsMedium == 0 the message is sent to the Seeeduino
 Turtle using
 * send_msg_serial
 *
 * Paramters:
 *  comms <Communication*>: a struct holding the UART handles as well as several
 flags such as the
 *  	comms.commsMedium flag.
 *  id <char*>: a string containing the identifier for the message (e.g. "ADC",
 "CA" etc.). (refer to parsing.h for full list of ID's)
 *  data <char*>: a string containing the data to be sent along with the
 identifier.
 */
void send_msg(Communication *comms, char *id, char *data);

/**
 * Toggles the communication medium between the Seeeduino Turtle (serial) and
 * the ESP32-C3 (WiFi). If data == "1" the controller will switch to
 * communicating using the ESP32-C3. If data == "0" the controller will will be
 * switched to communicate using the Seeeduino Turtle. The comms.commsMedium
 * flag is toggled along with these switches.
 *
 * Paramters:
 *  comms <Communication*>: a struct holding the UART handles as well as several
 * flags such as the comms.commsMedium flag. data <char*>: "1" if the user wants
 * to switch to WiFi, "0" if the user wants to switch to serial.
 */
void toggle_coms(Communication *comms, char *data);

/**
 * Takes the a message recieved from the ESP32-C3 and, if possible, extracts the
 * ID and data section from the GUI message. If the message the variable 'line'
 * holds is not of the correct structure the function may not properly end 'id'
 * and 'data'.
 *
 * If the 'line' passed to this function is has length less than 33 that means
 * it is not a message from the GUI. An example calibration request is shown
 * below: +MQTTSUBRECV:0,”team26/downlink”,30,CA:67840-70253-79652-101-475
 *
 * From this we can see a message fromt the GUI received over wifi will always
 * be of length > 33. If this function reveices a 'line' that is shorter than 33
 * characters long it will immeadiately return HAL_ERROR. If the 'line' passes
 * the length check the function uses the 3rd argument in the line, which is in
 * this case '30', to determine the length of the message
 * 'CA:67840-70253-79652-101-475'. Once the message is found the ID ('CA') and
 * data
 * ('67840-70253-79652-101-475') sections of the string are saved to the
 * appropriate variables.
 *
 * Parameters:
 *  line <char*>: the message received from the ESP32-C3.
 * 	msg <cahr*>: the message the GUI sent over WiFi
 *  id <char*>: the variable the ID part of the message will be saved to.
 *  data <char*>: the variable the data part of the message will be saved to.
 *
 * Returns:
 * 	err <HAL_StatusTypeDef>: HAL_OK if the 'line' was long enough to be a
 * received message from the GUI, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef get_msg(char *line, char *msg, char *id, char *data);

/**
 * Uses sprintf to convert the passed int to a string and then a '\0' is
 * appended to the new string.
 *
 * Parameters:
 *  str <char*>: the string the parsed int is going to the written to.
 *  num <int32_t>: the number to be parsed.
 */
void int_to_string(char *str, int32_t num);

/**
 * Uses sprintf to convert the passed float to a string and then a '\0' is
 * appended to the new string.
 *
 * Parameters:
 *  str <char*>: the string the parsed int is going to the written to.
 *  num <float>: the number to be parsed.
 */
void float_to_string(char *str, float num);

#endif
