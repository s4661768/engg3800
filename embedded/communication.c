/*
 * wifi.c
 *
 *  Created on: 9 Sep. 2022
 *      Author: prism
 */

#include "communication.h"
#include "main.h"
#include "parsing.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void init_wifi(UART_HandleTypeDef *wifi) {

  /* Long delays are used here to give the ESP32-C3 enough time to process each
   * message */
  char *msg = "AT+RST\r\n"; // Resets the ESP32-C3
  HAL_UART_Transmit(wifi, (uint8_t *)msg, strlen(msg), 10);
  HAL_Delay(3500);

  msg = "ATE0\r\n"; // Disables the echo
  HAL_UART_Transmit(wifi, (uint8_t *)msg, strlen(msg), 10);
  HAL_Delay(3500);

  msg = "AT+MQTTUSERCFG=0,7,\"TEAM26\",\"team26\",\"e5aabb21296d9254786e9aa0\","
        "0,0,\"/ws\"\r\n";
  HAL_UART_Transmit(wifi, (uint8_t *)msg, strlen(msg), 10);
  HAL_Delay(7000);

  msg = "AT+MQTTCONN=0,\"tp-mqtt.uqcloud.net\",443,1\r\n";
  HAL_UART_Transmit(wifi, (uint8_t *)msg, strlen(msg), 100);
  HAL_Delay(3500);
}

void listen_wifi(UART_HandleTypeDef *wifi) {

  char *msg = "AT+MQTTSUB=0,\"team26/downlink\",0\r\n";
  HAL_UART_Transmit(wifi, (uint8_t *)msg, strlen(msg), 10);
  HAL_Delay(3500);
}

void send_msg_wifi(UART_HandleTypeDef *wifi, char *id, char *data) {

  char *start = "AT+MQTTPUB=0,\"team26/uplink\",\"";
  char *end = "\",0,0\r\n";
  char *colon = ":";

  HAL_UART_Transmit(wifi, (uint8_t *)start, strlen(start), 100);
  HAL_UART_Transmit(wifi, (uint8_t *)id, strlen(id), 100);
  HAL_UART_Transmit(wifi, (uint8_t *)colon, strlen(colon), 10);
  HAL_UART_Transmit(wifi, (uint8_t *)data, strlen(data), 100);
  HAL_UART_Transmit(wifi, (uint8_t *)end, strlen(end), 100);
  HAL_Delay(12);
}

void send_msg_serial(UART_HandleTypeDef *serial, char *id, char *data) {
  char *newLine = "\r\n";
  char *colon = ":";

  HAL_UART_Transmit(serial, (uint8_t *)id, strlen(id), 10);
  HAL_UART_Transmit(serial, (uint8_t *)colon, strlen(colon), 10);
  HAL_UART_Transmit(serial, (uint8_t *)data, strlen(data), 10);
  HAL_UART_Transmit(serial, (uint8_t *)newLine, strlen(newLine), 10);
}

void send_msg(Communication *comms, char *id, char *data) {

  if (comms->commsMedium == WIFI) {

    send_msg_wifi(comms->wifi, id, data);

  } else if (comms->commsMedium == WIRED_SERIAL) {

    send_msg_serial(comms->serial, id, data);
  }
}

void toggle_coms(Communication *comms, char *data) {

  if (!strcmp(data, "1")) { // User wants to switch to WiFi

    send_msg(comms, COMMS_MEDIUM, "1");
    comms->commsMedium = WIFI;

  } else if (!strcmp(data, "0")) { // User wants to switch to wired serial

    send_msg(comms, COMMS_MEDIUM, "0");
    comms->commsMedium = WIRED_SERIAL;
  }
}

HAL_StatusTypeDef get_msg(char *line, char *msg, char *id, char *data) {
  HAL_StatusTypeDef err = HAL_ERROR;
  int8_t msgLenVal = 0;
  uint8_t lineIndex = 34; // Start of GUI message
  uint8_t msgIndex = 0;
  char msgLen[3] = {'\0'};
  uint8_t msgLenIndex = 0;
  uint8_t idIndex = 0;
  uint8_t dataIndex = 0;
  uint8_t colon =
      0; // Flag that gets set when we have passed the colon in the GUI message
  char c = '\0';

  if (strlen(line) > 33) {

    err = HAL_OK;

    while (line[lineIndex] != ',') {

      msgLen[msgLenIndex] = line[lineIndex];
      msgLenIndex++;
      lineIndex++;
    }

    lineIndex++;

    msgLenVal = atoi(msgLen);

    while (lineIndex < strlen(line)) {
      c = line[lineIndex];

      if (c != '\0') {

        msg[msgIndex] = c;
        msgIndex++;
      }

      lineIndex++;
    }

    msgIndex = 0;
    c = '\0';
    while (msgIndex < strlen(msg)) {

      c = msg[msgIndex];

      if (c == ':') {

        msgIndex++;
        colon = 1;
        continue;
      }

      if (colon == 0 && c != '\0') {

        id[idIndex] = c;
        idIndex++;

      } else if (colon == 1 && c != '\0') {

        data[dataIndex] = c;
        dataIndex++;
      }

      msgIndex++;
    }
  }
  return err;
}

void int_to_string(char *str, int32_t num) {
  uint8_t index = sprintf(str, "%ld", num);

  str[index] = '\0';
}

void float_to_string(char *str, float num) {
  uint8_t index = sprintf(str, "%f", num);

  str[index] = '\0';
}
