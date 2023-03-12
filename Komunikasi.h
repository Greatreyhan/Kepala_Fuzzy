/*
 * Komunikasi.h
 *
 *  Created on: Mar 2, 2022
 *      Author: Maulana Reyhan Savero
 */
#ifndef KOMUNIKASI_H_
#define KOMUNIKASI_H_

#include "main.h"
#include <stdbool.h>

typedef struct{
	bool ping;
	bool standby;
	bool jalan;
	bool translasi;
	bool rotasi;
	bool req;
	bool statis;
}feedback_t;

typedef struct{
	int16_t pos_x;
	int16_t pos_y;
	int16_t pos_z;
	int16_t roll;
	int16_t pitch;
	int16_t yaw;
	int8_t time;
	int8_t walkpoint;
	int8_t mode;
	int8_t logic;
	int8_t speed;
}com_get_t;

void komunikasi_init(UART_HandleTypeDef* uart_handler);
bool tx_ping(void);
uint8_t checksum_generator(uint8_t* arr, uint8_t size);
bool tx_move_steady(void);
bool tx_move_jalan(int16_t pos_x, int16_t pos_y, int16_t pos_z, int8_t speed);
bool tx_move_translasi(int16_t pos_x, int16_t pos_y, int16_t pos_z, int8_t time, int8_t walkpoint);
bool tx_move_rotasi(int16_t roll, int16_t pitch, int16_t yaw, int16_t pos_z, int8_t time, int8_t walkpoint, int8_t mode);
void rx_start(void);
void rx_feedback(feedback_t* fed);
void rx_start_get(void);
void rx_get(com_get_t* get);
bool tx_statis(int16_t pos_x, int16_t pos_y, int16_t pos_z);
#endif