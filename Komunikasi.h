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
	uint16_t pos_x;
	uint16_t pos_y;
	uint16_t pos_z;
	uint16_t roll;
	uint16_t pitch;
	uint16_t yaw;
	uint8_t time;
	uint8_t walkpoint;
	uint8_t mode;
	uint8_t logic;
	uint8_t speed;
}com_get_t;

void komunikasi_init(UART_HandleTypeDef* uart_handler);
bool tx_ping(void);
uint8_t checksum_generator(uint8_t* arr, uint8_t size);
bool tx_move_steady(void);
bool tx_move_jalan(uint16_t pos_x, uint16_t pos_y, uint16_t pos_z, uint8_t speed);
bool tx_move_translasi(uint16_t pos_x, uint16_t pos_y, uint16_t pos_z, uint8_t time, uint8_t walkpoint);
bool tx_move_rotasi(uint16_t roll, uint16_t pitch, uint16_t yaw, uint16_t pos_z, uint8_t time, uint8_t walkpoint, uint8_t mode);
void rx_start(void);
void rx_feedback(feedback_t* fed);
void rx_start_get(void);
void rx_get(com_get_t* get);
bool tx_statis(uint16_t pos_x, uint16_t pos_y, uint16_t pos_z);
#endif