/*
 * Modbus_slave.h
 *
 *  Created on: Mar 17, 2026
 *      Author: Admin
 */

#ifndef INC_MODBUS_SLAVE_H_
#define INC_MODBUS_SLAVE_H_

#include <stdint.h>
#include <crc.h>
#include "stm32f1xx_hal.h"
#define SLAVE_ID 5
#define ILLEGAL_FUNCTION 0x01 // dùng cho sai function code
#define ILLEGAL_DATA_ADDRESS 0x02 // dùng cho frame bị sai
#define ILLEAGAL_DATA_VALUE 0x03 // dùng cho master muốn đọc nhiều hơn số thanh ghi max


#define Max_holding_register 9
#define Max_input_register 9
#define Max_coil_register 63
#define Max_dis_input_register 63

void readHoldingregister(UART_HandleTypeDef* huart, uint8_t *RxData, uint8_t * TxData);
void readInputregister(UART_HandleTypeDef* huart, uint8_t *RxData, uint8_t * TxData);
void readCoilsregister(UART_HandleTypeDef* huart, uint8_t *RxData, uint8_t * TxData);
void read_dis_Inputregister(UART_HandleTypeDef* huart, uint8_t *RxData, uint8_t * TxData);
void writeSinglecoil(UART_HandleTypeDef* huart, uint8_t *RxData, uint8_t * TxData);
void writeSingle_holdingregister(UART_HandleTypeDef* huart, uint8_t *RxData, uint8_t * TxData);
void writeMultiple_Holdingregister(UART_HandleTypeDef* huart, uint8_t *RxData, uint8_t * TxData);
void writeMultiple_Coilsregister(UART_HandleTypeDef* huart, uint8_t *RxData, uint8_t * TxData);
void modbusException(uint8_t exception_code, UART_HandleTypeDef *huart,uint8_t * TxData, uint8_t *RxData);

#endif /* INC_MODBUS_SLAVE_H_ */
