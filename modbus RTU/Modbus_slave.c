/*
 * Modbus_slave.c
 *
 *  Created on: Mar 17, 2026
 *      Author: Admin
 */

#include <stm32f103xb.h>
#include "crc.h"
#include "Modbus_slave.h"
#include "stm32f1xx_hal.h"
#include <string.h>


static uint16_t Holding_register[10] = {
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9
};

static const uint16_t Input_Register[10] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };


static uint8_t Coil_Register[8] = {
		    0b10101100,  // Byte 0 → Coil 0 - 7
		    0b01101001,  // Byte 1 → Coil 8 - 15
		    0b11010010,  // Byte 2 → Coil 16 - 219
		    0b00011101,  // Byte 3 → Coil 24 - 31
		    0b11110000,  // Byte 4 → Coil 32 - 39
		    0b01010101,  // Byte 5 → Coil 40 - 47
		    0b00110011,  // Byte 6 → Coil 48 - 55
		    0b10000001   // Byte 7 → Coil 56 - 63
};


static const uint8_t dis_Input_Register[8] = {
		0b00110111,
		0b11110001,
		0b01001010,
		0b10011001,
		0b01101100,
		0b00011110,
		0b10100011,
		0b00011101,
};


void sendResponse(uint8_t *data, int size, UART_HandleTypeDef *huart){
	uint16_t crc = crc16(data, size);
	data[size] = crc & 0xFF;
	data[size+1] = (crc>>8) & 0xFF;


	HAL_UART_Transmit(huart, data, size+2, 1000);
}





void modbusException(uint8_t exception_code, UART_HandleTypeDef *huart,uint8_t * TxData, uint8_t *RxData){
	TxData[0] = SLAVE_ID;
	TxData[1] = RxData[1] | 0x80;
	TxData[2] = exception_code;
	sendResponse(TxData, 3, huart);
}





void readHoldingregister(UART_HandleTypeDef* huart, uint8_t *RxData, uint8_t * TxData){


	// check crc
	uint16_t crc = crc16(RxData, 6);
	if( (RxData[6] != (crc & 0xFF)) || (RxData[7] != ((crc>>8)& 0xFF)) ){
		modbusException(ILLEAGAL_DATA_VALUE, huart, TxData, RxData);
		return;
	}

	// check so thanh ghi muon doc
	uint16_t start_reg = (RxData[2]<<8) | RxData[3]; // save start register
	uint16_t num_reg = (RxData[4]<<8) | RxData[5];// save number of register want to read
	if( (num_reg < 1) || (num_reg > 125) )
	{
		modbusException(ILLEAGAL_DATA_VALUE, huart, TxData, RxData);
		return;
	}

	// check thanh ghi cuoi cung
	uint16_t end_reg = start_reg + num_reg -1 ; // save end register
	if(end_reg > Max_holding_register )
	{
		modbusException(ILLEGAL_DATA_ADDRESS, huart, TxData, RxData);
		return;
	}


	// Reset TxData
	memset(TxData, '\0', 5 + num_reg*2);

	// Ghi du lieu gui vao TxData
	TxData[0] = SLAVE_ID;
	TxData[1] = RxData[1];
	TxData[2] = num_reg*2;
	int index = 3;
	// ghi cac gia tri thanh ghi bat dau tu TxData[3]
	for (int i = 0; i< num_reg; i++){
		TxData[index] = (Holding_register[start_reg]>>8) & 0xFF;
		index++;
		TxData[index] = (Holding_register[start_reg]) & 0xFF;
		index++;
		start_reg++;
	}

	// gui TxData
	sendResponse(TxData, index, huart);
	return;
}
//
//
//
//
//
//

void readInputregister(UART_HandleTypeDef* huart, uint8_t *RxData, uint8_t * TxData){

	// check CRC
	uint16_t crc = crc16(RxData, 6);
	if( (RxData[6] != (crc & 0xFF)) || (RxData[7] != ((crc>>8)& 0xFF)) ){
		modbusException(ILLEAGAL_DATA_VALUE, huart, TxData, RxData);
		return;
	}

	uint16_t start_reg = (RxData[2]<<8) | RxData[3];

	// check so thanh ghi muon doc
	uint16_t num_reg = (RxData[4]<<8) | RxData[5];
	if ( ( num_reg < 1 ) || (num_reg > 125) )
	{
		modbusException(ILLEAGAL_DATA_VALUE, huart, TxData, RxData);
		return ;
	}

	// check thanh ghi cuoi cung
	uint16_t end_reg = start_reg + num_reg - 1; // first address = 0;
	if (end_reg > Max_input_register)
	{
		modbusException(ILLEGAL_DATA_ADDRESS, huart, TxData, RxData);
		return ;
	}

	// Reset du lieu trong TxData
	memset(TxData, '\0', 5 + num_reg*2);

	// Ghi du lieu muon gui vao TxData
	TxData[0] = SLAVE_ID;
	TxData[1] = RxData[1];
	TxData[2] = 2*num_reg;
	// ghi cac gia tri thanh ghi vao TxData bat dau tu TxData[3]
	int index = 3;
	for (int i = 0; i < num_reg; i++)
	{
		TxData[index] = (Input_Register[start_reg]>>8) & 0xFF;
		index++;
		TxData[index] = (Input_Register[start_reg]) & 0xFF;
		index++;
		start_reg++;
	}

	// Gui du lieu cho master
	sendResponse(TxData, index, huart);
	return;
}
//
//
//
//
//
//
void readCoilsregister(UART_HandleTypeDef* huart, uint8_t *RxData, uint8_t * TxData){

	// Check CRC
	uint16_t crc = crc16(RxData, 6);
	if( (RxData[6] != (crc & 0xFF)) || (RxData[7] != ((crc>>8)& 0xFF)) ){
		modbusException(ILLEAGAL_DATA_VALUE, huart, TxData, RxData);
		return;
	}
	uint16_t start_coils = (RxData[2]<<8) | RxData[3];

	// Check so thanh ghi muon doc
	uint16_t num_coils = (RxData[4]<<8) | RxData[5];
	if ( (num_coils < 1) || (num_coils > 2000) )
	{
		modbusException(ILLEAGAL_DATA_VALUE, huart, TxData, RxData);
		return;
	}

	// Check thanh ghi cuoi cung
	uint16_t end_coils = start_coils + num_coils - 1;
	if (end_coils > Max_coil_register)
	{
		modbusException(ILLEGAL_DATA_ADDRESS, huart, TxData, RxData);
		return;
	}

	// so byte = so coil/8 + phan du
	uint8_t num_byte = (num_coils / 8) + ((num_coils % 8) ? 1 : 0);

	// ResetTxData
	memset(TxData, '\0', 5 + num_byte);

	// Ghi du lieu vao TxData
	TxData[0] = SLAVE_ID;
	TxData[1] = RxData[1];
	TxData[2] = num_byte;

	// Ghi cac gia tri thanh ghi bat dau tu TxData[3]
	int index = 3; // chi so byte trong mang TxData
	int start_byte = start_coils / 8; // chi so byte trong Coil_register
	int bit_pos = start_coils % 8;  // chi so bit trong byte dang xet trong Coil_register
	int indx_pos = 0; // chi so bit trong byte dang xet trong TxData

	// Ghi cac gia tri trong coil_reg bat dau tu TxData[3]. Lưu 8 gia tri vao 1 byte
	for (int i = 0; i < num_coils; i++)
	{
		TxData[index] |= ((Coil_Register[start_byte] >> bit_pos) & 0x01)  << indx_pos;
		bit_pos++;  indx_pos++;
		if ( bit_pos > 7 )
		{
			bit_pos = 0;  start_byte++;
		}

		if ( indx_pos > 7)
		{
			indx_pos = 0;  index++;
		}
	}
	if ( num_coils % 8 !=0 ) index++;

	// Gui du lieu cho master
	sendResponse(TxData, index, huart);
	return;

}
//
//
//
//
//
//
void read_dis_Inputregister(UART_HandleTypeDef* huart, uint8_t *RxData, uint8_t * TxData)
{

	// Check CRC
	uint16_t crc = crc16(RxData, 6);
	if( (RxData[6] != (crc & 0xFF)) || (RxData[7] != ((crc>>8)& 0xFF)) ){
		modbusException(ILLEAGAL_DATA_VALUE, huart, TxData, RxData);
		return;
	}

	uint16_t start_reg = RxData[2]<<8 | RxData[3];

	// Check so thanh ghi muon doc
	uint16_t num_reg = RxData[4]<<8 | RxData[5];
	if( (num_reg < 1) || (num_reg > 2000) )
	{
		modbusException(ILLEAGAL_DATA_VALUE, huart, TxData, RxData);
		return;
	}

	// Check thanh ghi cuoi cung
	uint16_t end_reg = start_reg + num_reg - 1;
	if (end_reg > Max_dis_input_register)
	{
		modbusException(ILLEGAL_DATA_ADDRESS, huart, TxData, RxData);
		return;
	}

	uint8_t num_byte = (num_reg / 8) + ((num_reg % 8) ? 1 : 0);

	// Reset TxData
	memset(TxData, '\0', 5 + num_byte);

	// Ghi du lieu vao TxData
	TxData[0] = SLAVE_ID;
	TxData[1] = RxData[1];
	TxData[2] = num_byte;

	// Ghi du lieu cac thanh ghi bat dau tu TxData[3]
	int index = 3;
	int start_byte = start_reg / 8;
	int bit_pos = start_reg % 8;
	int indx_pos = 0;
	for (int i = 0; i < num_reg; i++)
	{
		TxData[index] |= ((dis_Input_Register[start_byte] >> bit_pos) & 0x01)  << indx_pos;
		bit_pos++;  indx_pos++;
		if ( bit_pos > 7 )
		{
			bit_pos = 0;  start_byte++;
		}

		if ( indx_pos > 7)
		{
			indx_pos = 0;  index++;
		}
	}
	if ( num_reg % 8 !=0 ) index++;

	// Gui du lieu cho master
	sendResponse(TxData, index, huart);
	return;

}
//
//
//
//
//
//
//
void writeSinglecoil(UART_HandleTypeDef* huart, uint8_t *RxData, uint8_t * TxData)
{

	// Check CRC
	uint16_t crc = crc16(RxData, 6);
	if( (RxData[6] != (crc & 0xFF)) || (RxData[7] != ((crc>>8)& 0xFF)) ){
		modbusException(ILLEAGAL_DATA_VALUE, huart, TxData, RxData);
		return;
	}

	// Chech thanh ghi muon doc
	uint16_t start_reg = RxData[2]<<8 | RxData[3];
	if(start_reg > Max_coil_register)
	{
		modbusException(ILLEGAL_DATA_ADDRESS, huart, TxData, RxData);
		return;
	}

	// Ghi du lieu vao thanh ghi mong muon
	uint16_t start_byte = start_reg / 8;
	uint16_t bit_shift = start_reg % 8;
	uint16_t set_value = RxData[4]<<8 | RxData[5];
	switch(set_value){
	case 0x0000:
		Coil_Register[start_byte] &= ~(1 << bit_shift); // set bit về 0
		break;

	case 0xFF00:
		Coil_Register[start_byte] |= (1 << bit_shift); // set bit lên 1
		break;

	default:
		modbusException(ILLEGAL_FUNCTION, huart, TxData, RxData);
		return;
		break;
	}

	// Gui du lieu cho master
	memset(TxData,'\0',8);
	TxData[0] = SLAVE_ID;
	TxData[1] = RxData[1];
	TxData[2] = RxData[2];
	TxData[3] = RxData[3];
	TxData[4] = RxData[4];
	TxData[5] = RxData[5];
	sendResponse(TxData, 6, huart);

}
//
//
//
//
//
//

void writeSingle_holdingregister(UART_HandleTypeDef* huart, uint8_t *RxData, uint8_t * TxData)
{

	// Check CRC
	uint16_t crc = crc16(RxData, 6);
	if( (RxData[6] != (crc & 0xFF)) || (RxData[7] != ((crc>>8)& 0xFF)) ){
		modbusException(ILLEAGAL_DATA_VALUE, huart, TxData, RxData);
		return;
	}

	// Check thanh ghi muon doc
	uint16_t start_reg = RxData[2]<<8 | RxData[3];
		if(start_reg > Max_holding_register)
		{
			modbusException(ILLEGAL_DATA_ADDRESS, huart, TxData, RxData);
			return;
		}

		// Ghi du lieu vao thanh ghi mong muon
		uint16_t set_value = RxData[4]<<8 | RxData[5];
		Holding_register[start_reg] = set_value;


		// Gui du lieu ve master
		memset(TxData,'\0',8);
		TxData[0] = SLAVE_ID;
		TxData[1] = RxData[1];
		TxData[2] = RxData[2];
		TxData[3] = RxData[3];
		TxData[4] = RxData[4];
		TxData[5] = RxData[5];
		sendResponse(TxData, 6, huart);
}
//
//
//
//
//
void writeMultiple_Holdingregister(UART_HandleTypeDef* huart, uint8_t *RxData, uint8_t * TxData)
{
	uint16_t start_reg = RxData[2]<<8 | RxData[3];
	uint16_t num_reg = RxData[4]<<8 | RxData[5];

	// check so thanh ghi muon ghi
	if( (num_reg < 1) | (num_reg > 123))
	{
		modbusException(ILLEAGAL_DATA_VALUE, huart, TxData, RxData);
		return;
	}

	// check so byte
	uint16_t num_byte = RxData[6];
	if (num_reg != num_byte/2) // moi thanh ghi can ghi 2 byte gia tri
	{
		modbusException(ILLEAGAL_DATA_VALUE, huart, TxData, RxData);
		return;
	}

	// check CRC
	uint16_t crc = crc16(RxData, (7 + num_byte));
	uint8_t crc_low  = crc & 0xFF;
	uint8_t crc_high = (crc >> 8) & 0xFF;

	if (RxData[9 + num_byte - 2] != crc_low ||
	    RxData[9 + num_byte - 1] != crc_high)
	{
	    modbusException(ILLEAGAL_DATA_VALUE, huart, TxData, RxData);
	    return;
	}

	//check thanh ghi cuoi cung
	uint16_t end_reg = start_reg + num_reg -1;
	if (end_reg > Max_holding_register){
		modbusException(ILLEGAL_DATA_ADDRESS, huart, TxData, RxData);
		return;
	}

	// Ghi du lieu vao cac thanh ghi
	int index = 7; // So chi byte trong RxData bat dau tu RxData[7]
	for (int i = 0; i < num_reg; i++)
	{
		Holding_register[start_reg] = RxData[index]<<8 | RxData[index+1];
		start_reg++;
		index = index + 2;
	}

	memset(TxData,'\0',8);
    TxData[0] = RxData[0];
    TxData[1] = RxData[1];
    TxData[2] = RxData[2];
    TxData[3] = RxData[3];
    TxData[4] = RxData[4];
    TxData[5] = RxData[5];

    // Gui du lieu cho master
	sendResponse(TxData, 6, huart);
	return;
}
//
//
//
//
//
void writeMultiple_Coilsregister(UART_HandleTypeDef* huart, uint8_t *RxData, uint8_t * TxData)
{
	uint16_t start_reg = RxData[2]<<8 | RxData[3];

	// Check so coil muon ghi
	uint16_t num_coil = RxData[4]<<8 | RxData[5];
	if ( (num_coil < 1) | (num_coil > 1968) )
	{
		modbusException(ILLEAGAL_DATA_VALUE, huart, TxData, RxData);
		return;
	}

	// Check thanh ghi cuoi cung
	uint16_t end_reg = start_reg + num_coil -1;
	if (end_reg > Max_coil_register)
	{
		modbusException(ILLEGAL_DATA_ADDRESS, huart, TxData, RxData);
	}

	// Check so byte
	uint16_t num_byte = RxData[6];
	if ( num_byte !=  num_coil/8 + ((num_coil % 8) ? 1 : 0) )
	{
		modbusException(ILLEAGAL_DATA_VALUE, huart, TxData, RxData);
		return;
	}

	// check CRC
	uint16_t crc = crc16(RxData, (7 + num_byte));
	uint8_t crc_low  = crc & 0xFF;
	uint8_t crc_high = (crc >> 8) & 0xFF;
	if (RxData[9 + num_byte - 2] != crc_low ||
	    RxData[9 + num_byte - 1] != crc_high)
	{
	    modbusException(ILLEAGAL_DATA_VALUE, huart, TxData, RxData);
	    return;
	}


	// Ghi du lieu vao Coil_register
	 int index = 7;
	 int bit_pos_idx = 0;
	 int start_byte = start_reg / 8;
	 int bit_pos_reg = start_reg % 8;

	 for (int i = 0; i < num_coil; i++)
	 {
		Coil_Register[start_byte] &= ~(1<<bit_pos_reg); // xoá bit tại vị trí bit_pos_reg
		Coil_Register[start_byte] |= ((RxData[index] >> bit_pos_idx) & 0x01) << bit_pos_reg; // set bit tại vị trí bit_pos_reg
		bit_pos_idx++;
		bit_pos_reg++;
		if (bit_pos_idx > 7)
		{
			bit_pos_idx = 0; index++;
		}

		if (bit_pos_reg > 7){
		bit_pos_reg = 0; start_byte++;
		}
	 }

	 // Gui du lieu cho master
	 memset(TxData,'\0',8);
	 TxData[0] = SLAVE_ID;
	 TxData[1] = RxData[1];
	 TxData[2] = RxData[2];
	 TxData[3] = RxData[3];
	 TxData[4] = RxData[4];
	 TxData[5] = RxData[5];
	 sendResponse(TxData, 6, huart);

}
