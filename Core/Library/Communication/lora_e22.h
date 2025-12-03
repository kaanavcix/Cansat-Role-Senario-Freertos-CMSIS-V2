/*
 * lora_e22.h
 *
 *  Created on: Jul 23, 2025
 *      Author: kaana
 */

#ifndef LIBRARY_COMMUNICATION_LORA_E22_H_
#define LIBRARY_COMMUNICATION_LORA_E22_H_

#include "main.h"

typedef enum
{
False = 0U,
True = 1U
}Boolean;

#define LORA_M1(ACTIVE) HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, (ACTIVE) != 0x00 ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define LORA_M0(ACTIVE) HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin, (ACTIVE) != 0x00 ? GPIO_PIN_SET : GPIO_PIN_RESET)

#define LORA_CONFIGURATION() do{ \
								LORA_M0(0x01); \
								LORA_M1(0x01); \
								} while(0)

#define LORA_TX_RX_ACTIVE() do{ \
								LORA_M0(0x00); \
								LORA_M1(0x00); \
								} while(0)

typedef enum BaundRate_EnumTypeDef {
        BAUD_1200 = 0U,
        BAUD_2400,
        BAUD_4800,
        BAUD_9600, // Bunu kullancaz
        BAUD_19200,
        BAUD_38400,
        BAUD_57600,
        BAUD_115200
} BaundRate_EnumTypeDefInit;

typedef enum Parity_EnumTypeDef {
        PARITY_8N1 = 0U, //Bunu kullancaz
        PARITY_8O1 = 1U,
        PARITY_8E1 = 2U
} Parity_EnumTypeDefInit;

typedef enum AirDataRate_EnumTypeDef {
    	AIR_2K4 	= 	0U,
		AIR_2K4_1	=	1U,
		AIR_2K4_2	=	2U,
        AIR_4K8 	= 	3U,
        AIR_9K6 	= 	4U,
        AIR_19K2 	= 	5U,
        AIR_38K4 	= 	6U,
        AIR_62K5 	= 	7U
} AirDataRate_EnumTypeDefInit;

typedef enum TXPower_EnumTypeDef {
        POWER_22DBM = 0,
        POWER_17DBM = 1,
        POWER_13DBM = 2,
        POWER_10DBM = 3
} TXPower_EnumTypeDefInit;

typedef enum SubPackage_EnumTypeDef {
        SUBPACKET_200B = 0,
        SUBPACKET_128B = 1,
        SUBPACKET_64B  = 2,
        SUBPACKET_32B  = 3
} SubPackage_EnumTypeDefInit;




typedef struct LoraAddressing_TypeDef{
	UART_HandleTypeDef			*Huart;
    uint16_t 					Address;         			// 16-bit cihaz adresi (ADDH + ADDL) 2^16 = 65352 adressleme var ilk 2 high son iki low
    uint8_t 					Channel;          			// Kanal (0 - 80), frekans = 850.125 + ch (MHz)
    BaundRate_EnumTypeDefInit 	BaundRate;
    Parity_EnumTypeDefInit 		ParityNo; 					// 8N1 8 1 STOP BIT NO PARITY
    AirDataRate_EnumTypeDefInit 	AirData; 					// default 00 2.4k
    TXPower_EnumTypeDefInit 		TxPower; 					//Default 22dbm
    SubPackage_EnumTypeDefInit 	SubPackageSize; 			//buda 200ü geçmeyin

    uint8_t 					EnableFixedTransmission;  	// 1: Fixed mode aktif, 0: Transparent
} LoraAddressing_TypeDefInit;



 Boolean Lora_Init(LoraAddressing_TypeDefInit *lora);
 Boolean Lora_Filling(LoraAddressing_TypeDefInit *lora);
 Boolean Lora_Transmiter(LoraAddressing_TypeDefInit *lora,uint8_t* message, uint8_t m_size);
 Boolean Lora_Reciever(LoraAddressing_TypeDefInit *lora, uint8_t* message, uint8_t m_size);

#endif /* LIBRARY_COMMUNICATION_LORA_E22_H_ */
