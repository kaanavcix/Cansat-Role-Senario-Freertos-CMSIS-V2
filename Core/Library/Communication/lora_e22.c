/*
 * lora_e22.c
 *
 *  Created on: Jul 23, 2025
 *      Author: kaana
 */



#include "lora_e22.h"

extern UART_HandleTypeDef huart2;

Boolean Lora_Init(LoraAddressing_TypeDefInit *lora)
{
    uint8_t addr[6];
    uint8_t addrRec[6] = {0};

    if (lora->Channel > 81)
        lora->Channel = 0;

    LORA_CONFIGURATION();
    addr[0] = 0xC0;
    addr[1] = 0x00;
    addr[2] = 0x03;


    addr[3] = 0x12 ;//(lora->Channel >> 8) & 0xFF;
    addr[4] = 0x34;//(lora->Channel) & 0xFF;

    addr[5] = 0x62;// ((lora->AirData   & 0x07) << 5) |
              //((lora->ParityNo & 0x03) << 3) |
              //((lora->BaundRate & 0x07));

    //addr[6] = ((lora->TxPower & 0x03) << 6) |
      //        ((lora->SubPackageSize & 0x03) << 4); // Diğer bitler kullanılabilir.

    //addr[7] = (lora->EnableFixedTransmission ? 0x40 : 0x00); // 0x40 = 1 << 6


    HAL_Delay(100);
    HAL_UART_Transmit(lora->Huart, addr, sizeof(addr), 500);

    HAL_StatusTypeDef status = HAL_UART_Receive(lora->Huart, addrRec, sizeof(addrRec), 500);

    if (status == HAL_OK)
        return (addrRec[0] == 0xC1) ? True : False;

    return False;
}

Boolean Lora_Filling(LoraAddressing_TypeDefInit *lora)
{
	lora->Huart = &huart2;
	  lora->Address = 0x1234;
	  lora->Channel = 17;
	  lora->BaundRate = BAUD_9600;
	  lora->AirData = AIR_2K4;
	  lora->ParityNo = 	PARITY_8N1;
	  lora->SubPackageSize = SUBPACKET_200B;
	  lora->EnableFixedTransmission  = 0x00;
	  return True;
}

Boolean Lora_Transmiter(LoraAddressing_TypeDefInit *lora,uint8_t* message, uint8_t m_size)
{
	LORA_TX_RX_ACTIVE();

	HAL_StatusTypeDef status = HAL_UART_Transmit(lora->Huart, message, m_size, 100);

	return (status == HAL_OK) ? True : False;
}
Boolean Lora_Reciever(LoraAddressing_TypeDefInit *lora, uint8_t* message, uint8_t m_size)
{
	LORA_TX_RX_ACTIVE();

	HAL_StatusTypeDef status = HAL_UART_Receive(lora->Huart, message, m_size, 100);

	return (status == HAL_OK) ? True : False;

}

