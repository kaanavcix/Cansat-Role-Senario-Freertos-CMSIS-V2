/*
* utiltity.h
*
* Created on: Jul 23, 2025
* Author: kaana
*/

#ifndef LIBRARY_UTILITY_UTILTITY_H_
#define LIBRARY_UTILITY_UTILTITY_H_


#include "stdbool.h"
#include "math.h"
#include "../Library/GPS/gps.h"
#include "main.h"
#include "../Library/Communication/lora_e22.h"
#include "stdlib.h"
#include "string.h"
#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "ctype.h"

typedef enum SYSTEM
{
    IDLE = 0U,
    WAITING = 1U,
    COMPELETED,
    CAUSE,
    TRANSMIT,
    RECIEVE
}SYSTEM;

/* Telemetri paketi veri yapısı - metin formatı */
typedef struct {
    uint32_t package_count;        // Paket sayacı
    uint8_t satallite_status;           // Uydu durumu (0-5)
    char error_code[7];              // Hata kodu (6 karakter + null)
    char sending_time[7];         // HHMMSS formatı (6 karakter + null)
    float pressure1;                  // Pascal
    float pressure2;                  // Pascal
    float altitude1;               // Metre
    float altitude2;               // Metre
    float diff_altitude;             // Metre
    float landing_speed;                // m/s
    float temperature;                 // Derece C
    float battery_voltage;             // Volt
    double gps1_latitude;           // Derece
    double gps1_longitude;          // Derece
    float gps1_altitude;            // Metre
    float pitch;                    // Derece
    float roll;                     // Derece
    float yaw;                      // Derece
    char rhrh[5];                   // RHRH komutu (4 karakter + null)
    float iot_s1_data;              // Sıcaklık veya diğer veri
    float iot_s2_data;              // Sıcaklık veya diğer veri
    uint32_t team_no;              // Takım numarası
} OptimizedTelemetriPaketi_TypeDef;

typedef struct BMP280_TypeDef
{
    float Altitude;
    float Pressure;
    float Temperature;
    float Humidity;
    float previousAltitude;
    float speed;

    uint8_t status;

}BMP280_TypeDefInit;

typedef struct MPU6050_TypeDef
{
    float AccX,AccY,AccZ;
    float GyroX,GyroY,GyroZ;
    float PitchAngle,RollAngle,YawAngle;
}MPU6050_TypeDefInit;

typedef struct GPS_HandleTypeDef
{
    float Altitude,Longitude,Latitude,Speed;

    lwgps_t gps;
    uint8_t rx_buffer[128];
    uint8_t rx_index;
    uint8_t rx_data;

}GPS_HandleTypeDefInit;

typedef struct Lora_HandleTypeDef
{
    char RecievePackage[100];
    char TransmitPackage[150];

} Lora_HandleTypeDEfInit;

typedef enum TELECOMMAND
{
	INITIAL = 0UL,
	START,
	CONTINUE,
	FINISH,
	STOP
} TELECOMMAND_ENUM;

typedef struct TelecommandHandle_TypeDef
{
	char command_temp[5];
	uint8_t command_received;
	TELECOMMAND_ENUM command_state;
} TelecommandHandle_TypeDefInit;


typedef struct Button_TypeDef
{
	uint8_t button_state;
	uint8_t separation_state;  // 0 = açık, 1 = kilitli
	uint8_t button_last_state;
} Button_TypeDefInit;

typedef struct UartData_TypeDef
{
	uint8_t rx_data;
	uint8_t rx_buffer[100];
	uint8_t rx_index;
} UartData_TypeDefInit;

typedef struct ContainerData_TypeDef
{
	float containerPressure;
	float containerAltitude;
} ContainerData_TypeDefInit;


/* Telemetri paket yapısı */
typedef struct {
    uint8_t start_marker;       // 'S' (0x53)
    uint32_t packet_number;     // 4 bytes
    uint8_t satellite_status;   // 1 byte
    uint8_t error_code[3];      // Compressed error code (3 bytes)
    uint32_t timestamp;         // 4 bytes (seconds since boot)
    uint16_t pressure1;         // 2 bytes (hPa, 0-1200 range, 0.1 hPa hassasiyet)
    uint16_t pressure2;         // 2 bytes (hPa, 0-1200 range, 0.1 hPa hassasiyet)
    int16_t altitude1;          // 2 bytes (meters, -1000 to +10000, 0.01m hassasiyet)
    int16_t altitude2;          // 2 bytes (meters, -1000 to +10000, 0.01m hassasiyet)
    uint16_t alt_difference;    // 2 bytes (0-255 meters, 0.01m hassasiyet)
    uint16_t descent_speed;     // 2 bytes (0-255 m/s, 0.01 m/s hassasiyet)
    int16_t temperature;        // 2 bytes (-128 to +127 °C, 0.1°C hassasiyet)
    uint16_t battery_voltage;   // 2 bytes (0-25.5V, 0.01V hassasiyet)
    int32_t latitude;           // 4 bytes (scaled integer)
    int32_t longitude;          // 4 bytes (scaled integer)
    int16_t gps_altitude;       // 2 bytes (meters, -1000 to +10000, 0.01m hassasiyet)
    int16_t pitch;              // 2 bytes (-180 to +180 degrees, 0.01 derece hassasiyet)
    int16_t roll;               // 2 bytes (-180 to +180 degrees, 0.01 derece hassasiyet)
    int16_t yaw;                // 2 bytes (-180 to +180 degrees, 0.01 derece hassasiyet)
    uint8_t rhrh[2];            // 2 bytes (compressed)
    int16_t iot_sensor1;        // 2 bytes (-128 to +127 °C, 0.1°C hassasiyet)
    int16_t iot_sensor2;        // 2 bytes (-128 to +127 °C, 0.1°C hassasiyet)
    uint32_t team_number;       // 4 bytes
    uint8_t end_marker;         // 'E' (0x45)
    uint16_t crc;               // 2 bytes CRC for error detection
} __attribute__((packed)) BinaryTelemetryPacket_t; // te



typedef struct GeneralControl_TypeDef
{
    BMP280_TypeDefInit bmp;
    MPU6050_TypeDefInit mpu;
    GPS_HandleTypeDefInit gps;
    LoraAddressing_TypeDefInit loraAdress;
    Lora_HandleTypeDEfInit lora;
    TelecommandHandle_TypeDefInit telecommand;
    Button_TypeDefInit button;
    OptimizedTelemetriPaketi_TypeDef telemetry_package;
    UartData_TypeDefInit uartData;
    ContainerData_TypeDefInit container;
    BinaryTelemetryPacket_t binary_telemetry;


} GeneralControl_TypeDefInit;


void TIM_Delay(uint32_t ms);

#endif /* LIBRARY_UTILITY_UTILTITY_H_ */
