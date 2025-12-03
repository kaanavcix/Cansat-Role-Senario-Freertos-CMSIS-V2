/**
  ******************************************************************************
  * @file           : telemetry_packet.h
  * @brief          : Telemetri paket yapısı ve fonksiyonları
  ******************************************************************************
  */

#ifndef LIBRARY_COMMUNICATION_TELEMETRY_PACKET_H_
#define LIBRARY_COMMUNICATION_TELEMETRY_PACKET_H_


#include "../Utility/utiltity.h"  // OptimizedTelemetriPaketi_TypeDef için



/* Fonksiyon prototipleri */
uint16_t calculate_crc16(uint8_t *data, uint16_t size);
float constrain(float value, float min, float max);
int32_t convert_gps_coordinate(double coord);
void pack_error_code(const char* error_str, uint8_t* packed);
void pack_rhrh(const char* rhrh_str, uint8_t* packed);
void CreateBinaryTelemetryPacket(BinaryTelemetryPacket_t* packet, OptimizedTelemetriPaketi_TypeDef* telemetry);
void CreateShortTimeString(char* time_str);
uint8_t UpdateOptimizedSatelliteStatus(OptimizedTelemetriPaketi_TypeDef *telemetri_paketi,float previous_altitude);
void UpdateOptimizedErrorCode(OptimizedTelemetriPaketi_TypeDef *telemetri_paketi, uint8_t sistem_statusu, char *error_str);
#endif /* LIBRARY_COMMUNICATION_TELEMETRY_PACKET_H_ */
