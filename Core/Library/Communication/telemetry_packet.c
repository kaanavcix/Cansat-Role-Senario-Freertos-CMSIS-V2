
#include "telemetry_packet.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* CRC-16 hesaplama fonksiyonu */
uint16_t calculate_crc16(uint8_t *data, uint16_t size) {
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < size; i++) {
        crc ^= (uint16_t)data[i];

        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;  // CRC-16-MODBUS polynomial
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

/* Değerleri sınırlamak için yardımcı fonksiyon */
float constrain(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/* GPS koordinatlarını ölçekli tam sayılara dönüştürme */
int32_t convert_gps_coordinate(double coord) {
    // 10^7 ile çarparak hassasiyeti koru (e.g. 40.12345678 → 401234567)
    return (int32_t)(coord * 10000000.0);
}

/* Hata kodunu binary gösterime dönüştürme */
void pack_error_code(const char* error_str, uint8_t* packed) {
    // Assuming error_str is 6 characters of '0' or '1'
    packed[0] = 0;
    packed[1] = 0;
    packed[2] = 0;

    // 6 biti ilk byte'a paketleme
    for (int i = 0; i < 6 && i < strlen(error_str); i++) {
        if (error_str[i] == '1') {
            packed[i/8] |= (1 << (i % 8));
        }
    }
}

/* RHRH kodunu 2 byte'a paketleme */
void pack_rhrh(const char* rhrh_str, uint8_t* packed) {
    // 4 karakteri 2 byte'a dönüştür
    packed[0] = 0;
    packed[1] = 0;

    for (int i = 0; i < 4 && i < strlen(rhrh_str); i++) {
        if (i < 2) {
            packed[0] |= ((rhrh_str[i] - '0') << (i * 4));
        } else {
            packed[1] |= ((rhrh_str[i] - '0') << ((i-2) * 4));
        }
    }
}

/* CreateBinaryTelemetryPacket fonksiyonunda yönelim verilerinin hassasiyetini arttırma */
void CreateBinaryTelemetryPacket(BinaryTelemetryPacket_t* packet, OptimizedTelemetriPaketi_TypeDef* telemetry) {
    // Yapıyı önce temizle
    memset(packet, 0, sizeof(BinaryTelemetryPacket_t));

    // İşaretçileri ayarla
    packet->start_marker = 'S';
    packet->end_marker = 'E';

    // Verileri uygun ölçekleme ile doldur
    packet->packet_number = telemetry->package_count;
    packet->satellite_status = telemetry->satallite_status;

    // Hata kodunu paketle
    pack_error_code(telemetry->error_code, packet->error_code);

    // Zaman damgasını boot'tan bu yana geçen saniyeye dönüştür
    uint32_t current_tick = HAL_GetTick();
    packet->timestamp = current_tick / 1000;

    // Basınç değerlerini ölçekle (Pa -> hPa, 0-1200 hPa aralığı ile sınırla)
    // 1 ondalık basamak hassasiyet (örn: 101.3 hPa)
    packet->pressure1 = (uint16_t)(constrain(telemetry->pressure1 / 100.0f, 0, 1200) * 10.0f);
    packet->pressure2 = (uint16_t)(constrain(telemetry->pressure2 / 100.0f, 0, 1200) * 10.0f);

    // Yükseklik değerlerini ölçekle (2 ondalık basamak hassasiyet)
    // -1000 ila +10000 metre ile sınırla, 0.01m çözünürlük
    packet->altitude1 = (int16_t)(constrain(telemetry->altitude1, -1000, 10000) * 100.0f);
    packet->altitude2 = (int16_t)(constrain(telemetry->altitude2, -1000, 10000) * 100.0f);

    // İrtifa farkı (0-255 metre ile sınırla, 0.01m çözünürlük)
    packet->alt_difference = (uint16_t)(constrain(telemetry->diff_altitude, 0, 255) * 100.0f);

    // İniş hızı (0-255 m/s ile sınırla, 0.01 m/s çözünürlük)
    packet->descent_speed = (uint16_t)(constrain(telemetry->landing_speed, 0, 255) * 100.0f);

    // Sıcaklık (-128 ila +127 °C ile sınırla, 0.1°C çözünürlük)
    packet->temperature = (int16_t)(constrain(telemetry->temperature, -128, 127) * 10.0f);

    // Pil gerilimi (0-25.5V, 0.01V çözünürlük)
    packet->battery_voltage = (uint16_t)(constrain(telemetry->battery_voltage, 0, 25.5) * 100.0f);

    // GPS koordinatları (ölçekli tam sayılar)
    packet->latitude = convert_gps_coordinate(telemetry->gps1_latitude);
    packet->longitude = convert_gps_coordinate(telemetry->gps1_longitude);

    // GPS yüksekliği (-1000 ila +10000 metre ile sınırla, 0.01m çözünürlük)
    packet->gps_altitude = (int16_t)(constrain(telemetry->gps1_altitude, -1000, 10000) * 100.0f);

    // Yönelim (-180 ila +180 derece ile sınırla, 0.01 derece çözünürlük)
    // Değerler 0.00 olsa bile yine de ölçeklendirilip gönderiliyor
    packet->pitch = (int16_t)(telemetry->pitch * 100.0f);  // 0.01 derece hassasiyet
    packet->roll = (int16_t)(telemetry->roll * 100.0f);    // 0.01 derece hassasiyet
    packet->yaw = (int16_t)(telemetry->yaw * 100.0f);      // 0.01 derece hassasiyet

    // RHRH paketleme
    pack_rhrh(telemetry->rhrh, packet->rhrh);

    // IoT sensörleri (-128 ila +127 °C ile sınırla, 0.1°C çözünürlük)
    packet->iot_sensor1 = (int16_t)(constrain(telemetry->iot_s1_data, -128, 127) * 10.0f);
    packet->iot_sensor2 = (int16_t)(constrain(telemetry->iot_s2_data, -128, 127) * 10.0f);

    // Takım numarası
    packet->team_number = telemetry->team_no;

    // CRC hesapla (CRC alanının kendisi hariç)
    packet->crc = calculate_crc16((uint8_t*)packet, sizeof(BinaryTelemetryPacket_t) - 2);
}

/* Kısa zaman formatında string oluştur */
void CreateShortTimeString(char* time_str) {
    uint32_t current_tick = HAL_GetTick();
    uint32_t seconds_total = current_tick / 1000;

    uint8_t hours = (seconds_total / 3600) % 24;
    uint8_t minutes = (seconds_total / 60) % 60;
    uint8_t seconds = seconds_total % 60;

    sprintf(time_str, "%02d%02d%02d", hours, minutes, seconds);
}

/* Optimize edilmiş uydu durumu güncelleme */
uint8_t UpdateOptimizedSatelliteStatus(OptimizedTelemetriPaketi_TypeDef *telemetri_paketi,float previous_altitude) {
    static uint8_t previous_status = 0;
    static float max_altitude = 0;
    uint8_t sistem_statusu = 0 ; // Varsayılan: Uçuşa Hazır

    // Maksimum yüksekliği takip et
    if (telemetri_paketi->altitude1 > max_altitude) {
        max_altitude = telemetri_paketi->altitude1;
    }

    // Yükseklik bazlı durum güncellemesi
    if (telemetri_paketi->altitude1 < 5.0 && max_altitude < 10.0) {
        sistem_statusu = 0; // Uçuşa Hazır (henüz yükselmemiş)
    }
    else if (telemetri_paketi->altitude1 > previous_altitude && telemetri_paketi->altitude2 > 50.0) {
        sistem_statusu = 1; // Yükselme (yükseliyor)
    }
    else if (telemetri_paketi->altitude1 > 400.0 && previous_status == 1) {
        sistem_statusu = 2; // Model Uydu İniş (400m'den sonra)
    }
    else if (telemetri_paketi->altitude1 <= 400.0 && telemetri_paketi->altitude1 > 50.0 && max_altitude > 400.0) {
        if (previous_status <= 2) {
            sistem_statusu = 2; // Model Uydu İniş devam ediyor
        } else {
            sistem_statusu = 3; // Ayrılma gerçekleşti
        }
    }
    else if (telemetri_paketi->altitude1 <= 50.0 && max_altitude > 400.0) {
        if (previous_status == 2) {
            sistem_statusu = 3; // Ayrılma (400m'den düştükten sonra)
        } else if (previous_status == 3) {
            sistem_statusu = 4; // Görev Yükü İniş
        }
    }
    else if (telemetri_paketi->altitude1 <= 10.0 && previous_status >= 3) {
        sistem_statusu = 5; // Kurtarma (yere indi)
    }

    previous_status = sistem_statusu;
    return previous_status;
}

/* Optimize edilmiş hata kodu oluşturma fonksiyonu */
void UpdateOptimizedErrorCode(OptimizedTelemetriPaketi_TypeDef *telemetri_paketi, uint8_t sistem_statusu, char *hata_durumu) {
    // Hata kodu pozisyonları:
    // Pozisyon 1: Model uydu iniş hızı (12-14 m/s dışında = 1)
    // Pozisyon 2: Görev yükü iniş hızı (6-8 m/s dışında = 1)
    // Pozisyon 3: Taşıyıcı basınç verisi (alınamıyorsa = 1)
    // Pozisyon 4: Görev yükü konum verisi (alınamıyorsa = 1)
    // Pozisyon 5: Ayrılma durumu (gerçekleşmezse = 1)
    // Pozisyon 6: Multi-spektral filtreleme sistemi (çalışmazsa = 1)
    strcpy(hata_durumu, "000000"); // Varsayılan: hata yok

    // Model uydu iniş hızı kontrolü (sistem durumu 2'de)
    if (sistem_statusu == 2) {
        if (telemetri_paketi->landing_speed < 12.0 || telemetri_paketi->landing_speed > 14.0) {
            hata_durumu[0] = '1';
        }
    }

    // Görev yükü iniş hızı kontrolü (sistem durumu 4'te)
    if (sistem_statusu == 4) {
        if (telemetri_paketi->landing_speed < 6.0 || telemetri_paketi->landing_speed > 8.0) {
            hata_durumu[1] = '1';
        }
    }

    // Taşıyıcı basınç verisi kontrol
    if (telemetri_paketi->pressure2 < 50000.0 || telemetri_paketi->pressure2 > 120000.0) {
        hata_durumu[2] = '1'; // Anormal basınç değeri
    }

    // GPS veri kontrol
    if (telemetri_paketi->gps1_latitude == 0.0 && telemetri_paketi->gps1_longitude == 0.0) {
        hata_durumu[3] = '1'; // GPS veri hatası
    }

    // Ayrılma durumu kontrol (400m civarında)
    if (telemetri_paketi->altitude1 <= 410.0 && telemetri_paketi->altitude1 >= 390.0 && sistem_statusu != 3) {
        hata_durumu[4] = '1'; // Ayrılma gerçekleşmedi
    }

    // Multi-spektral sistem kontrolü (RHRH komutu işlenip işlenmediği)
    if (strcmp(telemetri_paketi->rhrh, "0000") == 0 && sistem_statusu >= 3) {
        hata_durumu[5] = '1'; // Filtreleme sistemi çalışmıyor
    }
    hata_durumu[6] = '\0';
}
