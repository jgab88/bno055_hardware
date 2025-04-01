/**
 * Implementation for the VL53L5CX platform abstraction layer for Teensy 4.1
 * Implements the required functions for the ST VL53L5CX driver
 */

 #include <Arduino.h>
 #include <Wire.h>
 
 // Include the API header with C linkage to match the platform.h declarations
 #ifdef __cplusplus
 extern "C" {
 #endif
 #include "vl53l5cx_api.h"
 
 // Implementation of platform functions declared in platform.h
 uint8_t VL53L5CX_RdByte(
         VL53L5CX_Platform *p_platform,
         uint16_t RegisterAdress,
         uint8_t *p_value)
 {
   Wire1.beginTransmission((uint8_t)((p_platform->address) >> 1) & 0x7F);
   Wire1.write((uint8_t)(RegisterAdress >> 8));
   Wire1.write((uint8_t)(RegisterAdress & 0xFF));
   uint8_t status = Wire1.endTransmission(false);
   
   if (status != 0) {
     return status;
   }
   
   Wire1.requestFrom(((uint8_t)(p_platform->address >> 1) & 0x7F), 1);
   if (Wire1.available()) {
     *p_value = Wire1.read();
   } else {
     return 1; // Error - no data available
   }
   
   return 0;
 }
 
 uint8_t VL53L5CX_WrByte(
         VL53L5CX_Platform *p_platform,
         uint16_t RegisterAdress,
         uint8_t value)
 {
   Wire1.beginTransmission((uint8_t)((p_platform->address) >> 1) & 0x7F);
   Wire1.write((uint8_t)(RegisterAdress >> 8));
   Wire1.write((uint8_t)(RegisterAdress & 0xFF));
   Wire1.write(value);
   return Wire1.endTransmission();
 }
 
 uint8_t VL53L5CX_WrMulti(
         VL53L5CX_Platform *p_platform,
         uint16_t RegisterAdress,
         uint8_t *p_values,
         uint32_t size)
 {
   uint8_t status = 0;
   uint32_t i = 0;
   
   while (i < size && status == 0) {
     uint32_t chunk_size = size - i;
     if (chunk_size > 32) chunk_size = 32; // I2C buffer size limit for Teensy
     
     Wire1.beginTransmission((uint8_t)((p_platform->address) >> 1) & 0x7F);
     Wire1.write((uint8_t)(RegisterAdress >> 8));
     Wire1.write((uint8_t)(RegisterAdress & 0xFF));
     
     for (uint32_t j = 0; j < chunk_size; j++) {
       Wire1.write(p_values[i + j]);
     }
     
     status = Wire1.endTransmission(true);
     i += chunk_size;
     RegisterAdress += chunk_size;
   }
   
   return status;
 }
 
 uint8_t VL53L5CX_RdMulti(
         VL53L5CX_Platform *p_platform,
         uint16_t RegisterAdress,
         uint8_t *p_values,
         uint32_t size)
 {
   uint8_t status = 0;
   uint32_t i = 0;
   
   Wire1.beginTransmission((uint8_t)((p_platform->address) >> 1) & 0x7F);
   Wire1.write((uint8_t)(RegisterAdress >> 8));
   Wire1.write((uint8_t)(RegisterAdress & 0xFF));
   status = Wire1.endTransmission(false);
   
   if (status != 0) {
     return status;
   }
   
   while (i < size) {
     uint32_t chunk_size = size - i;
     if (chunk_size > 32) chunk_size = 32; // I2C buffer size limit for Teensy
     
     Wire1.requestFrom(((uint8_t)(p_platform->address >> 1) & 0x7F), chunk_size);
     uint32_t j = 0;
     while (Wire1.available() && j < chunk_size) {
       p_values[i + j] = Wire1.read();
       j++;
     }
     
     if (j != chunk_size) {
       return 1; // Error - not enough data read
     }
     
     i += chunk_size;
   }
   
   return status;
 }
 
 uint8_t VL53L5CX_WaitMs(
         VL53L5CX_Platform *p_platform,
         uint32_t TimeMs)
 {
   (void)p_platform; // Avoid unused parameter warning
   delay(TimeMs);
   return 0;
 }
 
 void VL53L5CX_SwapBuffer(
         uint8_t *buffer,
         uint16_t size)
 {
   uint32_t i, tmp;
   
   // Example of possible implementation using standard library
   for(i = 0; i < size; i = i + 4) 
   {
     tmp = (
       (buffer[i] << 24) |
       (buffer[i+1] << 16) |
       (buffer[i+2] << 8) |
       (buffer[i+3])
     );
     
     memcpy(&(buffer[i]), &tmp, 4);
   }
 }
 
 #ifdef __cplusplus
 } // extern "C"
 #endif