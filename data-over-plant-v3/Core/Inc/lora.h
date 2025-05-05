/*
 * lora.h
 *
 *  Created on: May 5, 2025
 *      Author: gilda
 */

#ifndef INC_LORA_H_
#define INC_LORA_H_

#include "main.h"
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#define LORA_NSS_GPIO_Port GPIOF
#define LORA_NSS_Pin       GPIO_PIN_6
#define LORA_RST_GPIO_Port GPIOC
#define LORA_RST_Pin       GPIO_PIN_3

typedef struct {
    uint8_t buffer[256];  // Buffer pour stocker les données reçues
    uint8_t size;         // Taille des données reçues
    int8_t rssi;          // Force du signal
    float snr;            // Rapport signal/bruit
} LoRaPacket_t;

void write_register(uint8_t addr, uint8_t value);
uint8_t read_register(uint8_t addr);
void init_lora_transmitter(void);
void init_lora_receiver(void);
void init_lora(void);
void lora_reset(void);
uint8_t lora_version(void);
uint8_t lora_status(void);
void lora_clear_irq(void);
char lora_is_tx_done(void);
void send_lora_packet(const uint8_t* data, uint8_t size);
bool lora_is_packet_available(void);
bool lora_has_crc_error(void);
LoRaPacket_t lora_receive_packet(void);

#endif /* INC_LORA_H_ */
