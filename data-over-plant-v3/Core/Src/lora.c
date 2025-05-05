/*
 * lora.c
 *
 *  Created on: May 5, 2025
 *      Author: gilda
 */

#include "lora.h"

void init_lora_transmitter()
{
	// Mettre en veille (obligatoire avant les changements de mode)
	write_register(0x01, 0x80); // Sleep mode + LoRa

	// Choisir fréquence (ici pour 868 MHz)
	write_register(0x06, 0xD9); // RegFrfMsb = 868 MHz
	write_register(0x07, 0x06);
	write_register(0x08, 0x66);

	// Puissance d’émission (PA_BOOST, 17 dBm)
	//write_register(0x09, 0x8F); // RegPaConfig

	// 0 000 0000
	write_register(0x09, 0x00); // RegPaConfig: PA_SELECT=1 (PA_BOOST), OutputPower=0 (minimum)

	// Paramètres LoRa (BW=125 kHz, CR=4/5, SF=7)
	write_register(0x1D, 0x72); // RegModemConfig1: BW=125kHz, CR=4/5, explicit header
	write_register(0x1E, 0x74); // RegModemConfig2: SF=7, CRC On
	write_register(0x26, 0x04); // RegModemConfig3: LowDataRateOptimize=OFF

	// Configuration de la FIFO
	write_register(0x0E, 0x80); // RegFifoTxBaseAddr
	write_register(0x0D, 0x80); // RegFifoAddrPtr
	unsigned char size = 7;
	write_register(0x22, size); // RegPayloadLength (taille du message)

	// Écriture des données dans le FIFO
	/*for (int i = 0; i < size; i++) {
	  write_register(0x00, TX_Buffer[i]); // RegFifo
	}*/

	// Passage en mode émetteur
	write_register(0x01, 0x83); // TX mode + LoRa
}

// Fonction pour initialiser le mode récepteur
void init_lora_receiver(void)
{
    // Mettre en veille (obligatoire avant les changements de mode)
    write_register(0x01, 0x80); // Sleep mode + LoRa

    // Choisir fréquence (ici pour 868 MHz - même fréquence que l'émetteur)
    write_register(0x06, 0xD9); // RegFrfMsb = 868 MHz
    write_register(0x07, 0x06);
    write_register(0x08, 0x66);

    // Paramètres LoRa (BW=125 kHz, CR=4/5, SF=7) - comme l'émetteur
    write_register(0x1D, 0x72); // RegModemConfig1: BW=125kHz, CR=4/5, explicit header
    write_register(0x1E, 0x74); // RegModemConfig2: SF=7, CRC On
    write_register(0x26, 0x04); // RegModemConfig3: LowDataRateOptimize=OFF

    // BEGIN TEST
    /*write_register(0x1D, 0x93); // RegModemConfig1: BW=125kHz, CR=4/5, explicit header
    write_register(0x1E, 0x64); // RegModemConfig2: SF=6, CRC On
    write_register(0x26, 0x04); // RegModemConfig3: LowDataRateOptimize=OFF
    write_register(0x37, 0x0C);
    // Write bits 2-0 of register address 0x31 to value "0b101"
    // 0100 0101 = 0x45
    write_register(0x31, 0x45);                 // Écrire la nouvelle valeur*/
    // END TEST

    // Optimisation pour la réception
    write_register(0x0F, 0x00); // RegFifoRxBaseAddr - adresse de base pour le buffer de réception

    // Configurer LNA (Low Noise Amplifier) pour la réception
    write_register(0x0C, 0x23); // RegLna: gain élevé, boost On

    // Configurer la détection de préambule
    write_register(0x1F, 0x08); // RegPreambleDetect: 1 octet de préambule requis

    // Régler le seuil de détection pour optimiser la sensibilité
    write_register(0x20, 0x00); // RegDetectionThreshold: valeur par défaut

    // Effacer tous les drapeaux d'interruption
    write_register(0x12, 0xFF);

    // Activer les interruptions RxDone et PayloadCrcError
    write_register(0x11, 0x00); // RegIrqFlags: désactiver toutes les interruptions

    // Passer en mode réception continue
    write_register(0x01, 0x85); // RX continuous mode + LoRa

    serial_print("Module LoRa initialisé en mode réception\r\n");
}

void init_lora(void)
{
#ifdef TRANSMITTER
	init_lora_transmitter();
#endif
#ifdef RECEIVER
	init_lora_receiver();
#endif
}

void lora_reset(void)
{
    HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10); // maintenir RESET à 0 pendant 10ms
    HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(10); // attendre encore un peu après le relâchement
}

uint8_t lora_version(void)
{
	uint8_t version = read_register(0x42);

	return version;
}

uint8_t lora_status(void)
{
    return read_register(0x12); // RegIrqFlags
}

void lora_clear_irq(void)
{
    write_register(0x12, 0xFF); // Effacer tous les drapeaux d'interruption
}

char lora_is_tx_done(void)
{
    return (lora_status() & 0x08) != 0; // Vérifie le drapeau TxDone
}

void send_lora_packet(const uint8_t* data, uint8_t size)
{
    // Mettre en mode veille
    write_register(0x01, 0x81);  // Standby mode + LoRa

    // Configurer le pointeur FIFO pour l'écriture
    write_register(0x0D, 0x80);  // RegFifoAddrPtr

    // Écrire les données dans la FIFO
    for (int i = 0; i < size; i++) {
        write_register(0x00, data[i]);  // RegFifo
    }

    // Définir la taille des données
    write_register(0x22, size);  // RegPayloadLength

    // Démarrer la transmission
    write_register(0x01, 0x83);  // TX mode + LoRa
}

// Vérifier si un paquet est disponible
bool lora_is_packet_available(void)
{
    uint8_t irq_flags = read_register(0x12); // RegIrqFlags
    return (irq_flags & 0x40) != 0; // RxDone flag
}

// Vérifier si la réception contient une erreur CRC
bool lora_has_crc_error(void)
{
    uint8_t irq_flags = read_register(0x12); // RegIrqFlags
    return (irq_flags & 0x20) != 0; // PayloadCrcError flag
}

// Lire un paquet complet avec informations supplémentaires
LoRaPacket_t lora_receive_packet(void)
{
    LoRaPacket_t packet = {0};

    // Vérifier si un paquet est disponible et pas d'erreur CRC
    if (lora_is_packet_available() && !lora_has_crc_error()) {
        // Récupérer les informations de qualité du signal
        packet.rssi = -137 + read_register(0x1A); // RegRssiValue (en dBm)

        // Convertir la valeur SNR lue (complément à 2 sur 8 bits)
        int8_t raw_snr = (int8_t)read_register(0x19); // RegPktSnrValue
        packet.snr = raw_snr * 0.25f; // Convertir en dB

        // Récupérer la taille du paquet
        packet.size = read_register(0x13); // RegRxNbBytes

        // Lire l'adresse du dernier octet dans la FIFO
        uint8_t current_addr = read_register(0x10); // RegFifoRxCurrentAddr

        // Positionner le pointeur de FIFO à l'adresse du début de la réception
        write_register(0x0D, current_addr); // RegFifoAddrPtr

        // Lire les données
        for (uint8_t i = 0; i < packet.size && i < sizeof(packet.buffer); i++) {
            packet.buffer[i] = read_register(0x00); // RegFifo
        }

        // Afficher les informations de réception
        //char buf[128];
        //snprintf(buf, sizeof(buf), "Paquet reçu (%d octets), RSSI: %d dBm, SNR: %.2f dB\r\n", packet.size, packet.rssi, packet.snr);
        //serial_print(buf);

        // Afficher le contenu du paquet (en ASCII)
        /*serial_print("Contenu: ");
        serial_print(packet.buffer);
        serial_print("\r\n");*/

        // Effacer le drapeau RxDone
        write_register(0x12, 0x40);
    }

    return packet;
}
