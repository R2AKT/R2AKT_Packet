#include <r2akt_packet.h>

#define PACKET_SRC_ADDR 0x00 // 0x00 = Master, 0x01 - 0xFE = Slave, 0xFF = Broadcast
#define PACKET_MAX_SIZE 250 // Max packet data size. If PACKET_MAX_SIZE > 250, autoset PACKET_CODEC to 0 (KISS (SLIP))
#define PACKET_SPEED 9600 // Packet speed
#define PACKET_TX_TOGLE 13 // For RS-485 module togle or/and indicate Tx state
#define PACKET_CODEC 1 //  0 = KISS (SLIP), 1 = COBS (max packet size 250 byte, if PACKET_MAX_SIZE > 250, set PACKET_CODEC to 0 (KISS (SLIP)))

//
//#define USE_SW_SERIAL
#ifdef USE_SW_SERIAL
  #include <SoftwareSerial.h>
  #define PACKET_RX_PIN 11 // Rx pin for SoftwareSerial (unused if HWSerial)
  #define PACKET_TX_PIN 12 // Tx pin for SoftwareSerial (unused if HWSerial)
  SoftwareSerial SWSerial = SoftwareSerial (PACKET_RX_PIN, PACKET_TX_PIN);
  Packet packet (&SWSerial, PACKET_SRC_ADDR, PACKET_MAX_SIZE, 0, PACKET_TX_TOGLE);
#else
  Packet packet (&Serial, PACKET_SRC_ADDR, PACKET_MAX_SIZE, 0, PACKET_TX_TOGLE);
#endif

void setup() {
  #ifdef USE_SW_SERIAL
    SWSerial.begin (PACKET_SPEED);
  #else
    Serial.begin (PACKET_SPEED);
  #endif

  packet.begin();

  uint8_t Buff[] = {0xFF,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0x88,0x77,0x66,0x55,0x44,0x33,0x22,0x11,0x0};
  uint8_t mac_dst_addr = 0x01;
  packet.send_phy (Buff, sizeof (Buff)/sizeof(Buff[0]));
  packet.send_mac (mac_dst_addr, Buff, sizeof (Buff)/sizeof (Buff[0]));
  packet.packet_send_to (mac_dst_addr, Buff, sizeof (Buff)/sizeof (Buff[0]));
}

///
void loop() {
  uint8_t Exchange_Buff[PACKET_MAX_SIZE] = {};
  //uint8_t Exchange_Buff[] = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0x88,0x77,0x66,0x55,0x44,0x33,0x22,0x11};

  // int16_t phy_rx_len;
  // do {
  //   phy_rx_len = packet.receive_phy (Exchange_Buff);
  //   if (phy_rx_len > 0) {
  //     packet.send_phy (Exchange_Buff, phy_rx_len);
  //   }
  // } while (phy_rx_len <= 0);

  // uint8_t mac_src_addr;
  // int16_t mac_rx_len;
  // do {
  //   mac_rx_len = packet.receive_mac (Exchange_Buff, &mac_src_addr);
  //   if (mac_rx_len > 0) {
  //     packet.send_mac (mac_src_addr, Exchange_Buff, mac_rx_len);
  //   }
  // } while (mac_rx_len <= 0);

  uint8_t packet_dst_addr = 0x2;
  uint8_t packet_src_addr = 0x1;
  int16_t packet_rx_len;
  do {
    packet_rx_len = packet.packet_receive_from (Exchange_Buff, packet_src_addr);
    if (packet_rx_len > 0) {
      packet.packet_send_to (packet_dst_addr, Exchange_Buff, packet_rx_len);
    }
  } while (packet_rx_len <= 0);
}