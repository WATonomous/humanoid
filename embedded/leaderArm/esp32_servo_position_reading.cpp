// THIS FILE IS FROM A PLATFORMIO SETUP FOR A DEVKIT ESP32
#include <Arduino.h>
#include <driver/uart.h>

/**
 * PACKET_STRUCTURE:
 * 0xFF, 
 * 0xFF, 
 * Servo ID, 
 * Length, 
 * Instruction, 
 * Parameter1 (Register to look at), 
 * Parameter2 (Length of data to read), 
 * Checksum Byte 
 */

/**
 * CHECKSUM_FORMULA:
 * CheckSUM = ~(ID + Length + Parameter1 + Parameter2) & 0xF]
 */

// SERVO_INSTRUCTIONS:
const uint8_t STS_HEADER      = 0xFF;
const uint8_t STS_PING        = 0x01;
const uint8_t STS_READ        = 0x02;
const uint8_t STS_WRITE       = 0x03;
const uint8_t STS_REGWRITE    = 0x04;
const uint8_t STS_ACTION      = 0x05;
const uint8_t STS_RESET       = 0x06;
const uint8_t STS_SYNCWRITE   = 0x08;
const uint8_t STS_SYNCREAD    = 0x82;

// SERVO_REGISTERS:
const uint8_t INSTR_POSITION  = 0x38;
const uint8_t INSTR_SPEED     = 0x3A;
const uint8_t INSTR_LOAD      = 0x3C;
const uint8_t INSTR_VOLTAGE   = 0x3E;
const uint8_t INSTR_TEMP      = 0x3F;
const uint8_t INSTR_CURRENT   = 0x41;

// SERVO IDs
const uint8_t ID1             = 0;
const uint8_t ID2             = 1;
const uint8_t ID3             = 2;
const uint8_t ID4             = 3;
const uint8_t ID5             = 4;
const uint8_t ID6             = 5;

const uint8_t NUM_SERVOS      = 6;

// ESP CONFIGURATION
const uint8_t UART_NUM            = 18; // TBD which uart pin to using
const uint8_t CUTOFF_TIME_MILLIS  = 50;
const uint8_t packetLengthRead    = 8;
const uint8_t packetLengthPing    = 6;
HardwareSerial servoSerial(1);

uint8_t txPacket[packetLengthRead] = {0};
uint8_t rxPacket[packetLengthRead] = {0};

uint8_t calculate_checksum(uint8_t packet[],int packetLength){
  
  unsigned int checkSum = 0;
  
  for (int i=2;i<packetLength;i++){
    checkSum += packet[i];
  }
  
  return (byte)(~checkSum);
}

void build_reading_packet(uint8_t* packet, uint8_t header1, uint8_t header2, uint8_t ID, uint8_t param1, uint8_t param2) {

  packet[0] = header1;
  packet[1] = header2;
  packet[2] = ID;
  packet[3] = 2;
  packet[4] = STS_READ;
  packet[5] = param1;
  packet[6] = param2;
  packet[7] = 0;
  packet[7] = calculate_checksum( packet, packetLengthRead );
}

void ping_packet_builder(uint8_t* packet, uint8_t header1, uint8_t header2, uint8_t ID, uint8_t param1, uint8_t param2) {
  packet[0] = header1;
  packet[1] = header2;
  packet[2] = ID;
  packet[3] = 2;
  packet[4] = STS_READ;
  packet[5] = 0;
  packet[5] = calculate_checksum( packet, packetLengthPing );
}

void servo_position( uint8_t* packet, uint8_t servoID ) {
  build_reading_packet( packet, STS_HEADER, STS_HEADER, servoID, INSTR_POSITION, 12 );
}

// packet_decode(&rx_packet, timeout) writes recieved packet to rx_packet, waiting timout ms for the bytes to show up before failing
bool read_packet_decode(uint8_t *rx_packet, int timeout){ 
  uint8_t buf[8];
  int length = uart_read_bytes(UART_NUM, buf, 8, pdMS_TO_TICKS(timeout));

  // Verify buffer
  if (length != 8          ||
      buf[0] != STS_HEADER || 
      buf[1] != STS_HEADER ||
      calculate_checksum(buf, packetLengthRead) != buf[7]
      ) {
    return false;
  }

  for (int i = 0; i < packetLengthRead; ++i){
    rx_packet[i] = buf[i];
  }

  return true;
}

/**
 * Position of the servo from rx, might need to consider the id
 */
uint16_t position_from_rx(uint8_t *rx_packet){
  uint8_t lowBit = rx_packet[5];
  uint8_t highBit = rx_packet[6];
  return lowBit | (highBit << 8);
}

void setup() {
  // rate, format, rx, tx !!! since half duplex, rx and tx are shared
  servoSerial.begin(1000000, SERIAL_8N1, 18, 18);

  // for debugging purposes
  Serial.begin(1000000);
}

// NEEDS TO BE WRITTEN!
void loop() {

  uint8_t idArray[NUM_SERVOS] = {ID1, ID2, ID3, ID4, ID5, ID6};

  for( int i = 0; i < NUM_SERVOS; ++i ) {

    // read all 6 servos
    servo_position(txPacket, idArray[i]);
    servoSerial.write(txPacket, packetLengthRead);
    servoSerial.flush();
    
    read_packet_decode( rxPacket, CUTOFF_TIME_MILLIS );
    uint8_t pos = position_from_rx( rxPacket );

    Serial.printf( "SERVO %d: %d\n", i, pos );
  }
}