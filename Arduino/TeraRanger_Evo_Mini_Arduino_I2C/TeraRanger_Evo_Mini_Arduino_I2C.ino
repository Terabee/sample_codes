/*
 * This code allows you to check if the I2C communication between the TeraRanger and the Arduino board works.
 * You just need to upload the code to the board with the TeraRanger connected on pins SDA, SCL, GND and 5V.
 * If the supply of the TeraRanger is stopped or interrupted while the code is running, just press the reset button on the board.
 * There are two way to check if the communication works:
 * - the first is to open the serial monitor and check if the distance is printed like this: "Distance in mm: XXX";
 * - the second is to observe the LED "L" on the board, the brightness of the LED will change with the distance measured by the TeraRanger One (Range: 20cm to 1m).
 * If the communication works you can remove the ERASABLE part of the code shown below.
 * Warning: Make sure that you connect the right pins of the TeraRanger One to your Arduino, for I2C operation you need to modify the open ended cable provided in the box.
 * Default I2C base address of the TeraRanger one is 0x30.
 */
 

#include <Wire.h>
// For TeraRanger Evo
#define SENSOR_ADDR 0x31  

// Create a Cyclic Redundancy Checks table used in the "crc8" function
static const uint8_t crc_table[] = {
    0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31,
    0x24, 0x23, 0x2a, 0x2d, 0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
    0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d, 0xe0, 0xe7, 0xee, 0xe9,
    0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
    0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1,
    0xb4, 0xb3, 0xba, 0xbd, 0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
    0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea, 0xb7, 0xb0, 0xb9, 0xbe,
    0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
    0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16,
    0x03, 0x04, 0x0d, 0x0a, 0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
    0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a, 0x89, 0x8e, 0x87, 0x80,
    0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
    0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8,
    0xdd, 0xda, 0xd3, 0xd4, 0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
    0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44, 0x19, 0x1e, 0x17, 0x10,
    0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
    0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f,
    0x6a, 0x6d, 0x64, 0x63, 0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
    0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13, 0xae, 0xa9, 0xa0, 0xa7,
    0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
    0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef,
    0xfa, 0xfd, 0xf4, 0xf3
};

/*
 * Brief : Calculate a Cyclic Redundancy Checks of 8 bits
 * Param1 : (*p) pointer to receive buffer
 * Param2 : (len) number of bytes returned by the TeraRanger
 * Return : (crc & 0xFF) checksum calculated locally
 */
uint8_t crc8(uint8_t *p, uint8_t len) {
  uint8_t i;
  uint8_t crc = 0x0;
  while (len--) {
    i = (crc ^ *p++) & 0xFF;
    crc = (crc_table[i] ^ (crc << 8)) & 0xFF;
  }
  return crc & 0xFF;
}

uint8_t buf[3];           // The variable "buf[3]" will contain the frame sent by the TeraRanger
uint16_t distance = 0;    // The variable "distance" will contain the distance value in millimeter
uint8_t CRC = 0;          // The variable "CRC" will contain the checksum to compare at TeraRanger's one

const byte short_mode[2] = {0x02,0x01};
const byte long_mode[2] = {0x02,0x03};

int inter = 25; // Inter reading delay, replace by 50 if using long mode

void setup() {
  pinMode(13, OUTPUT);    // Initialize digital pin 13 as an output (ERASABLE if the communication works)
  Serial.begin(115200);   // Open serial port 0 (which corresponds to USB port), set data rate to 115200 bps
  Wire.begin();           // Join I2C bus as master

  // Set your sensor mode
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(short_mode, 2);
  //Wire.write(long_mode, 2); // Sensor is in long mode when powered up
  Wire.endTransmission();
  delay(25);                  // This delay is necessary to ensure mode is switched correctly
}

// The main loop starts here
void loop() {
  
  Wire.beginTransmission(SENSOR_ADDR);  // Transmit to Evo Mini (THIS IS THE I2C BASE ADDRESS, CHANGE HERE IN CASE IT IS DIFFERENT)
  Wire.write(0x00);                     // Sends measure trigger byte
  Wire.endTransmission();               // Stop transmitting
  
  Wire.requestFrom(SENSOR_ADDR, 3);     // Read back three bytes from Evo Mini (THIS IS THE I2C BASE ADDRESS, CHANGE HERE IN CASE IT IS DIFFERENT)
  buf[0] = Wire.read();                 // First byte of distance
  buf[1] = Wire.read();                 // Second byte of distance
  buf[2] = Wire.read();                 // Byte of checksum
  
  CRC = crc8(buf, 2);                   // Save the "return" checksum in variable "CRC" to compare with the one sent by the TeraRanger
  
  if (CRC == buf[2]) {                  // If the function crc8 return the same checksum than the TeraRanger, then:
    distance = (buf[0]<<8) + buf[1];    // Calculate distance in mm
    Serial.print("Distance in mm : ");
    Serial.println(distance);
  }
  else {
    Serial.println("CRC error!");
  }

  delay(inter);                         // This delay is necessary to prevent reading too many times the same measurement (To be adapted depending on the sensor mode)
}
