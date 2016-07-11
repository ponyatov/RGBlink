
#define BUFFER_LENGTH 108 //edited in wire.h !!!!!!!!!!!!!!!

#include <Wire.h>

#include "Adafruit_NeoPixel_Ex.h"

#define PIN 8
#define LED 13
#define NUMBER_OF_LEDS 36
#define STROBE_SIZE 3*NUMBER_OF_LEDS // 3 bytes per pixel

#define BUFFER_SIZE 2048
#define STROBE_COUNT 10
#define MAX_COMMAND_LENGHT 25

#define PICTURE_SIZE 108
#define BEGIN_OF_PICTURES  64
#define EEPROM_DEVICE_ADDRESS 80
#define DEVICE_ADDRESS 80

#define WAIT_DATA_TRANSFER_TIMEOUT 500

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMBER_OF_LEDS, PIN, NEO_GRB + NEO_KHZ800);

int nColor = 0;
int nStep = 1;

const byte PACKET_SIZE = 32;

uint32_t stripColor = 0;

String incomingString;

uint8_t* ledStrip;

uint8_t* receiveBuffer = new uint8_t[50]; // magic number > PACKET_SIZE + sizeof(packetheader)

uint8_t* buf = new uint8_t[STROBE_SIZE];

unsigned long timeStart;
unsigned long timeStop;
unsigned long timeStopRead;

bool bShowDelay = false;

bool bWaitDataTransfer = false;
unsigned long timeWaitDataTransfer;

struct PACKET_HEADER
{
  byte chunkLength = 0;
  byte pictureNumber = 0;
  byte pictureHeight = 0;
  byte pictureWidth = 0;
  byte strobeNumber = 0;

  byte packetsTotal = 0; // number of packets per strobe
  byte packetSize = 0;
  byte packetNumber = 0;

  //byte crc = 0;

} packetheader;


int index = 0;

void setup() {

  Wire.begin(); // join i2c bus (address optional for master)
  Wire.setClock(400000);

  strip.begin();

  Serial.begin(57600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  pinMode(LED, OUTPUT);

  strip.show(); // Initialize all pixels to 'off'

  incomingString.reserve(MAX_COMMAND_LENGHT);

  delay(100);

}

void ledOn() {
  digitalWrite(LED, HIGH);
}

void ledOff() {
  digitalWrite(LED, LOW);
}


void loop() {

  timeStart = micros();

  //memset(buf, STROBE_SIZE, 0x00);

  if (!bWaitDataTransfer) {

    i2c_eeprom_read_buffer(DEVICE_ADDRESS, BEGIN_OF_PICTURES + index * STROBE_SIZE, buf, STROBE_SIZE);

    index++;

    timeStopRead = micros();

    strip.setPixels((uint8_t*)buf);

    strip.show();

  }

  timeStop = micros();

  if (index > PICTURE_SIZE - 1)
  {
    index = 0;
  }

  if (bShowDelay) {
    Serial.print(index);
    Serial.print(F(":   Delay read ="));
    Serial.print(timeStopRead - timeStart);
    Serial.print(F("   Overall delay ="));
    Serial.println(timeStop - timeStart);
  }

  //delay(10);

  GetSerialData();
}

void GetSerialData() {
  if (Serial.available())
  {
    char readChar = Serial.read();
    //Serial.print(readChar);

    if (readChar == 0x0A)
    {
      parseCommand();
    }
    else if (readChar == 0x0D)
    {
      //parseCommand();
    }
    else
    {
      incomingString = incomingString + readChar;
    }
  }

  if(bWaitDataTransfer){
    if(millis() > timeWaitDataTransfer){
      bWaitDataTransfer = false;
    }
  }
}

int ReceiveData() {
  // set timer
  uint32_t ulStartTime = millis();

  int result = 0;

  int ndx = 0;
  char readChar;

  while (1) {

    if (Serial.available()) {
      readChar = Serial.read();
      receiveBuffer[ndx] = readChar;
      ndx++;

      if (ndx >= receiveBuffer[0]) {
        //Serial.print(F("receiveBuffer[0] = "));
        //Serial.println(receiveBuffer[0]);
        ledOn();

        while (Serial.available()) {
          readChar = Serial.read();
          Serial.print(F("Extented data!"));
          Serial.print(readChar);
        }

        result = ndx;
        incomingString = "";
        break;
      }
      ulStartTime = millis();
    }

    if (millis() - ulStartTime > 2000) {
      Serial.println(F("ReceiveData() timeout!"));
      //ledOff();
      result = 0;
      break;
    }
  }
  //
  //  Serial.print(F("ndx ="));
  //Serial.println(ndx);

  return result;
}

uint8_t StoreData() {
  uint8_t i = 0;
  uint8_t res = 0;

  memcpy(&packetheader, receiveBuffer, sizeof(packetheader));

  /*
    Serial.print(F("chunkLength = "));
    Serial.println(packetheader.chunkLength);
    Serial.print(F("pictureNumber = "));
    Serial.println(packetheader.pictureNumber);
    Serial.print(F("pictureHeight = "));
    Serial.println(packetheader.pictureHeight);
    Serial.print(F("pictureWidth = "));
    Serial.println(packetheader.pictureWidth);
    Serial.print(F("strobeNumber = "));
    Serial.println(packetheader.strobeNumber);
    Serial.print(F("packetsTotal = "));
    Serial.println(packetheader.packetsTotal);
    Serial.print(F("packetSize = "));
    Serial.println(packetheader.packetSize);
    Serial.print(F("packetNumber = "));
    Serial.println(packetheader.packetNumber);
  */

  uint32_t strobePosition = packetheader.strobeNumber * STROBE_SIZE; // strobe position

  if (packetheader.strobeNumber > PICTURE_SIZE) {
    strobePosition = (PICTURE_SIZE - 1) * STROBE_SIZE;
  }

  uint32_t packetPosition = packetheader.packetNumber * packetheader.packetSize; // packet data offset

  if (packetheader.packetNumber == packetheader.packetsTotal - 1) {
    packetPosition = STROBE_SIZE - packetheader.packetSize;
  }

  if (packetPosition + packetheader.packetSize > STROBE_SIZE ) {
    packetPosition = STROBE_SIZE - packetheader.packetSize;
  }

  uint32_t lStartAddress = BEGIN_OF_PICTURES // device header offset
                           + packetheader.pictureNumber * PICTURE_SIZE  // picture offset
                           + strobePosition
                           + packetPosition;

  //  Serial.print(F("Packet storage start address = "));
  //  Serial.println(lStartAddress);

  res = saveData(lStartAddress, &receiveBuffer[sizeof(packetheader)], packetheader.packetSize);

  if (res > 0) {
    Serial.println(F("Error data writing!"));
  }
  else {
    Serial.println(F("Data write completed!"));
  }

  return res;
}

uint8_t saveData(int eeaddress, byte* data, byte datasize ) {

  uint8_t res = 0;

  Serial.println(F("Start saving data."));
  Serial.print(F("Size of data = "));
  Serial.println(datasize);
  Serial.print(F("Start address = "));
  Serial.println(eeaddress);

  for (int i = 0; i < datasize; i++) {
    res = res + i2c_eeprom_write_byte(DEVICE_ADDRESS, eeaddress + i, data[i]);
  }

  return res;
}


uint8_t i2c_eeprom_write_byte( int deviceaddress, int eeaddress, byte data ) {
  uint8_t rdata = data;
  uint8_t res = 0;
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8)); // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(rdata);
  res = Wire.endTransmission();
  if (res != 0)
  {
    Serial.print(F("Error writing to !  = "));
    Serial.println(eeaddress);
  }
  delay(10);
  return res;
}

void i2c_eeprom_read_buffer( int deviceaddress, unsigned int eeaddress, uint8_t *data, int len ) {
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8)); // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom(deviceaddress, len);
  int c = 0;
  for ( c = 0; c < len; c++ )
    if (Wire.available()) {
      data[c] = (uint8_t)Wire.read();
    }
}

void parseCommand() {
  Serial.println(incomingString);

  if (incomingString == F("SetShowDelay"))
  {
    bShowDelay = true;
  }

  if (incomingString == F("ClearShowDelay"))
  {
    bShowDelay = false;
  }

  if (incomingString == F("DT"))
  {
    bWaitDataTransfer = true;
    
    int ndx = ReceiveData();
    
    if (ndx > 0)
    {
      if (StoreData() < 1) {
        Serial.print(F("ndx ="));
        Serial.println(ndx);

        ledOff();

        timeWaitDataTransfer = millis() + WAIT_DATA_TRANSFER_TIMEOUT;

        bWaitDataTransfer = true;
      }
    }
    else{
              bWaitDataTransfer = false;
    }
  }

  incomingString = "";
}
