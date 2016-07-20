/* UART repeater adopted for HC05 i/o (used Communication/MultiSerial example) */

#define LED       13  /* LED pin */
#define HCSTATUS  20  /* SDA pin */
#define HCKEY     21  /* SCL pin */
void setup() {
  Serial.begin(115200);       // USB/serial
  Serial1.begin(38400);       // HC05 i/o 38400/57600
  pinMode(LED,      OUTPUT);  // setup LED pin
  pinMode(HCSTATUS,  INPUT);  // setup HC05 status input pin
  pinMode(HCKEY,    OUTPUT);  // setup HC05 control pin
  digitalWrite(HCKEY,true);   // enable HC key
}

void loop() {
  // read from port 1, send to port 0:
  if (Serial1.available()) {
    int inByte = Serial1.read();
    Serial.write(inByte);
  }

  // read from port 0, send to port 1:
  if (Serial.available()) {
    int inByte = Serial.read();
    Serial1.write(inByte);
    Serial.write(inByte);   // echo
  }

  // copy status from HC05 to Arduino LED
  digitalWrite(LED, digitalRead(HCSTATUS));
}
