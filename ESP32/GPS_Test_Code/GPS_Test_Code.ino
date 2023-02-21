#include <HardwareSerial.h>

const int led = 2;
const int enablePin = 15;
const int txPin = 17; // needs changing
const int rxPin = 16; // needs changing

HardwareSerial GPSSerial(1);

void setup() {
  pinMode(led, OUTPUT);
  pinMode(enablePin, OUTPUT);
  GPSSerial.begin(4800, SERIAL_8N1, rxPin, txPin);

  // turn on GPS
  digitalWrite(enablePin, HIGH);

  Serial.begin(115200);
  GPSSerial.setTimeout(1000);
}

void loop() {
  confirmTest();
  forwardGPSOutput();

  //GPSSerial.print("$PSRF103,0,1,01,0");
}

void forwardGPSOutput()
{
  if (GPSSerial.available() > 0)
  {
    Serial.print("Data from GPS: ");
    Serial.println(GPSSerial.readString()); // this seems to hang, no timeout??
  }
}

void confirmTest()
{
  digitalWrite(2, HIGH);
  delay(1000);
  digitalWrite(2, LOW);
  delay(1000);
  Serial.println("yo");
}
