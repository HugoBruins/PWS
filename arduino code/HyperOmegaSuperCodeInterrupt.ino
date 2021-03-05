#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <SD.h>

SoftwareSerial mySerial(6, 5);
TinyGPSPlus gps;

static const int RXPin = 3, TXPin = 2;
static const uint32_t GPSBaud = 9600;
SoftwareSerial ss(RXPin, TXPin);

const int chipSelect = 4;
Adafruit_BMP280 bmp; // I2C
const int BMP_address = 0x76;
String filename = "data";


String gpsLat = "-";
String gpsLon = "-";
String dataString;

//dit is nodig om later het verschil in luchtdruk te meten
float gemiddeldeLuchtdruk;
float gemiddeldeTemperatuur;

float constante;
float lapseRate = -0.0065;     //kelvin per meter in de troposfeer
int R = 287;                   //gasconstante
float g = -9.81;               //zwaartekracht volgens SIA
float hoogte;
long loop_timer = millis();

void setup() {

  Serial.begin(9600);
  mySerial.begin(9600);
  ss.begin(GPSBaud);
  randomSeed(analogRead(1));


  Serial.println("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    mySerial.println("sdKaart faalt hem jammer dit");
  }

  bmp.begin(BMP_address);

  constante = (1 / (g / (lapseRate * R)));

  //de sensor warmdraaien voor de nulmeting
  for (int i = 0; i <= 100; i++) {
    bmp.readPressure();
    bmp.readTemperature();
  }

  //nulmeting
  for (int i = 0; i <= 999; i++) {
    gemiddeldeLuchtdruk += bmp.readPressure();
    gemiddeldeTemperatuur += bmp.readTemperature();
  }

  gemiddeldeLuchtdruk = gemiddeldeLuchtdruk / 1000;
  gemiddeldeTemperatuur = (gemiddeldeTemperatuur / 1000) + 273.15;


  //geeft het bestand een willekeurig nummer aan het einde, zodat elke datalezing uniek is zonder dat er internet nodig is (voor tijd).
  int randomnumber =  random(10000);
  String nummer = String(randomnumber);
  filename = filename + nummer + ".txt";

  //debug

  Serial.println(gemiddeldeLuchtdruk);
  Serial.println(gemiddeldeTemperatuur);
  mySerial.println(gemiddeldeLuchtdruk);
  mySerial.println(gemiddeldeTemperatuur);
  Serial.println(filename);
  mySerial.println(filename);


  if (!bmp.begin(BMP_address)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  cli();
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 16074;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();
}





//0.97 hz
ISR(TIMER1_COMPA_vect) {
  while (ss.available() > 0)
    if (gps.encode(ss.read())) {
      if (gps.location.isValid())
      {
        gpsLat = String((gps.location.lat(), 6));
        gpsLon = String((gps.location.lng(), 6));
      }
      else
      {
        gpsLat = "0";
        gpsLon = "0";
      }
    }
  mySerial.println(dataString);
}








void loop() {


  String luchtdruk = String(round(bmp.readPressure()));
  String temperatuur = String(bmp.readTemperature());
  float luchtdruk2 = luchtdruk.toFloat();

  //hoogte berekenen
  float drukverschil = luchtdruk2 / gemiddeldeLuchtdruk;
  float temperatuurOpHoogte = gemiddeldeTemperatuur * (pow(drukverschil, constante));
  String hoogte = String((temperatuurOpHoogte - gemiddeldeTemperatuur) / lapseRate);

  dataString = gpsLat + " " + gpsLon + " " + hoogte + " " + temperatuur + " " + luchtdruk;

  File dataFile = SD.open(filename, FILE_WRITE);
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
  }

  Serial.println(dataString);


  delay(10);
}
