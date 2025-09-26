//Libraries:
//Adafruit SGP30 Sensor
//BME280 by Tyler Glenn
//EspSoftwareSerial
//LoRa by Sandeep Mistry
//sensirion-sps by Johannes Winkelmann
//TinyGPSPlus by Mikal Hart

//LED Pin 33
//Spannungsteiler Pin 35
//Piezo Pin 25
//Data
float loraData[6][4];
int loraAcc[6][4] = {
  {6, 6, 2, 2},
  {4, 1, 1, -1},
  {4, 4, 4, 4},
  {2, 2, -1, -1},
  {2, 2, 2, -1},
  {1, -1, -1, -1},
};

//MPU6050
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

//SD
//#include <SPI.h>
//#include <SD.h>
//#include <FS.h>
#include <SdFat.h>

const uint8_t SD_CS_PIN = 13;
const uint8_t SOFT_MISO_PIN = 27;
const uint8_t SOFT_MOSI_PIN = 14;
const uint8_t SOFT_SCK_PIN = 12;

SoftSpiDriver<SOFT_MISO_PIN, SOFT_MOSI_PIN, SOFT_SCK_PIN> softSpi;
//#if ENABLE_DEDICATED_SPI
//#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(0), &softSpi)
//#else  // ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(0), &softSpi)
//#endif  // ENABLE_DEDICATED_SPI

SdFat32 sd;
File32 file;
int writeCount = 0;
int fileCount = 0;
//CS 32
//SCK 18
//MOSI 23
//MISO 19
//VCC 5V

//SPS30
#include <sps30.h>
int sps30_FAIL = 0;
bool SPS30_AVAIL;
//Schwarz 5V
//Rot 21
//Weis 22
//Gelb GND
//Blau GND

//SGP30
#include "Adafruit_SGP30.h"
Adafruit_SGP30 sgp;
bool SGP30_AVAIL;
//SDA 21
//SCL 22

//LoRa
//1 byte / char braucht 1,5 ms
//1 Packet bracht 25 ms + Zeit pro char
//#include <SPI.h>
#include <LoRa.h>
#define LoRa_CS 32
#define LoRa_RST 26
#define LoRa_INT 34
bool LoRa_AVAIL; //False if LoRa failed to start
char *data[5];
//RST 26
//CS 25
//MOSI 23
//MISO 19
//SCK 18
//Go 34
                 
//GPS
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#define GPS_RXPin 17
#define GPS_TXPin 16
#define GPS_Baud 9600
//#define GPS_TIME 800 //Maximale GPS auslese Zeit
TinyGPSPlus gps;
SoftwareSerial ss(GPS_RXPin, GPS_TXPin);
//RX 16
//TX 17

//BME280
#include <BME280I2C.h>
#include <Wire.h>
BME280I2C::Settings settings(
    BME280::OSR_X1,              // Oversampling für Temperatur
    BME280::OSR_X1,              // Oversampling für Druck
    BME280::OSR_X1,              // Oversampling für Feuchtigkeit
    BME280::Mode_Forced,         // Betriebsmodus
    BME280::StandbyTime_1000ms,  // Standby-Zeit
    BME280::Filter_Off,          // Filter deaktiviert
    BME280::SpiEnable_False,     // SPI deaktiviert
    BME280I2C::I2CAddr_0x76      // I2C-Adresse
);
BME280I2C bme(settings);
int BME280_AVAIL;
float lowPres;
//SDA 21
//SCL 22

//Piezo
//pin 25
#include <EnvironmentCalculations.h>
float maxHeight;

//Watchdogtimer
#include <esp_task_wdt.h>



void sendError(int times){
  unsigned long start = millis();
  while(millis() -start < 60000){
    for(int i = 0; i < times; i++){
      digitalWrite(25, HIGH);
      delay(333);
      digitalWrite(25, LOW);
      delay(667);
    }
    delay(2000);
  }
}


float height()
{
  return EnvironmentCalculations::Altitude(bme.pres(), EnvironmentCalculations::AltitudeUnit_Meters, 1013, bme.temp(), EnvironmentCalculations::TempUnit_Celsius);
}


void initializePiezo() {
  Serial.println("Initialisiere Piezo...");
  pinMode(25, OUTPUT);
  digitalWrite(25, LOW);
  Serial.println("done");
}
void initializeMPU6050() {
  Serial.println("Initialisiere MPU6050...");
  if (!mpu.begin()) {
    sendError(2);
    Serial.println("MPU6050 konnte nicht gefunden werden. Überprüfen Sie die Verkabelung!");
  }
  else{
    Serial.println("MPU6050 gefunden");
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("done");
}

void initializeSD(){
  /*
  pinMode(SD_CS, OUTPUT);
  */
  Serial.println("Initialisiere SD-Karte...");
  if (!sd.begin(SD_CONFIG)) {
    sendError(3);
    sd.initErrorHalt(&Serial);
    //Serial.println("Error 1");

    Serial.println("SD failed");
  } else {
      if(!file.open("Data.txt", O_WRITE | O_CREAT | O_AT_END)) {
        sd.printSdError(&Serial);
        sendError(3);
      }
      file.close();
  }

  Serial.println("done");
  
}

void initializeSGP30(){
  Serial.println("Initialisiere SGP30...");
  if(!sgp.begin()){
    sendError(4);
  }
  Serial.println("done");
}

void initializeGPS(){
  Serial.println("Initialisiere GPS...");
  ss.begin(9600);
  Serial.println("done");
}

void initializeLoRa(){
  Serial.println("Initialisiere LoRa...");
  //LoRa.setSPI(SPI2);
  LoRa.setPins(LoRa_CS, LoRa_RST, LoRa_INT);
  LoRa.setTxPower(20);
  LoRa.setSyncWord(0x10);
  LoRa_AVAIL = LoRa.begin(868E6);
  if(!LoRa_AVAIL){
    sendError(6);
    Serial.println("LoRa failed");
  }
  Serial.println("done");
}

void initializeSPS30(){
  Serial.println("Initialisiere SPS30...");
  sensirion_i2c_init();
  if (sps30_start_measurement() < 0){
    sendError(7);
    SPS30_AVAIL = false;
  } else {
    SPS30_AVAIL = true;
  }
  Serial.println("done");
}

void initializeBME280(){
    Serial.println("Initialisiere BME280...");
    BME280_AVAIL = bme.begin();

    bme.setSettings(settings);

    if(BME280_AVAIL) {
      //lowPres = bme.pres();
      maxHeight = height();
      //Serial.println(maxHeight);
      //Serial.println(height());
    } else {
      sendError(8);
    }
    Serial.println("done");
}

void send(int sensor, int type, float data, int acc0){
  unsigned long timestamp = millis()/100;
  Serial.print(sensor);Serial.print(type);Serial.print(":");Serial.println(data, acc0);
  
  //Serial.print(timestamp); Serial.print(","); Serial.print(sensor); Serial.print(" , "); Serial.print(type); Serial.print(" : "); Serial.println(data, acc0);
  //LoRa.beginPacket();
  loraData[sensor][type] = data; 
  //LoRa.endPacket();
  // create file if it does not exist
  String name = "Data" + String(fileCount) + ".txt";
  char name_char[20];
  name.toCharArray(name_char, 20);
  //Serial.println(name_char);
  if(file.open(name_char, O_WRITE | O_CREAT | O_AT_END)) {
    file.print(timestamp); file.print(F(",")); file.print(sensor); file.print(F(",")); file.print(type); file.print(F(":")); file.println(data, acc0); 
  } else {
    sd.printSdError(&Serial);
    //sd.end();
    //sd.begin(SD_CONFIG);
    fileCount++;
    Serial.println("Failed to open the file");
  }
  file.close();
}



void sendGPS() {
  unsigned long s = millis();
  while (ss.available() > 0 && (s+100) > millis()) {
    gps.encode(ss.read());
  }
  if (gps.location.isUpdated() || gps.speed.isUpdated() || gps.altitude.isUpdated() || gps.satellites.isUpdated()) {
        send(0, 0, gps.location.lat(), 8);
        send(0, 1, gps.location.lng(), 8);
        send(0, 2, gps.speed.mps(), 4);
        send(0, 3, gps.altitude.meters(), 4);
        //send(0, 4, gps.satellites.value(), 0);
  } 
  // Print the connection quality (HDOP) to the Serial monitor
  //if (gps.hdop.isUpdated()) {
  //  Serial.print("HDOP (Horizontal Dilution of Precision): ");
  //  Serial.println(gps.hdop.value() / 100.0, 2);
  //}
  // Print the GPS time to the Serial monitor
  //if (gps.time.isUpdated()) {
  //  Serial.print("GPS Time: ");
  //  Serial.print(gps.time.hour()); Serial.print(":");
  //  Serial.print(gps.time.minute()); Serial.print(":");
  //  Serial.println(gps.time.second());
  //}
}


void sendBME(){
  if(!BME280_AVAIL){
    BME280_AVAIL = bme.begin();
    Serial.println("BME280 failed");
    //lowPres = bme.pres(); 
    maxHeight = height();
  }

  send(1, 0, bme.pres(), 8);
  send(1, 1, bme.temp(), 4);
  send(1, 2, bme.hum(), 2);
}

void sendSPS(){
  if (!SPS30_AVAIL) {
    if (sps30_start_measurement() < 0){
      SPS30_AVAIL = false;
    } else {
      SPS30_AVAIL = true;
    }
    Serial.println("SPS30 failed");
  } 

  if (sps30_FAIL > 5){
    sensirion_i2c_init();
    if (sps30_start_measurement() < 0){
      SPS30_AVAIL = false;
    } else {
      SPS30_AVAIL = true;
    }
    Serial.println("SPS30 failed");
  }

  uint16_t data_ready;
  struct sps30_measurement m;

  if(sps30_probe() == 0 && sps30_read_data_ready(&data_ready) >= 0 && data_ready && sps30_read_measurement(&m) >= 0){
    send(2, 0, m.mc_1p0, 8);
    send(2, 1, m.mc_2p5, 8);
    send(2, 2, m.mc_4p0, 8);
    send(2, 3, m.mc_10p0, 8);

    sps30_FAIL = 0;
  } else {
    sps30_FAIL++;
  }
}
uint32_t getAbsoluteHumidity(float temperature, float humidity) {
  // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
  const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
  const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
  return absoluteHumidityScaled;
}
void sendSGP(){
  //if (!SGP30_AVAIL) {
  //    SGP30_AVAIL = sgp.begin();
  //    Serial.println("SGP30 failed");
  //}
  sgp.setHumidity(getAbsoluteHumidity(loraData[1][1], loraData[1][2]));

  if (! sgp.IAQmeasure()) {
    Serial.println("Measurement failed");
    return;
  }


//SD
  send(3, 0, sgp.TVOC, 2);
  send(3, 1, sgp.eCO2, 2);

  uint16_t TVOC_base, eCO2_base;
  if (! sgp.getIAQBaseline(&eCO2_base, &TVOC_base)) {
    Serial.println("Failed to get baseline readings");
    return;
  }

  Serial.print("****Baseline values: eCO2: 0x"); Serial.print(eCO2_base, HEX);
  Serial.print(" & TVOC: 0x"); Serial.println(TVOC_base, HEX);
  //send(3, 0, sgp.rawH2, 2);
  //send(3, 1, sgp.rawEthanol, 2);
}

void sendMPU6050(){
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  send(4, 0, accel.acceleration.x, 4);
  send(4, 1, accel.acceleration.y, 4);
  send(4, 2, accel.acceleration.z, 4);
  //send(4, 3, gyro.gyro.x, 4);
  //send(4, 4, gyro.gyro.y, 4);
  //send(4, 5, gyro.gyro.z, 4);
}

void runPiezo(){
  if(maxHeight > 3000){
    maxHeight = height();
  }
  if(height() > maxHeight) {
    maxHeight = height();
    digitalWrite(25, LOW);
  } else if (height()+10 < maxHeight) {
    digitalWrite(25, !digitalRead(25));
  }
}

void runLED(){
  digitalWrite(33, !digitalRead(33));
}

void sendBattery(){
  float value = analogRead(35);
  float spannung = value*0.00164456+0.2597;
  send(5, 0, spannung, 1);
}

void sendData( void * pvParameters ) {
  for(;;){
    for(int i = 0; i < 6; i++){
      LoRa.beginPacket();
      for(int j = 0; j < 4; j++){
        float data = loraData[i][j];
        int acc = loraAcc[i][j];
        if(acc != -1){
          LoRa.print(i); LoRa.print(j); LoRa.print(":"); LoRa.println(data, acc); 
        }
      }
      LoRa.endPacket();
    }
    delay(100);
  }
}


void setup(){
  //pinMod
  Serial.begin(115200);
  Wire.begin();
  delay(100);

  //initializeWDT();
  initializePiezo();
  initializeGPS();
  initializeSD();
  initializeLoRa();
  initializeBME280();
  initializeMPU6050();
  initializeSPS30();
  initializeSGP30();
  
  pinMode(33, OUTPUT);
  Serial.println("Finished Startup");

  xTaskCreatePinnedToCore(
                      sendData,   /* Task function. */
                      "sendData",     /* name of task. */
                      10000,       /* Stack size of task */
                      NULL,        /* parameter of the task */
                      10,           /* priority of the task */
                      NULL,      /* Task handle to keep track of created task */
                      0);          /* pin task to core 0 */                  
  
}



void loop(){ 
  //if(!LoRa_AVAIL){
    //LoRa.end();
    //LoRa_AVAIL = LoRa.begin(868E6);
  //}
  sendBME();
  runPiezo();
  runLED();
  sendSPS();
  sendSGP();
  sendMPU6050();
  sendBattery();
  sendGPS();
}