/*
  Title : CS2_All_sensors
  Version : 1.0
  Project : ClearSky 2.0
  Date : 13/03/2019
  DEMS authors : Régis and Dorian

  library MQ-135
  https://github.com/GeorgK/MQ135

  softwareSerial
  https://github.com/PaulStoffregen/SoftwareSerial

  see wiring on the Fritzing shem on the GitHub :
  https://github.com/ClearSkyPollutions/Sensor_Drivers/blob/master/OLD%20AND%20TRY/DSM501/DSM501.ino
*/

/*****************************************************************************************/
/* Libriries include *********************************************************************/
/*****************************************************************************************/

#include <MQ135.h>
#include <Wire.h>
#include <SPI.h>                // I2C library supports 
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <ArduinoLowPower.h>
//#include "SoftwareSerial.h"

/*****************************************************************************************/
/* Global constants **********************************************************************/
/*****************************************************************************************/

//#define NO_DEBUG_CS  //--> uncomment this line to disable the debug mode

//#define PIN_SLEEP_MODE 3
//#define PIN_RESET 4
#define pmsSerial Serial1
#define PMS_SLEEP 5
// Declaration and initialization of the input pin
const int Analog_in = A1; // X-axis-signal
const int mq135Pin = A5;     // Pin sur lequel est branché de MQ135
const int Digital_in = 0; // Button
const int AlimNormal = 1;
const int AlimPrech = 2;


/*****************************************************************************************/
/* Global constants **********************************************************************/
/*****************************************************************************************/

//SoftwareSerial pmsSerial(6, 7);

//Setup connection of the sensor
Adafruit_BMP280 bmp; // I2C


//Variables
MQ135 gasSensor = MQ135(mq135Pin);  // Initialise l'objet MQ135 sur le Pin spécifié

void setup()
{
  pinMode(AlimPrech, OUTPUT);
  pinMode(AlimNormal, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);


  Serial.println("Démarrage du Setup...");
  Serial.begin(9600);     // Initialise le port série à 9600 bps
  pmsSerial.begin(9600);
  pinMode (Analog_in, INPUT);
  pinMode (Digital_in, INPUT);
  //bmp.begin();                           //To be deleted
  while (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    delay(3000);
  }
  Serial.println("bmp ok");
  LowPower.attachInterruptWakeup(RTC_ALARM_WAKEUP, dummy, CHANGE);
}

struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};

struct pms5003data data;



/**************************************************************************************************/
/* BOUCLE D'EXECUTION PRINCIPALE ******************************************************************/
/**************************************************************************************************/
void loop() {
  float gazRzero;
  float gazPpm;
  float gazAzero;
  float micAnalog;              //Analog signal from microphone
  int micDigital;               //Digital value from microphone
  float barPressure;            //Barometric pressure (Pa)
  float barTemperature;         //Temperature (°C)
  int barAltimeter;             //Altimeter (m)
  bool pmsOK;
  char MyMessage;

  
  digitalWrite(AlimPrech, HIGH);
  digitalWrite(AlimNormal, HIGH);
  /*Données du capteur de gaz*/
  readGaz(&gazRzero, &gazPpm, &gazAzero);       // Lecture des données du capteur de gaz

  delay(500);                                   // Actualise la mesure chaque seconde
  /*Données du capteur de particule*/
  pmsOK = readPMSdata(&pmsSerial);
  Serial.println(pmsOK);
  /*Données du capteur de son*/
  readSound(&micAnalog, &micDigital);

  //delay (200);
  /*Données du barometre*/
  readBarometer(&barPressure, &barTemperature, &barAltimeter);
  //barAltimeter
  MyMessage = (char)barAltimeter;
  Serial.println(MyMessage);
  
  delay(100);
  #ifdef NO_DEBUG_CS
      Serial.println("no debug");
  #else
    sendBarometer(barPressure, barTemperature, barAltimeter);
    sendSound(micAnalog, micDigital);
    sendGaz(gazRzero, gazPpm, gazAzero);          // Envoie de ces données
    if (pmsOK) {                                  // Si la lecture des données du capteur de particule fonctionne
      sendPMSdata();                              // On envoie ces données
    }
  #endif  

  

  audodo();       //mets l'arduino en veille
  delay(5000);  //Update every 5 sec

}

/*void tramSigFox() {
  Serial.println(barPressure);
  Serial.println(barTemperature);
  Serial.println(micAnalog);
  Serial.println(gazPpm);
  Serial.println(micAnalog);
}*/              

void audodo() {
  int i;

  digitalWrite(AlimPrech, false);
  digitalWrite(AlimNormal, false);
  digitalWrite(LED_BUILTIN, LOW);

  dormir(40);                     //temps de sommeil à calculé en fonction de la fréquence de mesure
  digitalWrite(AlimPrech, true);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(600000);
  digitalWrite(AlimNormal, true);
}

void dormir(float TsleepMin) {
  int TsleepMs;
  int T5minMS;
  int T10sMS;

  TsleepMs = TsleepMin * 60 * 1000;
  T5minMS = 60 * 1000 * 5;
  T10sMS = 10 * 1000;

  while (TsleepMs > T5minMS) {          //tant que Tsleep > 5min
    LowPower.sleep(T5minMS - T10sMS);    //dormir 4min50s
    delay(T10sMS);
    TsleepMs = TsleepMs - T5minMS;
  }
  LowPower.sleep(TsleepMs);
}

void dummy() {
  //alarm_source = 0;
}

/**************************************************************************************************/
/* Lecture des données du capteurs de Gaz *********************************************************/
/**************************************************************************************************/

void readGaz(float* rzero, float* ppm, float* azero) {
  *rzero = gasSensor.getRZero();
  *ppm = gasSensor.getPPM();
  *azero = analogRead(mq135Pin);
  return;
}

/**************************************************************************************************/
/* Envoie des données du capteurs de Gaz **********************************************************/
/**************************************************************************************************/

void sendGaz(float rzero, float ppm, float azero) {
  Serial.println("Données gaz");

  Serial.print("R0: ");
  Serial.println(rzero);

  Serial.print("A0: ");
  Serial.print(azero);

  Serial.print(" ppm CO2: ");
  Serial.println(ppm);
}

/**************************************************************************************************/
/* Lecture des données du capteurs de particule PMS ***********************************************/
/**************************************************************************************************/

boolean readPMSdata(Stream *s) {
  uint8_t buffer[32];
  uint8_t i;
  uint16_t sum = 0;
  uint16_t buffer_u16[15];

  if (! s->available()) {
    return false;
  }

  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }

  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }
  s->readBytes(buffer, 32);

  // get checksum ready
  for (i = 0; i < 30; i++) {
    sum += buffer[i];
  }

  /* debugging
    for (uint8_t i=2; i<32; i++) {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
    }
    Serial.println();
  */

  // The data comes in endian'd, this solves it so it works on all platforms

  for (i = 0; i < 15; i++) {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  }

  // put it into a nice struct :)
  memcpy((void *)&data, (void *)buffer_u16, 30);

  if (sum != data.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
}

/**************************************************************************************************/
/* Envoie des données du capteurs de particule PMS ************************************************/
/**************************************************************************************************/

void sendPMSdata() {
  Serial.println ("----------------------------------------------------------");
  Serial.println("Données Particule");
  Serial.println();
  Serial.println("Concentration Units in μg/m3(standard)");
  Serial.print("PM 1.0: "); Serial.print(data.pm10_standard);
  Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_standard);
  Serial.print("\t\tPM 10: "); Serial.println(data.pm100_standard);
  Serial.println("-------");
  Serial.println("Concentration Units in μg/m3(environmental)");
  Serial.print("PM 1.0: "); Serial.print(data.pm10_env);
  Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_env);
  Serial.print("\t\tPM 10: "); Serial.println(data.pm100_env);
  Serial.println("------");
  Serial.print("Particles > 0.3um / 0.1L air:"); Serial.println(data.particles_03um);
  Serial.print("Particles > 0.5um / 0.1L air:"); Serial.println(data.particles_05um);
  Serial.print("Particles > 1.0um / 0.1L air:"); Serial.println(data.particles_10um);
  Serial.print("Particles > 2.5um / 0.1L air:"); Serial.println(data.particles_25um);
  Serial.print("Particles > 5.0um / 0.1L air:"); Serial.println(data.particles_50um);
  Serial.print("Particles > 10.0 um / 0.1L air:"); Serial.println(data.particles_100um);
  Serial.println ("----------------------------------------------------------");
}

/**************************************************************************************************/
/* Lecture des données du capteurs de son *********************************************************/
/**************************************************************************************************/

void readSound(float *analog, int* digit) {
  // Current value will be read and converted to voltage
  *analog = analogRead (Analog_in) * (3.3 / 1023.0);
  *digit = digitalRead (Digital_in);
  return;
}

/**************************************************************************************************/
/* Envoie des données du capteurs de son **********************************************************/
/**************************************************************************************************/

void sendSound(float Analog, int Digital) {

  Serial.println("Données microphone");
  Serial.print ("Analog voltage value: "); //Print the Analog value
  Serial.print (Analog, 4);
  Serial.print ("V, ");

  //Test the Digital pin
  Serial.print ("Extreme value: ");
  if (Digital == 1)  {
    Serial.println (" reached");
  }
  else  {
    Serial.println (" not reached yet");
  }
  Serial.println ("----------------------------------------------------------");

}

/**************************************************************************************************/
/* Lecture des données du baromètre ***************************************************************/
/**************************************************************************************************/

void readBarometer(float *pressure, float *temperature, int *altimeter) {
  *pressure = bmp.readPressure();
  *temperature = bmp.readTemperature();
  *altimeter = bmp.readAltitude (1013.25);
  return;
}

/**************************************************************************************************/
/* Envoie des données du baromètre ***************************************************************/
/**************************************************************************************************/

void sendBarometer(float pressure, float temperature, int altimeter) {
  Serial.println("Adafruit BMP280 test:");

  Serial.print(F("Pressure: "));
  Serial.print(pressure);
  Serial.print(" Pa =\t");
  Serial.print(pressure / 100);
  Serial.print(" hPa\tTemp: ");
  Serial.print(temperature);
  Serial.print("°C\tAltimètre:");
  Serial.print(altimeter); // this should be adjusted to your local forcase
  Serial.println("m\n----------------------------------------------------------");
}
