#include <Wire.h>
#include <SD.h>
#include <RTClib.h>

const int chipSelect = 4; // Pin de sélection de la carte SD
const int wattmeterAddress = 0x40; // Adresse I2C du module Wattmètre
const int registerPower = 0x00; // Registre de lecture de la puissance

RTC_DS3231 rtc; // Objet RTC

void setup() {
  Wire.begin(); // Initialisation de la communication I2C
  rtc.begin(); // Initialisation du module RTC
  Serial.begin(9600); // Initialisation de la communication série (pour le débogage)
  while (!Serial) {
    ; // Attendre que la communication série soit initialisée
  }
  if (!SD.begin(chipSelect)) { // Initialisation de la carte SD
    Serial.println("Erreur d'initialisation de la carte SD");
    return;
  }
  Serial.println("Initialisation réussie");
}

void loop() {
  // Lire la valeur de puissance du module Wattmètre
  Wire.beginTransmission(wattmeterAddress);
  Wire.write(registerPower);
  Wire.endTransmission();
  Wire.requestFrom(wattmeterAddress, 2);
  int power = Wire.read() << 8 | Wire.read();
  Serial.print("Puissance : ");
  Serial.println(power);

  // Obtenir l'heure courante à partir du module RTC
  DateTime now = rtc.now();
  int year = now.year();
  int month = now.month();
  int day = now.day();
  int hour = now.hour();
  int minute = now.minute();
  int second = now.second();

  // Ouvrir un fichier en écriture sur la carte SD
  File dataFile = SD.open("data.txt", FILE_WRITE);
  if (dataFile) {
    // Écrire la valeur de puissance et l'heure courante dans le fichier
    dataFile.print(year);
    dataFile.print("/");
    dataFile.print(month);
    dataFile.print("/");
    dataFile.print(day);
    dataFile.print(" ");
    dataFile.print(hour);
    dataFile.print(":");
    dataFile.print(minute);
    dataFile.print(":");
    dataFile.print(second);
    dataFile.print(",");
    dataFile.println(power);
    dataFile.close(); // Fermer le fichier
  } else {
    Serial.println("Erreur d'ouverture du fichier");
  }

  delay(1000); // Attendre 1 seconde avant de répéter la mesure
}
