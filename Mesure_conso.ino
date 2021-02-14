// Mesure de consommation en Watt à l'aide d'un transformateur de courant
// Hardware configuration :
// - carte ARDUINO mega
// - transfo de courant SCT013 30A
// - afficheur LCD 2 lignes GDM1602X
// - shield Airlift Wifi Adafruit
//------------------------------------------------------------------------------------------------------------
// Auteur : Patrick Souty
// Date de création initiale : 31/01/2020
// Version 1.0 du 01/02/2021 - version fonctionnelle - seule limitation : résolution du CAN
//                             qui entraine un euil de mesure de l'ordre de 20 W (bruit de quantification)
// Version 2.0 du 02/02/2021 - ajout fonction serveur Web et bibliothèque Time
// Version 3.0 du 02/02/2021 - autocalibration 5V par mesure initiale d'une diode bandgap interne
// Version 4.0 du 03/02/2021 - mesure de courant en mode différentiel de l'ADC et avec gain de 10
// Version 5.0 du 05/12/2021 - gestion automatique du gain ADC en fonction des mesures effectuées
//                             ajout info sur temps de traitement
// Version 6.0 du 10/02/2021 - ajout stockage sur carte SD (en cours)
//                           - calcul de la consommation cumulée
// Versio 7.0 du 11/02/2021  - stockage mesures sur carte SD
// Version 7.1 du  13/02/2021 - extinction rétroéclairage LCD après 1er stockage sur carte SD
// Prochaines évolutions :      # consultation à distance fichiers carte SD
//                              # filtrage des mesures à faible courant
//                              # meilleure gestion gain auto en faisant sytématiquement un paquet sur 2 à gain min et en comparant les mesures
//                              # ou commande par boutons poussoirs 
//                              # commutation retro éclairage LCD
//                              # communication en bluetooth avec appli smartphone
//
//--------------------------------------------------------------------------------------------------------------------------------
// Définitions bibliotèques
#include <Wire.h>
#include <TimeLib.h>
#include <LiquidCrystal_I2C.h>      // bibliothèque pour afficheur LCD
LiquidCrystal_I2C lcd(0x27, 16, 2); // instanciation de l'afficheur LCD
#include <SPI.h>                    // bibliothèque pour la liaison SPI avec afficheur LCD
#include "SdFat.h"                     // bibliothèque pour la gestion de la carte SD
#include <WiFiNINA.h>               // bibliotèque pour module WiFi Airlift
#include <WiFiUdp.h>                // bibliothèque pour gestion protocole Udp
#include "arduino_secrets.h"
#pragma execution_character_set("latin-1")
String Date(unsigned long epoch, boolean Reverse = false);
#define Reversed  true
//
//--------------------------------------------------------------------------------------------------------------
// Paramètres et définitions de variables
enum GainSelect {STD, MIDDLE, MAX};         // énumération pour le choix de la valeur du gain ADC
const long ADCRange = 1024;                 // valeur numérique codée à pleine échelle par l'ADC + 1 LSB
const float Mega = 1.e6;
boolean Debug = true;
int CursorLine = 0;         // définit la ligne courante d'impression pour l'écran LCD
int Conso;   // consommation instantanée en Watt
int j;
int Count;   // variable générale dédiée aux comptages de boucles
unsigned long Time1970 = 0;   // variable mémorisant le temps fourni iniialement par le serveur NTP sous la forme de secondes depuis le 01/01/1970
unsigned long Time2020 = 0;   // variable mémorisant le temps fourni iniialement par le serveur NTP sous la forme de secondes depuis le 01/01/2020
unsigned long Time, T0, T1, T2, MeasurementTime, MeasurementDate, LastTime = 0, TimeStamp, StartTime;
unsigned long ProcessingTime;             // Pour la mémorisation du temps de traitement de l'interruption de sampling
unsigned long epoch;                      // utilisé pour le temps UTC
unsigned long DeltaTime;                  // utilisé pour la mesure de la période moyenne d'échnatillonnage                         
String MyString;                          // variable générique pour manipuler les string
unsigned long Somme2Sensor = 0;           // pour somme du carré des mesures senseur
long SommeSensor = 0;                     // pour somme des mesures senseur
unsigned long Somme2SensoraTraiter = 0;   // pour sauvegarde somme du carré des mesures senseur
long SommeSensoraTraiter = 0;             // pour sauvegarde somme des mesures senseur
int TrueCount =0;                         // pour détermination du nombre effectif d'échantillons traités dans un  paquet
int TrueCountaTraiter = 0;                // pour sauvegarde du nombre effectif d'échantillons traités

long Max = -512, Min = 511;               // utilisées pour le monitoring de valeurs min et max senseur
long MaxaTraiter, MinaTraiter;            // utilisées pour le monitoring de valeurs min et max senseur
boolean DataReady = false;                // sémaphore pour signaler données prêtes
const int SamplingFreq = 410;             // fréquence d'échantillonnage en Hz
const int Timer_Value = 256 - int(float(62500. / SamplingFreq) +0.5 ); // valeur à programmer dans le timer 2
                                                                // par exemple 300 Hz => 256 - 208 (208 x 16 µS = 3.328 ms soit 300.48 Hz)
                                                                // d'où Fmin = 245 Hz
const byte SamplingCount = 200;           // nombre d'échantillons pour calcul du courant efficace (le nombre réel est augmenté de 2)
const int DatalogInterval = 10 * 1000;    // périodicité stockage data en milli secondes
int PacketCount = 0;                      // comptage du nombre de paquets entre 2 log de données
const String Delimiter = ";";               // delimiter pour les ficheirs csv
float MeanPower =0;                       // Puissance moyenne calculée sur l'intervale DatalogInterval
unsigned long CumulConso =0;              // pour intégrer la consommation cumulée en W.s
unsigned long LastLog;                    // mémorise l'instant du dernier enregistrement sur carte DS
int LogCount = 0;                         // comptage nombre d'enrelistrements sur carte SD
const int MaxLogCount = 3;                // nombre de log avant de faire un vidage buffer carte SD
byte varCompteur = 0;                     // La variable compteur d'échantillons
const float SensorScaleFactor = 30. / 1.; // 30A efficace pour 1V efficace en sortie - en Volt / A
float ScaleFactorADC;                     // facteur d'échelle calibré de l'ADC en V/LSB (valeur typique = 0.004883)
float ScaleFactorADCdiff;                 // facteur d'échelle ADC en mode différentiel = 2 * facteur d'échelle en single
enum GainSelect ADCGainOption = STD;      // sélectionne le gain ADC, initialisé à gain min au démarrage
float LastPower = 10000;                  // Mémorisation dernière puissance mesurée, sert à la gestion automatique du gain
String DisplayGain = "L";                 // Permet d'afficher sur l'écran LCD le gain courant L, M ou H
float Seuil1 = 15., Seuil2 = 500.;        // seuil en Watt des options de gain 
boolean GainRequestUp = false;            // sert à la gestion automatique du gain
boolean GainRequestDown = false;          // sert à la gestion automatique du gain
uint8_t low, high;                        // utilisé pour la mesure différentielle avec l'ADC
float ADCGain;                            // Gain interne ADC : 1, 10 ou 200 selon configuration mux
float PacketGain;                         // mémorise le gain utilisé pour le dernier paquet à traiter
float VARef = 5.0;                        // Tension de référence du CAN ré-estimé ensuite par autocalibration avec diode bandgap
unsigned long VrefTypValue = long(3.45 / VARef * 1024.);  // valeur typique numérisation 3.3 V - utilisée pour le calcul de l'écart type
const float MainVoltage = 220;            // Tension moyenne du secteur en volt efficaces
boolean Breakpoint = true;                // Utilisé pour du debug pour permettre une action d'impression sur breakpoint
boolean EtatLED = false;                  // sert à gérer le clignotement d'une LED en fonctionnement
//
//--------------------------------------------------------------------------------------
// Définition LED
const int GreenLED1 = 2;
const int RedLED =3;
const int GreenLED2 = 8;
//--------------------------------------------------------------------------------------
// Definitions for WiFi Airlift board
#define SPIWIFI       SPI
#define SPIWIFI_SS    10   // Chip select pin
#define SPIWIFI_ACK    7   // a.k.a BUSY or READY pin
#define ESP32_RESETN   5   // Reset pin
#define ESP32_GPIO0   -1   // Not connected
//--------------------------------------------------------------------------------------
// Definitions for WiFi Airlift board - SD card reader
// set up variables using the SD utility library functions:

SdFat SD;
const int chipSelect = 4;
boolean CheckSDCard();
// Error messages stored in flash.
#define error(msg) SD.errorHalt(F(msg))
File myFile;
//
//---------------------------------------------------------------------------------------
// Definitions for analog inputs
#define CurrentSensorIn A0    // entrée analogique transfo de courant
#define RefVoltageIn A1       // entrée analogique pour mesure tension 3V3
//
//---------------------------------------------------------------------------------------
// Initialisations pour comm. UDP en WiFi
int status = WL_IDLE_STATUS;

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)

int keyIndex = 0;            // your network key Index number (needed only for WEP)

unsigned int localPort = 2390;      // local port to listen for UDP packets

IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server

const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;
//
//----------------------------------------------------------------------------------------------
// Définitions pour serveur Web
int serverPort = 80;
WiFiServer server(serverPort);
//
//----------------------------------------------------------------------------------------------
// Variables globales

//
//----------------------------------------------------------------------------------------------
//
void setup() {
  // init sorties LED
  pinMode(GreenLED1, OUTPUT);
  pinMode(RedLED, OUTPUT);
  pinMode(GreenLED2, OUTPUT);
  digitalWrite(GreenLED1,LOW);   // éteint la led
  digitalWrite(GreenLED2,LOW);   // éteint la led
  digitalWrite(RedLED,HIGH);   // allume la LED pour signaler la mise sous tension
  //
  ScaleFactorADC = InitScaleFactorADC();         // première mesure uniquement pour activer la diode bandgap et le mux ADC
  // init canal d'impression standard (pour le debug)
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  Count = 0;
  while (!Serial && (Count < 10)) {
    delay(100); // wait for serial port to connect. Needed for native USB port only. With timeout 1s max
    Count++;
  }
  Serial.println();
  Serial.println(F("----------------------------------------------------------------------------"));
  Serial.println(F("Power Meter Initialisation"));
  Serial.print(F("Timer_Value : "));
  Serial.println(Timer_Value);
  // initialise afficheur LCD
  lcd.init();
  lcd.clear();
  lcd.noAutoscroll();
  lcd.setCursor(0, 0);
  lcd.backlight();

  PrintLCD(F("Power meter init"), 0);
  PrintLCD(F("Try to connect ."), 1);

  //------------------------------------------------------------------------------
  // initialisation module WiFi et communication WiFi
  WiFi.setPins(SPIWIFI_SS, SPIWIFI_ACK, ESP32_RESETN, ESP32_GPIO0, &SPIWIFI);
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
  PrintLCD(F("WiFi module down!"), 1);
    Serial.println(F("No communication with Airlift WiFi module !"));
    // don't continue
    while (true);   {  // bloque l'exécution sur cette ligne
      BlinkLED(RedLED);
    }
  }
  String fv = WiFi.firmwareVersion();
  if (fv < "1.0.0") {
  PrintLCD("Upgrade firmware !", 1);
    Serial.println(F("Please upgrade WiFi module firmware"));
  }

  // attempt to connect to Wifi network:
  Serial.print("Try to connect to SSID: ");
  Serial.println(ssid);
  PrintLCD("Try to connect ..", 1);
  while (status != WL_CONNECTED) {    
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    BlinkLED(GreenLED1);
  }
  digitalWrite(GreenLED1,HIGH);
  PrintLCD("WiFi Com OK", 0);
  Serial.print(F("Connected to WiFi"));
  printWifiStatus();
  PrintLCD("Try Com NTP ..", 1);
  Serial.println(F("\nStarting connection to server..."));
  Udp.begin(localPort);

  //------------------------------------
  // synchro initiale du temps
  Count = 0;
  while ((Count < 10) && (Time == 0)) {
    Time = GetTime();
    T0 = millis();    // mémorise le temps Arduino correspondant au temps NTP
    Count++;
    delay(1000);
  }
  if (Time == 0) {
    PrintLCD("Time Synchro KO !", 0);
    PrintLCD("Let's go on ...", 1);
    Time = 3786825600UL;
  }
  else {
    PrintLCD("Time Synchro OK !", 0);
    PrintLCD("Let's go on ...", 1);
    
  }
  Time2020 = PandemicTime(Time);
  Time1970 = UnixTime(Time);
  setTime(Time1970); // Sync Arduino clock to the time received on the NTP server
  Serial.print("Date : ");
  MyString = Date(now());
  Serial.println(MyString);
  delay(1000);  // délai pour permettre la lecture sur l'afficheur
  //
  //------------------------------------------------------------------------------------------------
  // init gestion fichier carte SD
  //
  if (!CheckSDCard()) {
        // don't continue
    PrintLCD("SD card log KO  ", 1);
    while (true);   {  // bloque l'exécution sur cette ligne
      BlinkLED(RedLED);
    }
  }
  SD.begin(chipSelect);
  // création du fichier log de stockage
  MyString = Date(now(), Reversed) + "-" + UTCTime(now());
  String MyFileName = MyString;
  MyFileName.replace("/","-");
  MyFileName.replace(":","-");   // transforme date et heure pour en faire un nom acceptable pour un fichier
  MyFileName = "Log_" + MyFileName + ".csv";    // crée un nom de fichier au format csv et comportant la date
//  MyFileName ="TestLogD.csv";
  myFile = SD.open(MyFileName, FILE_WRITE); 
//  if (!myFile.open(MyFileName, O_WRONLY | O_CREAT | O_EXCL)) {
//    error("file.open");
//  }
  myFile.println(F("Electrical power consumption log file"));
  myFile.println("Start date / UTC time: " + MyString);
  myFile.println("Date" + Delimiter + "Heure_UTC" + Delimiter + "Power_W" + Delimiter +"Conso_Wh");         // imprime en en-tête le nom des colonnes
  Serial.print(F("Création fichier log : "));
  Serial.println(MyFileName);
//  myFile.close();
//  if (!myFile.open(MyFileName, O_WRONLY | O_CREAT | O_EXCL)) {
//    error("file.open");
//  }
  //
  //------------------------------------------------------------------------------------------------
  // Init serveur Web
  server.begin();
  Serial.print(F("Listening for clients on port ")); Serial.println(serverPort);
  //
  //------------------------------------------------------------------------------------------------
  // Mesure finale de calibration de l'ADC avec la diode bandgap
  ScaleFactorADC = InitScaleFactorADC();
  ScaleFactorADCdiff = 2 * ScaleFactorADC;
  CustomPrint("Facteur d'échelle : ", (ScaleFactorADC * Mega), " µV/LSB");
  VARef = ScaleFactorADC * float(ADCRange);
  CustomPrint("Tension AVCC 5V estimée : ", VARef , " V");
  Serial.println();
  int Measure =  analogRead(RefVoltageIn);  // mesure initiale pour ré-initiliaser le Mux ADC 
  //
  ConfigADC(ADCGainOption);       // applique le gain initial
  //
  //------------------------------------------------------------------------------------------------
  // init interrupts for sampling frequency
  float SampleTime = 1 / float(SamplingFreq);    // période d'échantillonage en s
  cli(); // Désactive l'interruption globale
  bitClear (TCCR2A, WGM20); // WGM20 = 0
  bitClear (TCCR2A, WGM21); // WGM21 = 0 
  TCCR2B = 0b00000110; // Clock / 256 soit 16 micro-s et WGM22 = 0
  TIMSK2 = 0b00000001; // Interruption locale autorisée par TOIE2
  Serial.println("Init interrupts done");
  sei(); // Active l'interruption globale
  TCNT2 = Timer_Value;  // initialise le timer 2
  LastTime = micros();      // initialise l'instant de début du 1er paquet avec l'instant de déclenchement des IT
  StartTime = millis();    // mémorise l'instant de démarrage des mesures
  LastLog = StartTime;     // le 1er enregistrement sur carte SD se fera DatalogInterval s après cet instant
  //
 
} // fin de du code Setup
//---------------------------------------------------------------------------------------------------
// Routine d'interruption
ISR(TIMER2_OVF_vect) {
  // traitement à chaque sampling
  // les mesures sont accumuées au format long correspondant à la sortie numérique de l'ADC
  // elles seront converties en VOLT u Ampère lors du traitement final
  //
  TCNT2 = Timer_Value; // réarme le timer 2
  // lancement convesion ADC
  //
  ADCSRA |= (1<<ADSC);                //start conversion
  while (ADCSRA & (1<<ADSC));         //wait untill coversion be completed(ADSC=0);
  TimeStamp = micros();               // mémmorise au plus près l'instant de conversion
  if (Breakpoint) {
    Serial.println(F("Au moins une IT reçue"));
    Breakpoint = false;
  }
  // formatage mesure
  low = ADCL;
  high = ADCH;
  if(high & (1<<1)){                      //in differential mode our value is between -512 to 511 (not 0 to 1023). it means we have 9 bits and 10th bit is the sign bit. but because
    high |= 0b11111110;                   //the number of ADCH and ADCL bits are 10, for signed number we dont have repeatition of 1 in "ADCH" byte.
  }                                       //so we repeat 1 Ourselves.:)
  int SensorValue = (high << 8) | low;    //arrange (ADCH ADCL) to a 16 bit and return it's value.
                                          // La mesure est un entier signé compris entre -512 et +511, la résolution est VCC/1024
  // accumulation mesures 
  Max = max(Max, SensorValue);
  Min = min(Min, SensorValue);
  Somme2Sensor += long(SensorValue * SensorValue);       // mémorise les valeurs
  SommeSensor += long(SensorValue);
  TrueCount++;                                           // compte le nombre d'échantillons réel (+2 par rapport à SamplingCount)
    if (varCompteur++ > SamplingCount) { // traitement après N sampling
    MeasurementTime = millis();         // mémorise le temps courant de la fin du paquet de mesures
    DeltaTime = TimeStamp - LastTime;    // calcule le temps total d'acquisition du paquet en µs
    varCompteur = 0;
    MaxaTraiter = Max;
    MinaTraiter = Min;
    TrueCountaTraiter = TrueCount;
    Somme2SensoraTraiter = Somme2Sensor; // mémorise les données à traiter pour traitement différé
    SommeSensoraTraiter = SommeSensor;
    PacketGain = ADCGain;                 // mémorise le gain ADC utilisé pour ce paquet car celui-ci peut être modifié à la fin de cette routine
    Somme2Sensor = 0;   // réinitialise les accumulateurs
    SommeSensor = 0;
    TrueCount = 0;
    Max = -512;
    Min = 511;
    LastTime = TimeStamp; 
    long MyValue = long(SensorValue);    // mémorise la dernière valeur, utile en debug
    DataReady = true;       // signale que des données sont prêtes
    //
    // Gestion changement de gain
    //
    if (GainRequestUp) {
      switch (ADCGainOption){
        case STD:
          ADCGainOption = MIDDLE;
          break;
        case MIDDLE:
          ADCGainOption = MAX;
          break;
      }
      ConfigADC(ADCGainOption);       // applique le nouveau gain
    }
    if (GainRequestDown) {
      switch (ADCGainOption){
        case MAX:
          ADCGainOption = MIDDLE;
          break;
        case MIDDLE:
          ADCGainOption = STD;
          break;
      }
      ConfigADC(ADCGainOption);       // applique le nouveau gain
    }
    if (Debug) {
      ADCGainOption = MIDDLE;
      ConfigADC(ADCGainOption);
    }
    ProcessingTime = micros() - TimeStamp;
  }
}
 
void loop() {
  //
  while (!DataReady) {     // boucle en attendant la disponibilité de data
    delay(100);
    }   
  // clignotement de la LED d'activité
  EtatLED = !EtatLED;     // inverse l'état de la LED
  if (EtatLED) {
    digitalWrite(GreenLED2,HIGH);   // allume la LED
  }
  else {
    digitalWrite(GreenLED2,LOW);   // éteint la LED
  }
  // traitement des données
  float Nech = float(TrueCountaTraiter);
  float RawBias = float(SommeSensoraTraiter) / Nech;
  float CurrentEff = sqrt(float(Somme2SensoraTraiter) / Nech - RawBias * RawBias) * ScaleFactorADCdiff * SensorScaleFactor / PacketGain;    // courant efficace en A
  float Power = MainVoltage * CurrentEff;         // Puissance consommée en Watt
  float Bias = RawBias * ScaleFactorADC * ScaleFactorADCdiff / PacketGain;
  float SamplingPeriod = float(DeltaTime *10 / TrueCountaTraiter +5) * 0.1;     // période d'échantillonnage moyenne en µs
  float SamplingFreq = 1.e6 / SamplingPeriod;
  // --------------------------- impression des résultats ---------------------------
  // Calcul des temps et dates
  Time = (unsigned long)(float(MeasurementTime-T0)* 0.001);               //  nombre de seconde par rapport à lheure NTP
  unsigned long MeasurementDate = Time + Time1970;
  String MyDate = Date(MeasurementDate) + Delimiter + UTCTime(MeasurementDate);
  unsigned int TimeFromLastLog = MeasurementTime - LastLog;   // intervale de temps écoulé en ms depuis le dernier enregistrement sur carte SD
  // traitements consommation cumulée et puissance moyenne
  CumulConso += ((unsigned long) (Power * float(DeltaTime) * 1.e-5 +5.)) / 10;            // incrémente le delta de consommation pendant le paquet en W.s
  float CumulConsoWh = float (CumulConso /3600);
  MeanPower = MeanPower + Power;          // somme les mesures de puissances pur calcul ultérieur de la moyenne;
  PacketCount++;                          // incrémente le nombre d packet traités depuis le dernier data log
  // impression
  Serial.print(MyDate);
//  CustomPrint("Sensor Max :",float(MaxaTraiter)," ");
//  CustomPrint("Sensor Min :",float(MinaTraiter)," ");
//  CustomPrint("Somme2SensoraTraiter : ", Somme2SensoraTraiter, " ");
//  CustomPrint("SommeSensoraTraiter : ", SommeSensoraTraiter, " ");
//  CustomPrint("TrueCount : ", long(TrueCountaTraiter), " ");
  CustomPrint(F(" - Cour. eff. :"),CurrentEff," A");
  CustomPrint(F("Biais ADC :"),Bias," A");
  CustomPrint(F("P : "),Power," W");
  CustomPrint(F("Fech. moy. : "), SamplingFreq," Hz");
  CustomPrint(F("Max proc. time : "),ProcessingTime, " µs");
  CustomPrint(F("Conso. : "),CumulConsoWh , " W/h");
  Serial.print(GainRequestUp);
  Serial.print(GainRequestDown);
  Serial.print(" ");
  Serial.println(ADCGainOption);
  // impression sur afficheur LCD
  String sPower;
  if (Power >= 100.) {
    sPower = String(Power,0);
  }
  else {
    sPower = String(Power,1);
  }
  sPower = ExtendString(sPower, 4);       // on étend le format à 4 caractères
  String sConso;
  if (CumulConsoWh >= 10000.) {
    sConso = String(CumulConsoWh*0.001,1) + " kWh";
  }
  else {
    sConso = String(CumulConsoWh,0) + " Wh";
  }
  sConso =ExtendString(sConso, 8);
  MyString = sPower + " W " + sConso;
  PrintLCD(MyString,0);
  switch (ADCGainOption) {
    case STD:
      DisplayGain = "L";
      break;
    case MIDDLE:
      DisplayGain = "M";
      break;
    case MAX:
      DisplayGain = "H";
      break;    
  }
  String sCurrent;
  if (CurrentEff >= 10.) {
    sCurrent = String(CurrentEff,1);    // soit 4 caractères
  }
  else {
    sCurrent = String(CurrentEff,2);    // soit 4 caractères
  }
  unsigned long TempsEcoule = MeasurementTime - StartTime;   // temps écoulé en ms
  int Hour = int(TempsEcoule/3600000);
  int Minute = int(TempsEcoule / 60000 - Hour * 60);
  MyString = String(Minute);  
  if (Minute < 10) {MyString = "0" + MyString;}    // cadre sur 2 digits
  String sTempsEcoule = String(Hour) + ":" + MyString;
  MyString = DisplayGain + " " + ExtendString(sTempsEcoule + " " + sCurrent + " A", 14);
  PrintLCD(MyString,1);
  // ------------------------- gestion data logging sur carte SD ------------------------
  if (LogCount > MaxLogCount) {   // il est temps de vider le buffer vers la carte SD pour ne pas perdre de données
    // nota interviendra le cas échéant dans la séquence suivant un log de data et non pendant cette séquence
    // on évite ainsi de cumuler les 2 actions dans la même séquence
    myFile.flush();        // vide le buffer
    LogCount = 0;          // remet à 0 le compteur de log
    Serial.println(F("Flush SD effectué"));
  }
  if (TimeFromLastLog >= DatalogInterval) {       // il est temps d'écrire les données sur la carte SD
    MeanPower = MeanPower / float(PacketCount);   // moyennes des mesures réalisées
    MyString = MyDate + Delimiter + String(MeanPower, 1) + Delimiter + String(CumulConsoWh);
    myFile.println(MyString);      // écriture sur la carte SD, on a environ 1s pour le faire
    // remet à 0 les variables de comptage
    MeanPower = 0;
    PacketCount = 0;  
    LastLog = MeasurementTime;
    lcd.noBacklight();                // on éteint le retroéclairage LCD
    //
    LogCount++;             // incrémente le compteur de log
    Serial.println(F("Log SD effectué"));
  }
   //
  DataReady = false;        // réinitialise le sémaphore
  //
  // Gestion du gain automatique
  //
  if (!(GainRequestDown || GainRequestUp)) {
    // on ne traite l'analyse du changement de gain que si une demande n'était pas déjà en cours car celles-ci sont prises en compte avec un décalage d'un pquet
    if (abs(Power/LastPower - 1) > 0.1) {
      // on diminue le gain
//      GainRequestDown = true;       // sera pris en compte dans la prochaine routine d'interruption en fin de paquet
    }
    else {
      // on analyse la puissance mesurée pour décider de commuter éventuellement sur un autre gain
      switch (ADCGainOption) {
          case MAX:   // on est à gain max, on doit baisser le gain si la puissance est supérieur au seuil
            if (Power >= Seuil1) {
              GainRequestDown = true;
            }
            break;
          case MIDDLE:   // on est à gain moyen, on doit baisser le gain si la puissance est > seuil haut et l'augmenter si la puissance est < au seuil bas
            if (Power >= Seuil2) {
              GainRequestDown = true;
            }
            if (Power < Seuil1) {
              GainRequestUp = true;
            }
            break;
          case STD:      // on est à gain min, on peut augmenetr le gain si la puissance est inférieure au seuil 2
            if (Power < Seuil2) {
              GainRequestUp = true;
            }
            break;
      }
    }
  }   //  fin analyse si changement de gain nécessaire
  else {
    // une demande était en cours et vient d'être prise en compte pour le prochain paquet
    // on peut donc remettre à zéro la demande
      GainRequestDown = false;
      GainRequestUp = false;
  }
  LastPower = Power;
  //
  // Traitement serveur
  // listen for incoming clients
  WiFiClient client = server.available();
  if (client) {
    Serial.println("New client");
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");  // the connection will be closed after completion of the response
          client.println("Refresh: 5");  // refresh the page automatically every 5 sec
          client.println();
          client.println("<!DOCTYPE HTML>");
          client.println("<html>");
          // output Power measurment and UTC date and time
          client.print("Power consumption is ");
          client.print(Power);
          client.print(" W at UTC time ");
          client.print(MyDate);
          client.println("<br />");
          client.println("</html>");
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);

    // close the connection:
    client.stop();
    Serial.println("Client disconnected");
  } 
}
//----------------------------------------------------------------------
String ExtendString(String aString, byte L) {   // permet de compléter une String avec des blancs pour obtenir une longueur constante
  int sL = aString.length();
  String ExtendedString = aString;
  if (sL < L) {
    for (int i = 0; i < (L -sL); i++) {
        ExtendedString = " " + ExtendedString;
    }
  }
  return ExtendedString;
}
//-----------------------------------------------------------------------
void BlinkLED(int LEDNumber) {
  digitalWrite(LEDNumber, LOW);
  delay(500);
  digitalWrite(LEDNumber, HIGH);
  delay(500);
}

void CustomPrint(String Text1, float Data, String Text2) {
  Serial.print(Text1);
  Serial.print(Data);
  Serial.print(Text2);
  Serial.print(" - ");
}

void CustomPrint(String Text1, unsigned long Data, String Text2) {
  Serial.print(Text1);
  Serial.print(Data);
  Serial.print(Text2);
  Serial.print(" - ");
}

void CustomPrint(String Text1, long Data, String Text2) {
  Serial.print(Text1);
  Serial.print(Data);
  Serial.print(Text2);
  Serial.print(" - ");
}

// routine d'impression simplifié sur LCD
void PrintLCD(String data, int Ligne) {
  lcd.setCursor(0, Ligne);
  lcd.print(data);
  lcd.print("                 ");   // sert à effacer les caractères restant d'une précédente impression
}

// routine d'impression sur LCD ou serial selon configuration
template <class AnyType>
void MyPrintln(AnyType data) {
  if (PrintLCD) {
    lcd.setCursor(0, CursorLine);
    CursorLine = 1 - CursorLine;
    lcd.print(data);
    lcd.setCursor(0, CursorLine);
    if (Debug) {
      Serial.println(data);
    }
  }
  else {
    Serial.println(data);
  }
}

template <class AnyType>
void MyPrint(AnyType data) {
  if (PrintLCD) {
    lcd.print(data);
    if (Debug) {
      Serial.print(data);
    }
  }
  else {
    Serial.print(data);
  }
}
//--------------------------------------------------------------------------
// routines com serveur NTP

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address) {
  //Serial.println("1");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  //Serial.println("2");
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  //Serial.println("3");

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  //Serial.println("4");
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  //Serial.println("5");
  Udp.endPacket();
  //Serial.println("6");
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

String Date(unsigned long epoch, boolean Reverse = false) {    // convertit une date au format Unix secondes depuis 01/01/1970 en jour/mois/année
  
  String sDate;if (Reverse) {
    sDate = String(year(epoch)) + "/" + String(month(epoch)) + "/" + String(day(epoch));
  }
  else {
    sDate = String(day(epoch)) + "/" + String(month(epoch)) + "/" + String(year(epoch));
  }
  return sDate;
}

unsigned long UnixTime(unsigned long secsSince1900) {
  // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
  const unsigned long seventyYears = 2208988800UL;
  // subtract seventy years:
  unsigned long epoch = secsSince1900 - seventyYears;
  return epoch;
}

unsigned long PandemicTime(unsigned long secsSince1900)  {
  // Pandemic time starts on Jan 01 2020. In seconds that's 3786825600
  const unsigned long ManyYears = 3786825600UL;
  // subtract many years:
  unsigned long epoch = secsSince1900 - ManyYears;
  return epoch;
}
String UTCTime(unsigned long epoch) {
   // provides UTC time from time in seconds frome 01/01/1970 (time at Greenwich Meridian (GMT))
    String Hour = String((epoch  % 86400L) / 3600); // calculate the hour (86400 equals secs per day)
    Serial.print(':');
    String Minute = String((epoch  % 3600) / 60); // calculate the minute (3600 equals secs per minute)
    if (((epoch % 3600) / 60) < 10) {
      // In the first 10 minutes of each hour, we'll want a leading '0'
      Minute = "0" + Minute;
    }
    String Seconds = String(epoch % 60); // calculate the second
    if ((epoch % 60) < 10) {
      // In the first 10 seconds of each minute, we'll want a leading '0' 
      Seconds = "0" + Seconds;
    }    
    return Hour + ":" + Minute + ":" + Seconds;
}

unsigned long GetTime() {
  sendNTPpacket(timeServer); // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(1000);
  unsigned long secsSince1900 = 0;
  if (Udp.parsePacket()) {
    Serial.println(F("packet received"));
    // We've received a packet, read the data from it
    Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    secsSince1900 = highWord << 16 | lowWord;
    Serial.print(F("Seconds since Jan 1 1900 = "));
    Serial.println(secsSince1900);

    // now convert NTP time into everyday time:
    Serial.print(F("Unix time = "));
        // print Unix time:
    unsigned long epoch = UnixTime(secsSince1900);
    Serial.println(epoch);

    // print the hour, minute and second:
    Serial.print(F("UTC time : "));       // UTC is the time at Greenwich Meridian (GMT)
    Serial.println(UTCTime(epoch));
  }
  else {
    secsSince1900 = 0 ;
  }
  return secsSince1900;
}

float InitScaleFactorADC(void)
    {  
// détermine un facteur d'échelle calibré de l'ADC à l'aide d'une mesure de la diode bandgap interne
// Nota : abandon le 03/02/2021 de la version utilisant un format long integer en µV/LSB
// soit V en Volt = N(ADC) * ScaleFactorADC()
//      
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
     // For mega boards
     const long InternalReferenceVoltage = 1103L;  // Adjust this value to your boards specific internal BG voltage x1000 => 1115 replaced by 1103
        // REFS1 REFS0          --> 0 1, AVcc internal ref.
        // MUX4 MUX3 MUX2 MUX1 MUX0  --> 11110 1.1V (VBG)
     ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (1<<MUX4) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);
#else
     // For 168/328 boards
     const long InternalReferenceVoltage = 1050L;  // Adust this value to your boards specific internal BG voltage x1000
        // REFS1 REFS0          --> 0 1, AVcc internal ref.
        // MUX3 MUX2 MUX1 MUX0  --> 1110 1.1V (VBG)
     ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);     
#endif
        // Start a conversion 
     ADCSRA |= _BV( ADSC );
        // Wait for it to complete
     while( ( (ADCSRA & (1<<ADSC)) != 0 ) );
        // Scale the value
     int results = (((InternalReferenceVoltage * 10000L) / ADC) + 5L) / 10L; // calcul au format long avec arrondi
     return float(results) * 1.e-6;
    }
void ConfigADC(enum GainSelect ADCGainOption)  {
  // Configuration de l'ADC pour mesures de tension différentielle
  // ADMUX = 1<<REFS0 choose AVCC for reference ADC voltage = default config
  switch(ADCGainOption) {
    case STD:
      ADMUX = (1<<REFS0)|(1<<MUX4);              //set MUX5:0 to 010000. Positive Differential Input => ADC0 and Negative Differential Input => ADC1 with Gain 1x.  
      ADCGain = 1.;
      break;
    case MIDDLE:
      ADMUX = (1<<REFS0)|(1<<MUX3)|(1<<MUX0);    //set MUX5:0 to 001001. Positive Differential Input => ADC1 and Negative Differential Input => ADC0 with Gain 10x
      ADCGain = 10.;
      break;
    case MAX:
      ADMUX = (1<<REFS0)|(1<<MUX3)|(1<<MUX1)|(1<<MUX0);    //set MUX5:0 to 001011. Positive Differential Input => ADC1 and Negative Differential Input => ADC0 with Gain 200x
      ADCGain = 200.;
      break;   
  }
  ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);    //enable ADC, ADC frequency=16MB/128=125kHz (set of prescaler)
  ADCSRB = 0x00; 
  }
//-------------------------------------------------------------------------------------------------------------------------------------------------
boolean CheckSDCard() {
  Serial.print("\nInitializing SD card...");
    // Initialize at the highest speed supported by the board that is
  // not over 50 MHz. Try a lower speed if SPI errors occur.
  if (!SD.begin(chipSelect, SD_SCK_MHZ(20))) {
    SD.initErrorPrint();
    return false;
  }
  else { return true;}
//  // we'll use the initialization code from the utility libraries
//  // since we're just testing if the card is working!
//  if (!card.init(SPI_HALF_SPEED, chipSelect)) {
//    Serial.println("initialization failed. Things to check:");
//    Serial.println("* is a card inserted?");
//    Serial.println("* is your wiring correct?");
//    Serial.println("* did you change the chipSelect pin to match your shield or module?");
//    return false;
//  } else {
//    Serial.println("Wiring is correct and a card is present.");
//  }
//
//  // print the type of card
//  Serial.println();
//  Serial.print("Card type:         ");
//  switch (card.type()) {
//    case SD_CARD_TYPE_SD1:
//      Serial.println("SD1");
//      break;
//    case SD_CARD_TYPE_SD2:
//      Serial.println("SD2");
//      break;
//    case SD_CARD_TYPE_SDHC:
//      Serial.println("SDHC");
//      break;
//    default:
//      Serial.println("Unknown");
//  }
//
//  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
//  if (!volume.init(card)) {
//    Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
//    return false;
//  }
//
//  Serial.print("Clusters:          ");
//  Serial.println(volume.clusterCount());
//  Serial.print("Blocks x Cluster:  ");
//  Serial.println(volume.blocksPerCluster());
//
//  Serial.print("Total Blocks:      ");
//  Serial.println(volume.blocksPerCluster() * volume.clusterCount());
//  Serial.println();
//
//  // print the type and size of the first FAT-type volume
//  uint32_t volumesize;
//  Serial.print("Volume type is:    FAT");
//  Serial.println(volume.fatType(), DEC);
//
//  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
//  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
//  volumesize /= 2;                           // SD card blocks are always 512 bytes (2 blocks are 1KB)
//  Serial.print("Volume size (Kb):  ");
//  Serial.println(volumesize);
//  Serial.print("Volume size (Mb):  ");
//  volumesize /= 1024;
//  Serial.println(volumesize);
//  Serial.print("Volume size (Gb):  ");
//  Serial.println((float)volumesize / 1024.0);
//
//  Serial.println("\nFiles found on the card (name, date and size in bytes): ");
//  root.openRoot(volume);
//
//  // list all files in the card with date and size
//  root.ls(LS_R | LS_DATE | LS_SIZE);
//  return true;
}
