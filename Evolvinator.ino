/*
Evolvinator
 
 REV 1
 
 This program controls the Evolvinator. There are 3 main components:
 Pump Control - dictates the speed of the p-1 perstaltic pump
 OD Sensor - makes OD measurements
 - also controls the valve 
 Temp Control - monitors the temperature and feeds back to the heat resistors
 
 Search ENTER to see where you need to input parameters
 Search CALIBRATION to see where you need to input calibration data
 */

/* >>>>>>>>>>>>>>>>>>>>>>>>>>> Constants and Variables <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */
// Libraries
#include <SPI.h> //library handles communication with the ethernet shield
#include <Ethernet.h> //for communication with NTP server and 
#include <EthernetUdp.h> //handles incoming/outgoing packets through the ethernet shield
#include <TimeLib.h> //used to update and display time
#include <SD.h> //handles writing data to SD card
#include <PID_v1.h> //used to set up cutoff point for OD sensor

// Ethernet
byte mac[] = { 
  0x90, 0xA2, 0xDA, 0x00, 0x4F, 0x74 };   // assign's ethernet shield's MAC Adress
IPAddress ip(192, 168, 100, 52);          // ENTER IP address 
unsigned int localPort = 8888;
EthernetServer server(80);                // default web request server
EthernetUDP ethernet_UDP;

// Flow
const byte pinP1FlowWrite = 0;            // which pin tells p1 (through pin 14) what speed (0-200 Hz)
unsigned long feedFrequency = 180000;     // frequency of the pulses given (default 1 ever 3 minutes)

// OD
const byte pinODLED = 1;                  // pin that powers the OD LED
const byte pinODRead = A1;                // pin that reads the OD sensor
const byte pinValve = 3;                  // pin that controls the valve
float ODDesired = 0.5;                    // Set desired OD
float ODMin[10];                          // stores recent OD measurements (current = ODMin[0]                         
float OD3MinAvg;
float ODZero = 0;                         // photodiode blank reading 

// Temp - temp is temperature of sensor (metal) unless otherwise indicated
const byte pinTempRead = A0;              // analog input will read variable voltage from AD22100
const byte pinTempWrite = 5;              // sends PWM to resistors to control temp
float tempDesired = 37;                   // Set desired temperature
float tempPrintAvg;                       // temperature converted to water temp
double temp, tempPWM, tempDesiredPID;
double aggKp = .5, aggKi = 0.1, aggKd = 0.1;
double consKp = .2, consKi = 0.01, consKd = 0.05;
PID tempPID(&temp, &tempPWM, &tempDesiredPID, aggKp, aggKi, aggKd, DIRECT);

// UV LED
const byte pinUVLED = 2;                  // pin that powers the UV LED

// Modes
boolean debugMode = true;
boolean calibrationMode = false;

// SD
const int pinSD = 4;

// time
unsigned long currentMs;
unsigned long oldMsTempRead = 0;
unsigned long oldMsTempCheck = 0;
unsigned long oldMsODRead = 0;
unsigned long oldMsPulseFed = 0;         

// timing
time_t epoch;
time_t tStart;                            // starting time
time_t t;                                 // current time
time_t tElapsed;                          // elapsed time (s)
time_t tPulse;                            // time of the last pulse
time_t tBackup;                           // reference time for if time sync with NTP server fails
unsigned long msBackup;                   // associated arduino clock time
unsigned long tUnixStart;                 // unix time at run start
unsigned long tUnix;                      // unix time
unsigned long msElapsedPrestart;          // ms elapsed run start.


/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Setup - runs once <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */
void setup() {
  // General
  analogReference(DEFAULT);                // analog input reference is 5 V

  // Serial, Ethernet
  Serial.begin(9600);
  pinMode(53, OUTPUT);                      // SS pin on Mega
  ethernet_UDP.begin(localPort);
  Ethernet.begin(mac, ip);
  server.begin();
  delay(1);                                 // give it a msecond to connect

  // Pump Control
  pinMode(pinP1FlowWrite, OUTPUT);          // output to pump what speed it should go (tone between 0-200 Hz)
  flowSet();

  // OD Sensor
  pinMode(pinODLED, OUTPUT);                // pin that controls LED is output
  digitalWrite(pinODLED, LOW);              // light off by default
  pinMode(pinODRead, INPUT);                // pin that reads photodiode is input
  digitalWrite(pinODRead, HIGH);            // enable 20k pullup resistor
  pinMode(pinValve, OUTPUT);                // pin that controls valve is output
  digitalWrite(pinValve, LOW);              // valve open at start

  // Temp Control
  pinMode(pinTempRead, INPUT);              // pin that reads the temp sensor is input
  pinMode(pinTempWrite, OUTPUT);            // pin that controls heating resistors is output
  tempSet();
  tempPID.SetMode(AUTOMATIC);
  tempPID.SetSampleTime(10000);
  tempPID.SetOutputLimits(0, 70);

  // SD
  SD.begin(pinSD);
}

/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Loop - is program <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */
void loop() {

  // If run has started
  if (tStart) {
    // Take OD measurement ever minute
    currentMs = millis();
    if (currentMs - oldMsODRead > 60000) {
      ODRead();
      oldMsODRead = currentMs;
    }

    // Feed pulse if threshold is reached and it's been long enough
    currentMs = millis();
    if (OD3MinAvg > ODDesired && currentMs - oldMsPulseFed > feedFrequency) {
      pulseFeed();
      oldMsPulseFed = currentMs;
    }
  }

  // Check temp every 5 seconds
  currentMs = millis();                    
  if (currentMs - oldMsTempRead > 5000) {  
    tempRead();
    oldMsTempRead = currentMs;
  }
  // PID adjust every 10 seconds
  tempWrite(); 

  // Check and adjust time if neccessary
  currentMs = millis();
  timeCheck();  
  
  // Check for web requests
  webLoop(); 
}

/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Functions - List function calls below <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< 
 0 Start Run
 1 Flow Control
 1a flowSet
 1b pulseFeed
 1c addMedia 
 2 OD Sensor
 2a ODRead
 2b ODCalibrate
 3 Temperature Control
 3a tempSet
 3b tempRead
 3c tempWrite
 4 Time
 5 WebLoop
 6 SD Card
 6a SDInitialize
 6b SDDataLog
 6c SDWebLoad
 */

// 0 startRun  ----------------------------------
void startRun() {
  tStart = now();
  tElapsed = 0;
  tUnixStart += (millis() - msElapsedPrestart) / 1000;    // to adjust unix time
  tUnix = tUnixStart + tElapsed;
  SDInitialize();
  digitalWrite(pinValve, LOW);          // open air valve
}

