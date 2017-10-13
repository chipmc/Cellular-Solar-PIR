/*
* Project Cellular-Solar-PIR
* Description: Cellular Connected Data Logger for Solar installations
* Author: Chip McClelland
* Date:8 October 2017
*/

// On the solar installation I am not using the hardware watchdog 

// Easy place to change global numbers
//These defines let me change the memory map and configuration without hunting through the whole program
#define VERSIONNUMBER 7             // Increment this number each time the memory map is changed
#define WORDSIZE 8                  // For the Word size
#define PAGESIZE 4096               // Memory size in bytes / word size - 256kb FRAM
// First Word - 8 bytes for setting global values
#define DAILYOFFSET 2               // First word of daily counts
#define HOURLYOFFSET 30             // First word of hourly counts (remember we start counts at 1)
#define DAILYCOUNTNUMBER 28         // used in modulo calculations - sets the # of days stored
#define HOURLYCOUNTNUMBER 4064      // used in modulo calculations - sets the # of hours stored - 256k (4096-14-2)
#define VERSIONADDR 0x0             // Memory Locations By Name not Number
#define SENSITIVITYADDR 0x1         // For the 1st Word locations
#define DEBOUNCEADDR 0x2            // One byte for debounce (stored in cSec mult by 10 for mSec)
#define RESETCOUNT 0x3              // This is where we keep track of how often the Electron was reset
#define DAILYPOINTERADDR 0x4        // One byte for daily pointer
#define HOURLYPOINTERADDR 0x5       // Two bytes for hourly pointer
#define CONTROLREGISTER 0x7         // This is the control register acted on by both Simblee and Arduino
//Second Word - 8 bytes for storing current counts
#define CURRENTHOURLYCOUNTADDR 0x8  // Current Hourly Count
#define CURRENTDAILYCOUNTADDR 0xA   // Current Daily Count
#define CURRENTCOUNTSTIME 0xC       // Time of last count
//These are the hourly and daily offsets that make up the respective words
#define DAILYDATEOFFSET 1           //Offsets for the value in the daily words
#define DAILYCOUNTOFFSET 2          // Count is a 16-bt value
#define DAILYBATTOFFSET 4           // Where the battery charge is stored
#define HOURLYCOUNTOFFSET 4         // Offsets for the values in the hourly words
#define HOURLYBATTOFFSET 6          // Where the hourly battery charge is stored
// Finally, here are the variables I want to change often and pull them all together here
#define SOFTWARERELEASENUMBER "0.15"
#define PARKCLOSES 22
#define PARKOPENS 7

// Included Libraries
#include "Adafruit_FRAM_I2C.h"                           // Library for FRAM functions
#include "FRAM-Library-Extensions.h"                     // Extends the FRAM Library
#include "electrondoc.h"                                 // Documents pinout


// Prototypes and System Mode calls
SYSTEM_MODE(SEMI_AUTOMATIC);    // This will enable user code to start executing automatically.
SYSTEM_THREAD(ENABLED);         // Means my code will not be held up by Particle processes.
FuelGauge batteryMonitor;       // Prototype for the fuel gauge (included in Particle core library)
PMIC pmic;                      //Initalize the PMIC class so you can call the Power Management functions below.


// State Maching Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, SLEEPING_STATE, NAPPING_STATE, REPORTING_STATE, RESP_WAIT_STATE };
State state = INITIALIZATION_STATE;

// Pin Constants
const int intPin = D3;                      // Acclerometer interrupt pin
const int blueLED = D7;                     // This LED is on the Electron itself
const int userSwitch = D5;                  // User switch with a pull-up resistor
const int tmp36Pin = A0;             // Simple Analog temperature sensor
const int tmp36Shutdwn = B5;         // Can turn off the TMP-36 to save energy


// Timing Variables
unsigned long resetWaitTimeStamp = 0;       // Starts the reset wait clock
unsigned long resetWaitTime = 30000;        // Will wait this lonk before resetting.
unsigned long sleepDelay = 60000;           // Amount of time to stay awake after an event - too short and could cost power
unsigned long lastEvent = 0;                // Keeps track of the last time there was an event
bool waiting = false;
int currentPeriod = 0;                      // Change length of period for testing 2 places in main loop

// Program Variables
int temperatureF;                    // Global variable so we can monitor via cloud variable
int resetCount;                      // Counts the number of times the Electron has had a pin reset
bool ledState = LOW;                        // variable used to store the last LED status, to toggle the light
const char* releaseNumber = SOFTWARERELEASENUMBER;  // Displays the release on the menu

// FRAM and Unix time variables
time_t t;
byte lastHour = 0;                   // For recording the startup values
byte lastDate = 0;                   // These values make sure we record events if time has lapsed
int hourlyPersonCount = 0;            // hourly counter
int hourlyPersonCountSent = 0;        // Person count in flight to Ubidots
int dailyPersonCount = 0;             //  daily counter
int dailyPersonCountSent = 0;         // Daily person count in flight to Ubidots
bool dataInFlight = false;           // Tracks if we have sent data but not yet cleared it from counts until we get confirmation
byte currentHourlyPeriod;            // This is where we will know if the period changed
byte currentDailyPeriod;             // We will keep daily counts as well as period counts
int countTemp = 0;                   // Will use this to see if we should display a day or hours counts


// PIR Sensor variables
volatile bool sensorDetect = false;         // This is the flag that an interrupt is triggered

// Battery monitor
int stateOfCharge = 0;                      // stores battery charge level value

//Menu and Program Variables
 unsigned long lastBump = 0;         // set the time of an event
 boolean inTest = false;             // Are we in a test or not
 int numberHourlyDataPoints;         // How many hourly counts are there
 int numberDailyDataPoints;          // How many daily counts are there
 char Signal[17];                            // Used to communicate Wireless RSSI and Description
 char* levels[6] = {"Poor", "Low", "Medium", "Good", "Very Good", "Great"};


void setup()
{
  Particle.connect();                         // Connect to Particle on bootup - will disonnect on nap or sleep
  Serial.begin(9600);                         // Serial for debugging, will come out later
  pmic.begin();                               // For power management
  delay(3000);    // Temp so we can get connected
  Serial.println("");                         // Header information
  Serial.print(F("Electron-Sleep-Test-PIR - release "));
  Serial.println(releaseNumber);

  //pmic.setChargeCurrent(0,0,1,0,0,0);         //Set charging current to 1024mA (512 + 512 offset) thank you @RWB for these two lines
  pmic.setInputVoltageLimit(4840);            //Set the lowest input voltage to 4.84 volts. This keeps my 5v solar panel from operating below 4.84 volts.
  stateOfCharge = int(batteryMonitor.getSoC()); // Percentage of full charg
  Serial.print("Charge current limit is: ");
  Serial.println(pmic.getChargeCurrent());

  pinMode(intPin,INPUT);                      // PIR interrupt pinMode
  pinMode(blueLED, OUTPUT);                   // declare the Blue LED Pin as an output

  attachInterrupt(intPin,sensorISR,RISING);   // Will know when the PIR sensor is triggered
  char responseTopic[125];
  String deviceID = System.deviceID();
  deviceID.toCharArray(responseTopic,125);
  Particle.subscribe(responseTopic, UbidotsHandler, MY_DEVICES);      // Subscribe to the integration response event

  Particle.variable("HourlyCount", hourlyPersonCount);
  Particle.variable("DailyCount", dailyPersonCount);
  Particle.variable("Signal", Signal);
  Particle.variable("ResetCount", resetCount);
  Particle.variable("Temperature",temperatureF);
  Particle.variable("Release",releaseNumber);
  Particle.variable("stateOfChg", stateOfCharge);

  Particle.function("startStop", startStop);
  Particle.function("resetFRAM", resetFRAM);
  Particle.function("resetCounts",resetCounts);
  Particle.function("SendNow",sendNow);

  if (fram.begin()) {                // you can stick the new i2c addr in here, e.g. begin(0x51);
    Serial.println(F("Found I2C FRAM"));
  } else {
    Serial.println(F("No I2C FRAM found ... check your connections"));
  }

  if (FRAMread8(VERSIONADDR) != VERSIONNUMBER) {  // Check to see if the memory map in the sketch matches the data on the chip
    Serial.print(F("FRAM Version Number: "));
    Serial.println(FRAMread8(VERSIONADDR));
    Serial.read();
    Serial.println(F("Memory/Sketch mismatch! Erase FRAM? (Y/N)"));
    while (!Serial.available());
    switch (Serial.read()) {    // Give option to erase and reset memory
    case 'Y':
      ResetFRAM();
      break;
    case 'y':
      ResetFRAM();
      break;
    default:
      Serial.println(F("Cannot proceed"));
      BlinkForever();
    }
  }

  resetCount = FRAMread8(RESETCOUNT);         // Retrive system recount data from FRAMwrite8
  if (System.resetReason() == RESET_REASON_PIN_RESET)  // Check to see if we are starting from a pin reset
  {
  resetCount++;
  FRAMwrite8(RESETCOUNT,resetCount);          // If so, store incremented number - watchdog must have done This
  }
  Serial.print("Reset count: ");
  Serial.println(resetCount);

  Time.zone(-4);                              // Set time zone to Eastern USA daylight saving time
  getSignalStrength();                        // Test signal strength at startup
  StartStopTest(1);                           // Default action is for the test to be running
  state = IDLE_STATE;                         // Idle and look for events to change the state
}

void loop()
{
  switch(state) {
  case IDLE_STATE:
    if (Time.hour() != currentPeriod)                       // Spring into action each hour on the hour
    {
      currentPeriod = Time.hour();                          // Set the new current period
      state = REPORTING_STATE;                              // We want to report on the hour
      break;
    }
    if (sensorDetect) recordCount();                        // The ISR had raised the sensor flag
    if (millis() >= (lastEvent + sleepDelay)) state = NAPPING_STATE;  // Too long since last sensor flag - time to nap
    if (Time.hour() >= PARKCLOSES) state = SLEEPING_STATE;  // The park is closed, time to sleep
    break;

  case SLEEPING_STATE: {
    Particle.disconnect();                                   // Disconnect from Particle in prep for sleep
    delay(3000);
    Cellular.disconnect();                                   // Disconnect from the cellular network
    delay(3000);
    Cellular.off();                                          // Turn off the cellular modem    digitalWrite(blueLED,LOW);    // Turn off the on-board light
    digitalWrite(blueLED,LOW);                               // Turn off the LED
    lastEvent = millis();                                    // Reset millis so we don't wake and then nap again
    Serial.println("Park closed go to sleep");
    long secondsToOpen = (3600*(24 - Time.hour()+PARKOPENS));  // Set the alarm (in seconds) till park opens again
    System.sleep(SLEEP_MODE_DEEP,secondsToOpen);              // Sleep till morning
    state = REPORTING_STATE;                                  // Report when we wake from sleep
    } break;

  case NAPPING_STATE: {
    Particle.disconnect();                                   // Disconnect from Particle in prep for sleep
    delay(3000);
    Cellular.disconnect();                                   // Disconnect from the cellular network
    delay(3000);
    Cellular.off();                                          // Turn off the cellular modem    digitalWrite(blueLED,LOW);    // Turn off the on-board light
    sensorDetect = true;                                     // Woke up so there must have been an event
    lastEvent = millis();                                    // Reset millis so we don't wake and then nap again
    digitalWrite(blueLED,LOW);                               // Turn off the LED
    Serial.print("Going to sleep until event or ");
    int secondsToHour = (60*(60 - Time.minute()));      // Time till the top of the hour
    Serial.print(secondsToHour);
    Serial.println(" seconds");
    delay(1000);
    System.sleep(intPin,RISING,secondsToHour);               // Sensor will wake us with an interrupt
    attachInterrupt(intPin,sensorISR,RISING);                // Sensor interrupt from low to high
    sleepDelay = 10000;                                      // Sets the sleep delay to 10 seconds after a nap
    state = IDLE_STATE;                                      // Back to the IDLE_STATE after a nap
    } break;

  case REPORTING_STATE: {
    bool success = false;                             // Was data received
    Cellular.on();                                           // turn on the Modem
    Cellular.connect();                                      // Connect to the cellular network
    while(!Cellular.ready())
    Particle.connect();                                      // Connect to Particle
    while(!Particle.connected());
    Particle.process();
    success = Particle.publish("State","Reporting");         // Sample message.
    sleepDelay = 60000;     // Sets the sleep delay to 60 seconds after reporting to give time to flash if needed
    if (success) state = IDLE_STATE;                         // Success - go to IDLE_STATE
    else state = ERROR_STATE;                                // Failure - go to ERROR_STATE
    } break;

  case ERROR_STATE:                                          // To be enhanced - where we deal with errors
    if (!waiting)                                            // Will use this flag to wiat 30 seconds before reset
    {
      waiting = true;
      resetWaitTimeStamp = millis();
      Particle.publish("State","Resetting in 30 sec");
    }
    if (millis() >= (resetWaitTimeStamp + resetWaitTime)) System.reset();   // Today, only way out is reset
    break;
  }
}

void recordCount()                                          // Handles counting when the sensor triggers
{
  sensorDetect = false;                                     // Reset the flag
  if (digitalRead(intPin)) {
    Serial.println("It is a tap - counting");
    lastEvent = millis();                                     // Important to keep from napping too soon
    hourlyPersonCount++;                                      // Increment the PersonCount
    Serial.print("Hourly: ");                                 // Serial reporting for debugging
    Serial.print(hourlyPersonCount);
    Serial.print("  Time: ");
    Serial.println(Time.timeStr());                           // Prints time string example: Wed May 21 01:08:47 2014
    ledState = !ledState;                                     // toggle the status of the LEDPIN:
    digitalWrite(blueLED, ledState);                          // update the LED pin itself
  }
}

void StartStopTest(boolean startTest)  // Since the test can be started from the serial menu or the Simblee - created a function
 {
     if (startTest) {
         inTest = true;
         t = Time.now();                    // Gets the current time
         currentHourlyPeriod = Time.hour();   // Sets the hour period for when the count starts (see #defines)
         currentDailyPeriod = Time.day();     // And the day  (see #defines)
         // Deterimine when the last counts were taken check when starting test to determine if we reload values or start counts over
         time_t unixTime = FRAMread32(CURRENTCOUNTSTIME);
         lastHour = Time.hour(unixTime);
         lastDate = Time.day(unixTime);
         dailyPersonCount = FRAMread16(CURRENTDAILYCOUNTADDR);  // Load Daily Count from memory
         hourlyPersonCount = FRAMread16(CURRENTHOURLYCOUNTADDR);  // Load Hourly Count from memory
         if (currentDailyPeriod != lastDate) {
             LogHourlyEvent();
             LogDailyEvent();
         }
         else if (currentHourlyPeriod != lastHour) {
             LogHourlyEvent();
         }
         Serial.println(F("Test Started"));
     }
     else {
         inTest = false;
         t = Time.now();
         FRAMwrite16(CURRENTDAILYCOUNTADDR, dailyPersonCount);   // Load Daily Count to memory
         FRAMwrite16(CURRENTHOURLYCOUNTADDR, hourlyPersonCount);  // Load Hourly Count to memory
         FRAMwrite32(CURRENTCOUNTSTIME, t);   // Write to FRAM - this is so we know when the last counts were saved
         hourlyPersonCount = 0;        // Reset Person Count
         dailyPersonCount = 0;         // Reset Person Count
         Serial.println(F("Test Stopped"));
     }
 }

void LogHourlyEvent() // Log Hourly Event()
{
  time_t LogTime = FRAMread32(CURRENTCOUNTSTIME);     // This is the last event recorded - this sets the hourly period
  unsigned int pointer = (HOURLYOFFSET + FRAMread16(HOURLYPOINTERADDR))*WORDSIZE;  // get the pointer from memory and add the offset
  LogTime -= (60*Time.minute(LogTime) + Time.second(LogTime)); // So, we need to subtract the minutes and seconds needed to take to the top of the hour
  FRAMwrite32(pointer, LogTime);   // Write to FRAM - this is the end of the period
  FRAMwrite16(pointer+HOURLYCOUNTOFFSET,hourlyPersonCount);
  stateOfCharge = int(batteryMonitor.getSoC());
  FRAMwrite8(pointer+HOURLYBATTOFFSET,stateOfCharge);
  unsigned int newHourlyPointerAddr = (FRAMread16(HOURLYPOINTERADDR)+1) % HOURLYCOUNTNUMBER;  // This is where we "wrap" the count to stay in our memory space
  FRAMwrite16(HOURLYPOINTERADDR,newHourlyPointerAddr);
}

 void LogDailyEvent() // Log Daily Event()
 {
   time_t LogTime = FRAMread32(CURRENTCOUNTSTIME);// This is the last event recorded - this sets the daily period
   int pointer = (DAILYOFFSET + FRAMread8(DAILYPOINTERADDR))*WORDSIZE;  // get the pointer from memory and add the offset
   FRAMwrite8(pointer,Time.month(LogTime)); // The month of the last count
   FRAMwrite8(pointer+DAILYDATEOFFSET,Time.day(LogTime));  // Write to FRAM - this is the end of the period  - should be the day
   FRAMwrite16(pointer+DAILYCOUNTOFFSET,dailyPersonCount);
   stateOfCharge = batteryMonitor.getSoC();
   FRAMwrite8(pointer+DAILYBATTOFFSET,stateOfCharge);
   byte newDailyPointerAddr = (FRAMread8(DAILYPOINTERADDR)+1) % DAILYCOUNTNUMBER;  // This is where we "wrap" the count to stay in our memory space
   FRAMwrite8(DAILYPOINTERADDR,newDailyPointerAddr);
 }

 void sendEvent(bool dailyEvent)
 {
   getSignalStrength();
   int currentTemp = getTemperature();  // in degrees F
   stateOfCharge = int(batteryMonitor.getSoC()); // Percentage of full charge
   digitalWrite(donePin, HIGH);
   digitalWrite(donePin,LOW);     // Pet the dog so we have a full period for a response
   doneEnabled = false;           // Can't pet the dog unless we get a confirmation via Webhook Response and the right Ubidots code.
   char data[256];                                         // Store the date in this character array - not global
   snprintf(data, sizeof(data), "{\"hourly\":%i, \"daily\":%i,\"battery\":%i, \"temp\":%i}",hourlyPersonCount, dailyPersonCount, stateOfCharge, currentTemp);
   Particle.publish("Send_Counts", data, PRIVATE);
   if (dailyEvent)
   {
     dailyPersonCountSent = dailyPersonCount; // This is the number that was sent to Ubidots - will be subtracted once we get confirmation
     currentDailyPeriod = Time.day();  // Change the time period
   }
   hourlyPersonCountSent = hourlyPersonCount; // This is the number that was sent to Ubidots - will be subtracted once we get confirmation
   dataInFlight = true; // set the data in flight flag
   currentHourlyPeriod = Time.hour();  // Change the time period
   Serial.println(F("Event Sent"));
 }

void UbidotsHandler(const char *event, const char *data)  // Looks at the response from Ubidots - Will reset Photon if no successful response
{
  // Response Template: "{{hourly.0.status_code}}"
  if (!data) {                                            // First check to see if there is any data
    Particle.publish("UbidotsResp", "No Data");
    return;
  }
  int responseCode = atoi(data);                          // Response is only a single number thanks to Template
  if ((responseCode == 200) || (responseCode == 201))
  {
    Particle.publish("UbidotsHook","Success");
    Serial.println("Request successfully completed");
    dataInFlight = false;                                 // Data has been received
    doneEnabled = true;                                   // Successful response - can pet the dog again
    digitalWrite(donePin, HIGH);                          // If an interrupt came in while petting disabled, we missed it so...
    digitalWrite(donePin, LOW);                           // will pet the dog just to be safe
  }
  else Particle.publish("UbidotsHook", data);             // Publish the response code
}



void getSignalStrength()
{
    CellularSignal sig = Cellular.RSSI();  // Prototype for Cellular Signal Montoring
    int rssi = sig.rssi;
    int strength = map(rssi, -131, -51, 0, 5);
    sprintf(Signal, "%s: %d", levels[strength], rssi);
}

int startStop(String command)   // Will reset the local counts
{
  if (command == "1" && !inTest)
  {
    StartStopTest(1);
    return 1;
  }
  else if (command == "0" && inTest)
  {
    StartStopTest(0);
    return 1;
  }
  else
  {
    Serial.print("Got here but did not work: ");
    Serial.println(command);
    return 0;
  }
}

int resetFRAM(String command)   // Will reset the local counts
{
  if (command == "1")
  {
    ResetFRAM();
    return 1;
  }
  else return 0;
}

int resetCounts(String command)   // Resets the current hourly and daily counts
{
  if (command == "1")
  {
    FRAMwrite16(CURRENTDAILYCOUNTADDR, 0);   // Reset Daily Count in memory
    FRAMwrite16(CURRENTHOURLYCOUNTADDR, 0);  // Reset Hourly Count in memory
    hourlyPersonCount = 0;                    // Reset count variables
    dailyPersonCount = 0;
    hourlyPersonCountSent = 0;                // In the off-chance there is data in flight
    dataInFlight = false;
    return 1;
  }
  else return 0;
}

int sendNow(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    sendEvent(0);
    return 1;
  }
  else return 0;
}

int getTemperature()
{
  int reading = analogRead(tmp36Pin);   //getting the voltage reading from the temperature sensor
  float voltage = reading * 3.3;        // converting that reading to voltage, for 3.3v arduino use 3.3
  voltage /= 4096.0;                    // Electron is different than the Arduino where there are only 1024 steps
  int temperatureC = int(((voltage - 0.5) * 100));  //converting from 10 mv per degree with 500 mV offset to degrees ((voltage - 500mV) times 100) - 5 degree calibration
  temperatureF = int((temperatureC * 9.0 / 5.0) + 32.0);  // now convert to Fahrenheit
  return temperatureF;
}

void watchdogISR()
{
  if (doneEnabled)
  {
    digitalWrite(donePin, HIGH);
    digitalWrite(donePin, LOW);
  }
}


void sensorISR()
{
  sensorDetect = true;                                      // sets the sensor flag for the main loop
}
