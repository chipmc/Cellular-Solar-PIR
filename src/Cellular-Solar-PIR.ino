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
#define DEBOUNCEADDR 0x2            // One uint8_t for debounce (stored in cSec mult by 10 for mSec)
#define RESETCOUNT 0x3              // This is where we keep track of how often the Electron was reset
#define DAILYPOINTERADDR 0x4        // One uint8_t for daily pointer
#define HOURLYPOINTERADDR 0x5       // Two bytes for hourly pointer
#define CONTROLREGISTER 0x7         // This is the control register for storing the current state - future use
//Second Word - 8 bytes for storing current counts
#define CURRENTHOURLYCOUNTADDR 0x8  // Current Hourly Count - 16 bits
#define CURRENTDAILYCOUNTADDR 0xA   // Current Daily Count - 16 bits
#define CURRENTCOUNTSTIME 0xC       // Time of last count - 32 bits
//These are the hourly and daily offsets that make up the respective words
#define DAILYDATEOFFSET 1           //Offsets for the value in the daily words
#define DAILYCOUNTOFFSET 2          // Count is a 16-bt value
#define DAILYBATTOFFSET 4           // Where the battery charge is stored
#define HOURLYCOUNTOFFSET 4         // Offsets for the values in the hourly words
#define HOURLYBATTOFFSET 6          // Where the hourly battery charge is stored
// Finally, here are the variables I want to change often and pull them all together here
#define SOFTWARERELEASENUMBER "0.60"
#define PARKCLOSES 20
#define PARKOPENS 7

// Included Libraries
#include "Adafruit_FRAM_I2C.h"                           // Library for FRAM functions
#include "FRAM-Library-Extensions.h"                     // Extends the FRAM Library
#include "electrondoc.h"                                 // Documents pinout

// Prototypes and System Mode calls
SYSTEM_MODE(SEMI_AUTOMATIC);    // This will enable user code to start executing automatically.
SYSTEM_THREAD(ENABLED);         // Means my code will not be held up by Particle processes.
FuelGauge batteryMonitor;       // Prototype for the fuel gauge (included in Particle core library)
PMIC power;                      //Initalize the PMIC class so you can call the Power Management functions below.

// State Maching Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, SLEEPING_STATE, NAPPING_STATE, REPORTING_STATE, RESP_WAIT_STATE };
State state = INITIALIZATION_STATE;

// Pin Constants
const int intPin = D3;                      // Acclerometer interrupt pin
const int blueLED = D7;                     // This LED is on the Electron itself
const int userSwitch = D5;                  // User switch with a pull-up resistor
const int tmp36Pin = A0;                    // Simple Analog temperature sensor
const int tmp36Shutdwn = B5;                // Can turn off the TMP-36 to save energy
const int donePin = D6;                     // Pin the Electron uses to "pet" the watchdog
const int wakeUpPin = A7;                   // This is the Particle Electron WKP pin
const int hardResetPin = D4;                // Power Cycles the Electron and the Carrier Board


// Timing Variables
unsigned long publishTimeStamp = 0;         // Keep track of when we publish a webhook
unsigned long webhookWaitTime = 45000;      // How long will we let a webhook go before we give up
unsigned long resetWaitTimeStamp = 0;       // Starts the reset wait clock
unsigned long resetWaitTime = 30000;        // Will wait this lonk before resetting.
unsigned long sleepDelay = 90000;           // Longer delay before sleep when booting up or on the hour - gives time to flash
unsigned long timeTillSleep = 0;            // This will either be short or long depending on nap or sleep
unsigned long napDelay = 3000;              // Normal amount of time after event before taking a nap
unsigned long lastEvent = 0;                // Keeps track of the last time there was an event
bool waiting = false;                       // Keeps track of things that are in flight - enables non-blocking code
bool readyForBed = false;                   // Keeps track of the things that you do once before sleep

// Program Variables
int temperatureF;                    // Global variable so we can monitor via cloud variable
int resetCount;                      // Counts the number of times the Electron has had a pin reset
bool ledState = LOW;                        // variable used to store the last LED status, to toggle the light
const char* releaseNumber = SOFTWARERELEASENUMBER;  // Displays the release on the menu

// FRAM and Unix time variables
time_t t;
byte lastHour = 0;                   // For recording the startup values
byte lastDate = 0;                   // These values make sure we record events if time has lapsed
int hourlyPersonCount = 0;           // hourly counter
int hourlyPersonCountSent = 0;       // Person count in flight to Ubidots
int dailyPersonCount = 0;            //  daily counter
int dailyPersonCountSent = 0;        // Daily person count in flight to Ubidots
bool dataInFlight = false;           // Tracks if we have sent data but not yet cleared it from counts until we get confirmation
byte currentHourlyPeriod;            // This is where we will know if the period changed
byte currentDailyPeriod;             // We will keep daily counts as well as period counts

// PIR Sensor variables
volatile bool sensorDetect = false;         // This is the flag that an interrupt is triggered

// Battery monitor
int stateOfCharge = 0;                      // stores battery charge level value

//Menu and Program Variables
uint32_t lastBump = 0;         // set the time of an event
bool inTest = false;             // Are we in a test or not
retained char Signal[17];                            // Used to communicate Wireless RSSI and Description
const char* levels[6] = {"Poor", "Low", "Medium", "Good", "Very Good", "Great"};

void setup()
{
  pinMode(intPin,INPUT);
  pinMode(wakeUpPin,INPUT);                   // This pin is active HIGH
  pinMode(blueLED, OUTPUT);                   // declare the Blue LED Pin as an output
  pinMode(tmp36Shutdwn,OUTPUT);               // Supports shutting down the TMP-36 to save juice
  digitalWrite(tmp36Shutdwn, HIGH);           // Turns on the temp sensor
  pinMode(donePin,OUTPUT);                    // Allows us to pet the watchdog
  pinMode(hardResetPin,OUTPUT);               // For a hard reset - HIGH

  if(!connectToParticle()) digitalWrite(hardResetPin,HIGH);    // If connection fails, bounce the power
  Particle.publish("State","Initalizing");

  power.begin();
  power.disableWatchdog();
  power.disableDPDM();
  power.setInputVoltageLimit(4840);     //Set the lowest input voltage to 4.84 volts. This keeps my 5v solar panel from operating below 4.84 volts (defauly 4360)
  power.setInputCurrentLimit(900);     // default is 900mA
  power.setChargeCurrent(0,0,0,0,0,0); // default is 512mA
  power.setChargeVoltage(4112);        // default is 4.112V termination voltage
  power.enableDPDM();

  stateOfCharge = int(batteryMonitor.getSoC()); // Percentage of full charge

  attachInterrupt(wakeUpPin, watchdogISR, RISING);   // The watchdog timer will signal us and we have to respond
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
  Particle.function("Reset",resetNow);
  Particle.function("HardReset",hardResetNow);
  Particle.function("SleepInFive",sleepInFive);
  Particle.function("SendNow",sendNow);

  if (!fram.begin()) {                // you can stick the new i2c addr in here, e.g. begin(0x51);
    Particle.publish("Startup","No FRAM");
    state = ERROR_STATE;
  }

  if (FRAMread8(VERSIONADDR) != VERSIONNUMBER) {  // Check to see if the memory map in the sketch matches the data on the chip
    Particle.publish("Startup","FRAM mismatch - erasing");
    ResetFRAM();                        // Reset the FRAM to correct the issue
    if (FRAMread8(VERSIONADDR) != VERSIONNUMBER) state = ERROR_STATE;  // Resetting did not fix the issue
  }

  resetCount = FRAMread8(RESETCOUNT);         // Retrive system recount data from FRAMwrite8
  if (System.resetReason() == RESET_REASON_PIN_RESET)  // Check to see if we are starting from a pin reset
  {
    resetCount++;
    FRAMwrite8(RESETCOUNT,static_cast<uint8_t>(resetCount));          // If so, store incremented number - watchdog must have done This
  }

  Time.zone(-5);                              // Set time zone to Eastern USA daylight saving time
  takeMeasurements();
  StartStopTest(1);                           // Default action is for the test to be running
  timeTillSleep = sleepDelay;                 // Set initial delay for 60 seconds

  if (stateOfCharge <= 20)  state = SLEEPING_STATE;   // Low battery level will cause an ERROR and reset which brings us here
  else if (state != ERROR_STATE)                      // IDLE unless error from above code
  {
    state = IDLE_STATE;                                // Idle and look for events to change the state
    Particle.publish("State","Idle");
  }
}

void loop()
{
  switch(state) {
  case IDLE_STATE:
    if(hourlyPersonCountSent) {   // Cleared here as there could be counts coming in while "in Flight"
      hourlyPersonCount -= hourlyPersonCountSent;    // Confirmed that count was recevied - clearing
      FRAMwrite16(CURRENTHOURLYCOUNTADDR, static_cast<uint16_t>(hourlyPersonCount));  // Load Hourly Count to memory
      hourlyPersonCountSent = 0;
    }
    if (sensorDetect) recordCount();                                                  // The ISR had raised the sensor flag
    if (millis() >= (lastEvent + timeTillSleep)) state = NAPPING_STATE;               // Too long since last sensor flag - time to nap
    if (Time.hour() >= PARKCLOSES || Time.hour() < PARKOPENS) state = SLEEPING_STATE;  // The park is closed, time to sleep
    if (Time.hour() != currentHourlyPeriod && !readyForBed) state = REPORTING_STATE;   // We want to report on the hour but not after bedtime
    break;

  case SLEEPING_STATE: {                                        // This state is triggered once the park closes and runs until it opens
    if (!readyForBed)                                           // Only do these things once - at bedtime
    {
      if (Particle.connected()) {
        disconnectFromParticle();                               // If connected, we need to disconned and power down the modem
      }
      detachInterrupt(intPin);                                  // Done sensing for the day
      dailyPersonCount = 0;                                     // All the counts have been reported so time to zero everything
      FRAMwrite16(CURRENTDAILYCOUNTADDR, 0);                    // Reset the counts in FRAM as well
      resetCount = 0;
      FRAMwrite8(RESETCOUNT,resetCount);
      hourlyPersonCount = 0;
      FRAMwrite16(CURRENTHOURLYCOUNTADDR, 0);
      ledState = false;
      digitalWrite(blueLED,LOW);                                // Turn off the LED
      digitalWrite(tmp36Shutdwn, LOW);                          // Turns off the temp sensor
      digitalWrite(donePin,HIGH);
      digitalWrite(donePin,LOW);                                // Pet the watchdog
      readyForBed = true;                                       // Set the flag for the night
    }
    int secondsToHour = (60*(60 - Time.minute()));              // Time till the top of the hour
    System.sleep(SLEEP_MODE_DEEP,secondsToHour);        // Very deep sleep till the next hour (cellular, uC and Fuel Gauge off)
    digitalWrite(donePin,HIGH);
    digitalWrite(donePin,LOW);                                  // Pet the watchdog
    if (Time.hour() < PARKCLOSES && Time.hour() >= PARKOPENS)   // Time to wake up and go to work
    {
      state = IDLE_STATE;                                       // Go to IDLE state for more instructions once we awake
      lastEvent = millis();                                     // Reset millis so we don't wake and then nap again
      attachInterrupt(intPin,sensorISR,RISING);                 // Sensor interrupt from low to high
      currentDailyPeriod = Time.day();                          // Waking from a night's sleep - it is a new day
      digitalWrite(tmp36Shutdwn,HIGH);                          // Turn on the temp sensor
      readyForBed = false;
    }
    } break;

  case NAPPING_STATE: {
    if (Particle.connected())
    {
      disconnectFromParticle();                              // If connected, we need to disconned and power down the modem
      timeTillSleep = napDelay;                              // Set the time we will wait before napping again
    }
    ledState = false;                                        // Turn out the light
    digitalWrite(blueLED,LOW);                               // Turn off the LED
    sensorDetect = true;                                     // Woke up so there must have been an event
    lastEvent = millis();                                    // Reset millis so we don't wake and then nap again
    int secondsToHour = (60*(60 - Time.minute()));           // Time till the top of the hour
    System.sleep(intPin,RISING,secondsToHour);               // Sensor will wake us with an interrupt
    state = IDLE_STATE;                                      // Back to the IDLE_STATE after a nap
    } break;

  case REPORTING_STATE: {
    timeTillSleep = sleepDelay;                              // Sets the sleep delay to give time to flash if needed
    if (!connectToParticle()) {
      state = ERROR_STATE;
      break;
    }
    takeMeasurements();
    LogHourlyEvent();
    sendEvent();
    publishTimeStamp = millis();
    digitalWrite(donePin,HIGH);
    digitalWrite(donePin,LOW);                          // Pet the watchdog once an hour
    Particle.publish("State","Waiting for Response");
    state = RESP_WAIT_STATE;                            // Wait for Response
    } break;

  case RESP_WAIT_STATE:
    if (!dataInFlight)                                  // Response received
    {
      if (stateOfCharge <= 20) state = ERROR_STATE;     // Very low battery, time to reset then sleep
      else state = IDLE_STATE;                          // Battery OK, proceed
      Particle.publish("State","Idle");
    }
    else if (millis() >= (publishTimeStamp + webhookWaitTime)) state = ERROR_STATE;  // Response timed out
    break;

  case ERROR_STATE:                                          // To be enhanced - where we deal with errors
    if (!waiting)                                            // Will use this flag to wiat 30 seconds before reset
    {
      waiting = true;
      resetWaitTimeStamp = millis();
      Particle.publish("State","Error");
    }
    if (millis() >= (resetWaitTimeStamp + resetWaitTime)) System.reset();   // Today, only way out is reset
    break;
  }
}

void recordCount()                                          // Handles counting when the sensor triggers
{
  sensorDetect = false;                                     // Reset the flag
  if (digitalRead(intPin)) {
    Particle.publish("State","Counting");
    if (Time.minute() > 2) timeTillSleep = napDelay;        // reset the time we will wait before napping again
    lastEvent = millis();                                   // Important to keep from napping too soon
    t = Time.now();
    hourlyPersonCount++;                    // Increment the PersonCount
    FRAMwrite16(CURRENTHOURLYCOUNTADDR, static_cast<uint16_t>(hourlyPersonCount));  // Load Hourly Count to memory
    dailyPersonCount++;                    // Increment the PersonCount
    FRAMwrite16(CURRENTDAILYCOUNTADDR, static_cast<uint16_t>(dailyPersonCount));   // Load Daily Count to memory
    FRAMwrite32(CURRENTCOUNTSTIME, t);   // Write to FRAM - this is so we know when the last counts were saved
    ledState = !ledState;              // toggle the status of the LEDPIN:
    digitalWrite(blueLED, ledState);    // update the LED pin itself
  }
}

void StartStopTest(boolean startTest)  // Since the test can be started from the serial menu or the Simblee - created a function
{
 if (startTest) {
     inTest = true;
     currentHourlyPeriod = Time.hour();   // Sets the hour period for when the count starts (see #defines)
     currentDailyPeriod = Time.day();     // And the day  (see #defines)
     // Deterimine when the last counts were taken check when starting test to determine if we reload values or start counts over
     time_t unixTime = FRAMread32(CURRENTCOUNTSTIME);
     lastHour = Time.hour(unixTime);
     lastDate = Time.day(unixTime);
     dailyPersonCount = FRAMread16(CURRENTDAILYCOUNTADDR);  // Load Daily Count from memory
     hourlyPersonCount = FRAMread16(CURRENTHOURLYCOUNTADDR);  // Load Hourly Count from memory
     if (currentHourlyPeriod != lastHour) LogHourlyEvent();
 }
 else {
     inTest = false;
     t = Time.now();
     FRAMwrite16(CURRENTDAILYCOUNTADDR, static_cast<uint16_t>(dailyPersonCount));   // Load Daily Count to memory
     FRAMwrite16(CURRENTHOURLYCOUNTADDR, static_cast<uint16_t>(hourlyPersonCount));  // Load Hourly Count to memory
     FRAMwrite32(CURRENTCOUNTSTIME, t);   // Write to FRAM - this is so we know when the last counts were saved
     hourlyPersonCount = 0;        // Reset Person Count
     dailyPersonCount = 0;         // Reset Person Count
 }
}

void LogHourlyEvent() // Log Hourly Event()
{
  time_t LogTime = FRAMread32(CURRENTCOUNTSTIME);     // This is the last event recorded - this sets the hourly period
  unsigned int pointer = (HOURLYOFFSET + FRAMread16(HOURLYPOINTERADDR))*WORDSIZE;  // get the pointer from memory and add the offset
  LogTime -= (60*Time.minute(LogTime) + Time.second(LogTime)); // So, we need to subtract the minutes and seconds needed to take to the top of the hour
  FRAMwrite32(pointer, LogTime);   // Write to FRAM - this is the end of the period
  FRAMwrite16(pointer+HOURLYCOUNTOFFSET,static_cast<uint16_t>(hourlyPersonCount));
  FRAMwrite8(pointer+HOURLYBATTOFFSET,stateOfCharge);
  unsigned int newHourlyPointerAddr = (FRAMread16(HOURLYPOINTERADDR)+1) % HOURLYCOUNTNUMBER;  // This is where we "wrap" the count to stay in our memory space
  FRAMwrite16(HOURLYPOINTERADDR,newHourlyPointerAddr);
}

void sendEvent()
{
  char data[256];                                         // Store the date in this character array - not global
  snprintf(data, sizeof(data), "{\"hourly\":%i, \"daily\":%i,\"battery\":%i, \"temp\":%i, \"resets\":%i}",hourlyPersonCount, dailyPersonCount, stateOfCharge, temperatureF,resetCount);
  Particle.publish("Ubidots-Hook", data, PRIVATE);
  hourlyPersonCountSent = hourlyPersonCount; // This is the number that was sent to Ubidots - will be subtracted once we get confirmation
  currentHourlyPeriod = Time.hour();  // Change the time period
  dataInFlight = true; // set the data inflight flag
}

void UbidotsHandler(const char *event, const char *data)  // Looks at the response from Ubidots - Will reset Photon if no successful response
{
  // Response Template: "{{hourly.0.status_code}}"
  if (!data) {                                            // First check to see if there is any data
    Particle.publish("Ubidots Hook", "No Data");
    return;
  }
  int responseCode = atoi(data);                          // Response is only a single number thanks to Template
  if ((responseCode == 200) || (responseCode == 201))
  {
    Particle.publish("State","Response Received");
    dataInFlight = false;                                 // Data has been received
  }
  else Particle.publish("Ubidots Hook", data);             // Publish the response code
}

void getSignalStrength()
{
    CellularSignal sig = Cellular.RSSI();  // Prototype for Cellular Signal Montoring
    int rssi = sig.rssi;
    int strength = map(rssi, -131, -51, 0, 5);
    snprintf(Signal,17, "%s: %d", levels[strength], rssi);
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
  else return 0;
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
    FRAMwrite8(RESETCOUNT,0);          // If so, store incremented number - watchdog must have done This
    resetCount = 0;
    hourlyPersonCount = 0;                    // Reset count variables
    dailyPersonCount = 0;
    hourlyPersonCountSent = 0;                // In the off-chance there is data in flight
    dailyPersonCountSent = 0;

    dataInFlight = false;
    return 1;
  }
  else return 0;
}

int resetNow(String command)   // Will reset the Electron
{
  if (command == "1")
  {
    System.reset();
    return 1;
  }
  else return 0;
}

int hardResetNow(String command)   // Will perform a hard reset on the Electron
{
  if (command == "1")
  {
    digitalWrite(hardResetPin,HIGH);
    return 1;
  }
  else return 0;
}

int sleepInFive(String command)   // Will perform a hard reset on the Electron
{
  if (command == "1")
  {
    timeTillSleep = 300000;       // Set this equal to 5 minutes.  Will reset in the program once it goes to NAPPING_STATE
    return 1;
  }
  else return 0;
}


int sendNow(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    state = REPORTING_STATE;
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

void sensorISR()
{
  sensorDetect = true;                                      // sets the sensor flag for the main loop
}

void watchdogISR()
{
  digitalWrite(donePin, HIGH);                              // Pet the watchdog
  digitalWrite(donePin, LOW);
}

bool connectToParticle()
{
  if (!Cellular.ready())
  {
    Cellular.on();                                           // turn on the Modem
    Cellular.connect();                                      // Connect to the cellular network
    if(!waitFor(Cellular.ready,90000)) return false;         // Connect to cellular - give it 90 seconds
  }
  Particle.connect();                                      // Connect to Particle
  if(!waitFor(Particle.connected,30000)) return false;     // Connect to Particle - give it 30 seconds
  return true;
}

bool disconnectFromParticle()
{
  Particle.disconnect();                                   // Disconnect from Particle in prep for sleep
  waitFor(notConnected,10000);
  Cellular.disconnect();                                   // Disconnect from the cellular network
  delay(3000);
  Cellular.off();                                           // Turn off the cellular modem
  return true;
}

bool notConnected() {
  return !Particle.connected();                             // This is a requirement to use waitFor
}

void takeMeasurements() {
  getSignalStrength();                                      // Test signal strength at startup
  getTemperature();                                         // Get Temperature at startup as well
  stateOfCharge = int(batteryMonitor.getSoC());             // Percentage of full charge
}
