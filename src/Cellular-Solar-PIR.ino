/*
* Project Cellular-Solar-PIR
* Description: Cellular Connected Data Logger for Solar installations
* Author: Chip McClelland
* Date:8 October 2017
*/


// Easy place to change global numbers
#define SOFTWARERELEASENUMBER "0.15"
#define PARKCLOSES 22
#define PARKOPENS 7

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

  pinMode(intPin,INPUT);                      // PIR interrupt pinMode
  pinMode(blueLED, OUTPUT);                   // declare the Blue LED Pin as an output

  attachInterrupt(intPin,sensorISR,RISING);   // Will know when the PIR sensor is triggered

  Particle.variable("Release",releaseNumber); // Make sure we know what version of software is running
  Particle.variable("stateOfChg", stateOfCharge); // Track Battery Level

  Time.zone(-4);                              // Set time zone to Eastern USA daylight saving time

  //pmic.setChargeCurrent(0,0,1,0,0,0);         //Set charging current to 1024mA (512 + 512 offset) thank you @RWB for these two lines
  pmic.setInputVoltageLimit(4840);            //Set the lowest input voltage to 4.84 volts. This keeps my 5v solar panel from operating below 4.84 volts.
  stateOfCharge = int(batteryMonitor.getSoC()); // Percentage of full charg
  Serial.print("Charge current limit is: ");
  Serial.println(pmic.getChargeCurrent());

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

void getSignalStrength()
{
    CellularSignal sig = Cellular.RSSI();  // Prototype for Cellular Signal Montoring
    int rssi = sig.rssi;
    int strength = map(rssi, -131, -51, 0, 5);
    sprintf(Signal, "%s: %d", levels[strength], rssi);
}

void sensorISR()
{
  sensorDetect = true;                                      // sets the sensor flag for the main loop
}
