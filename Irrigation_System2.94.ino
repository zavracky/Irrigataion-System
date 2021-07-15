#define VERSION "2.94"
/* REVISION HISTORY
 *  Written by Paul M. Zavracky
 *  Borrows heavily from the work of zillions of others
 *  
 *  The irrigation system comprises a set of 1 to 8 1/4 12v water valves, a moisture sensor, an arduino, 
 *  an NRF24 Radio, and 8 relays.  The Arduino takes both the digital and analog signals from the 
 *  moisture sensor and uses this information to determine if watering is needed.  If so, the Arduino
 *  sends a signal to the relay board, actualing the appropriate relay.  The relay connects a 12V supply
 *  to the water valve.  The Arduino continues monitoring the moisture sensor.  Once a predetermined level
 *  is reached, the Arduino turns off the water flow.
 *  Uses Larry Bank's Bit Bang I2C library. You can find it here:
 *  https://github.com/bitbank2/BitBang_I2C
 */

// setup for MCP23008 port expander
// definitions for mcp23008
#define MCP23008_ADDR 0x23 //Address of MCP23008
#define IODIR 0x00 // I/O DIRECTION REGISTER 
#define IPOL 0x01 // INPUT POLARITY PORT REGISTER
#define GPINTEN 0x02 // INTERRUPT-ON-CHANGE PINS
#define DEFVAL 0x03 // DEFAULT VALUE REGISTER
#define INTCON 0x04 // INTERRUPT-ON-CHANGE CONTROL REGISTER
#define OCON 0x05 // I/O EXPANDER CONFIGURATION REGISTER
#define GPPU 0x06 // GPIO PULL-UP RESISTOR REGISTER
#define INTF 0x07 // INTERRUPT FLAG REGISTER
#define INTCAP 0x08 // INTERRUPT CAPTURED VALUE FOR PORT REGISTER
#define GPIO 0x09 // GENERAL PURPOSE I/O PORT REGISTER
#define OLAT 0x0A // OUTPUT LATCH REGISTER 0

// setup for MySensors
#define MY_NODE_ID 30
//#define MY_DEBUG // comment out this line to remove MySensors use of serial port!!!
#define SERIAL_TEST true // d0 and d1 can not be used if the serial bus is active!!!

#define MY_PARENT_NODE_ID 0
#define MY_PARENT_NODE_IS_STATIC

// Enable and select radio type attached 
#define MY_RADIO_RF24
#define MY_RF24_PA_LEVEL  RF24_PA_LOW

// Set this to the pin you connected the Water Leak Detector's analog pins to
// The Analog pins are used to sense the moisture level
#define AI0_PIN A0
#define AI1_PIN A2
#define AI2_PIN A1
#define AI3_PIN A3
#define AI4_PIN A4
#define AI5_PIN A5
#define AI6_PIN A6
#define AI7_PIN A7

// Arduino output pin setup
#define INTERRUPT_PIN 8 // For MCP23008, pins 8 - 5
#define RESET_PIN 7 
#define SDA_PIN 6
#define SCL_PIN 5
#define POWER_PIN 4 // Sensor Power Pin
#define FLOW_PIN 3 // Flow sensor input pin
#define MCP_PIN5 2 // IRQ for NF24 Radio
#define MCP_PIN6 1 // Currently unused. This pin and the one below will not work in SERIAL_MODE OR MY_DEBUG are true!!!
#define MCP_PIN7 0

// analog inputs
#define CHILD_ID_START_OF_AI 0
#define CHILD_ID_AI0 0
#define CHILD_ID_AI1 1
#define CHILD_ID_AI2 2
#define CHILD_ID_AI3 3
#define CHILD_ID_AI4 4 
#define CHILD_ID_AI5 5
#define CHILD_ID_AI6 6
#define CHILD_ID_AI7 7

// digital outputs (switches in HA)
#define CHILD_ID_START_OF_RELAYS 10
#define CHILD_ID_DO0 10 
#define CHILD_ID_DO1 11
#define CHILD_ID_DO2 12
#define CHILD_ID_DO3 13
#define CHILD_ID_DO4 14
#define CHILD_ID_DO5 15
#define CHILD_ID_DO6 16
#define CHILD_ID_DO7 17

//Auto/Manual Mode Switches
#define CHILD_ID_START_OF_AUTOS 20 
#define CHILD_ID_AutoMan0 20 
#define CHILD_ID_AutoMan1 21
#define CHILD_ID_AutoMan2 22
#define CHILD_ID_AutoMan3 23
#define CHILD_ID_AutoMan4 24
#define CHILD_ID_AutoMan5 25
#define CHILD_ID_AutoMan6 26
#define CHILD_ID_AutoMan7 27

#define CHILD_ID_FLOW 30
#define CHILD_ID_DAY_NIGHT 31 // day night sensor gets data from Home Assistant
#define CHILD_ID_LOW_FLOW 32 // low flow alarm indicator for HA - sends alarm, shuts down system
#define CHILD_ID_OPEN_SENSOR 33 // open sensor alarm indicator for HA - sends alarm, shuts down system
#define CHILD_ID_SAFEFLOW_SWITCH 34 // if there is an automatic shutdown (safeFlow = false), this switch let's the system restart (safeFlow = true)

// remeber that fully dry give 1023 reading
#define MAX_VALVE_TIME 5 // maximum time a valve can stay on in minutes. If moisture level hasn't dropped on next cycle, valve will
                          // turn on againg
#define VALVES_OFF_MOISTURE_MEASUREMENT_TIME 15 // Time in minutes between moisture sensor readings when no valves are open.
#define VALVES_ON_MOISTURE_MEASUREMENT_TIME 0.1 // Time in minutes between moisture sensor readings when a valve is open.

// remember that fully dry gives 1023 reading
#define TARGET_FULL_MOISTURE 200 // maximum expected reading for fully irrigated soil (soil is wet enough)
#define TARGET_MIN_MOISTURE 700 // minimum expected reading for fully irrigated soil (soil is too dry)
#define NUM_VALVES 6 // the total numbeer of valves (max 8)

#define RELAY_ON 1
#define RELAY_OFF 0
#define K 2.28 // 1380 pulses/liter found online.  Max Flow through 30' should be about 5 GPH or 20 liters/hour or 0.33 liters/min or .0055 liters per sec.  
               // That implies about 8 pulses per second.  Some report closer to 30 Hz?  Our unit should be milliliters per second, so K should be 1.38. 
               // For the sensor used in this project, the reported value K = 1.38 was too small.  Flow (pulsed based) yeilded 73, for 44 ml/sec.
               // In this code, flowRate = count/2K, where counts are made on both rising and falling edges.  Flow rate should equal flowRate/1.65, so
               // Knew = 1.28 * 1.65 or 2.28.

//#define Rseries 1000.00 // series resistance used in moisture measurement
// load libraries

#include <SPI.h>
#include <MySensors.h>
#include <BitBang_I2C.h>

// Start I2C
BBI2C bbi2c;  

// Variables Definitions
uint16_t moisture_Level[NUM_VALVES]; // moisture level is just the binary voltage reading at the analog input.  It's converted to % by Home Assistant.
                                     // Convertion equation: M% = (((((1000.0 / (924 / AnalogReading - 1)))/151.35)^(-0.27))*100)
                                     // where 924 is the maximum open circuit sensor reading.  (on next itteration will tie drive voltage to Vref!
uint16_t oldmoisture_Level[NUM_VALVES] = {0}; // initialize all old data to zero
uint16_t startMoistureLevel = 1024; // used for sensor testing

boolean newLoop = true; // to update HA after power outage
//unsigned long oldNoDataTime = 0; // used to check for no data to force communication with HA
unsigned long oldValveOnTime = 0; // used to control the amount of time a valve is open
unsigned long oldNoMoistureTime = 0; // minutes between moisture measurements when no valve is on

byte AI_pin[] = {AI0_PIN, AI1_PIN, AI2_PIN, AI3_PIN, AI4_PIN, AI5_PIN, AI6_PIN, AI7_PIN};
byte CHILD_ID_AI[] = {CHILD_ID_AI0, CHILD_ID_AI1, CHILD_ID_AI2, CHILD_ID_AI3, CHILD_ID_AI4, CHILD_ID_AI5, CHILD_ID_AI6, CHILD_ID_AI7};
byte CHILD_ID_DO[] = {CHILD_ID_DO0, CHILD_ID_DO1, CHILD_ID_DO2, CHILD_ID_DO3, CHILD_ID_DO4, CHILD_ID_DO5, CHILD_ID_DO6, CHILD_ID_DO7};
byte CHILD_ID_AutoMan[] = {CHILD_ID_AutoMan0, CHILD_ID_AutoMan1, CHILD_ID_AutoMan2, CHILD_ID_AutoMan3, 
     CHILD_ID_AutoMan4, CHILD_ID_AutoMan5, CHILD_ID_AutoMan6, CHILD_ID_AutoMan7};

bool autoMan_State[NUM_VALVES] = {false}; // man if false, auto if true
bool ack = 1; 
float flowRate = 0.0;
//float Rsoil = 0.0;
bool safeFlow = true;  // if flow measures to long, the system will set this flag false and prevent valves from being actuated (prevent over heating)
bool lowFlow = false; // if the flow is too low, the system sets this flag and shuts down.  HA can see this flag.
bool openSensor = false; // if a sensor is open and doesn't responde to irrigation, the system sets this flag and shuts down.  HA can see this flag.
byte flowTestCount = 0; // keeps track of the number of zero flow measurements made - system shuts down after NUM_FLOW_TESTS
bool dayNight = false; // if true, it is daytime and the system should not water plants
uint8_t bitValue[] = {1,2,4,8,16,32,64,128};

// MySensors messages
MyMessage msgAI[NUM_VALVES]; // these messages tell HA the moisture level
MyMessage msgGPIO[NUM_VALVES]; // these messages let HA know what relays
MyMessage msgAutoMan[NUM_VALVES]; // these messages let HA know what relays
MyMessage msgFlow(CHILD_ID_FLOW, V_FLOW);
MyMessage msgDayNight(CHILD_ID_DAY_NIGHT, V_TRIPPED); // used to prevent watering during the day
MyMessage msgLowFlow(CHILD_ID_LOW_FLOW, V_TRIPPED); // used to let HA know that we have no water flow
MyMessage msgOpenSensor(CHILD_ID_OPEN_SENSOR, V_TRIPPED); // used to let HA know that we may have an open sensor
MyMessage msgRestart(CHILD_ID_SAFEFLOW_SWITCH, V_STATUS); // safeMode switch, shut down due to system failure, restart through HA

void setup(){
  // setup MCP23008 for output
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, 1); // reset pin on mcp23008
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, true); // turn off sensor power
  pinMode(INTERRUPT_PIN, INPUT);  
  
  //Setup inputs and outputs
    for (int i = 0; i < NUM_VALVES; i++) {
      pinMode(AI_pin[i], INPUT);
    }
    pinMode(FLOW_PIN, INPUT_PULLUP);  // flow sensor requires the input have a pullup resistor
    
  // initialize messages
  for (int i = 0; i < NUM_VALVES; i++) {
    msgAI[i].sensor = i; // Analog sensors start at child ID 0
    msgAI[i].type = V_LEVEL; // these are the moisture sensors
    msgGPIO[i].sensor = i + 10; // Relays start at child ID 10
    msgGPIO[i].type = V_STATUS; // these are the sprinkler valves
    msgAutoMan[i].sensor = i + 20; // AutoMan switches start at child ID 20
    msgAutoMan[i].type = V_STATUS; // these are the switch to set mode
  }
  
  // I2C initialization
  memset(&bbi2c, 0, sizeof(bbi2c));
  bbi2c.bWire = 0; // use bit bang, not wire library
  bbi2c.iSDA = SDA_PIN;
  bbi2c.iSCL = SCL_PIN;
  I2CInit(&bbi2c, 100000L);
  delay(100); // allow devices to power up

  // I2C ready to go, set up for output
  writeRegister(IODIR, 0); // make all GPIO pins outputs
  writeRegister(IPOL, 0); // make all GPIO pins not inverted
  writeRegister(GPIO, 255); // turn all bits on, all valves off (valves are active low!)
}

void presentation()
{ 
  // Send the sketch version information to the gateway
  sendSketchInfo("IRRIGATION SYSTEM ", VERSION);
  Serial.print("Irrigation System, 2021, Version ");Serial.println(VERSION);  // this line will identify version even when MySensor Debug is turned off.

  // Register all sensors to gw (they will be created as child devices)
  for (int i = 0; i < NUM_VALVES; i++) {
    present(CHILD_ID_AI[i], S_MOISTURE);
    present(CHILD_ID_DO[i], S_SPRINKLER);
    present(CHILD_ID_AutoMan[i], S_LIGHT);
  }
  present(CHILD_ID_FLOW, S_WATER, "Water Flow Rate");
  present(CHILD_ID_DAY_NIGHT, S_DOOR); // used to prevent watering during the day - binary sensor
  present(CHILD_ID_LOW_FLOW, S_DOOR); // used to let HA know that we have no water flow - binary sensor
  present(CHILD_ID_OPEN_SENSOR, S_DOOR); // used to let HA know that we have an open sensor - binary sensor
  present(CHILD_ID_SAFEFLOW_SWITCH, S_LIGHT); // if off, system is idle, if one, system is active

  send(msgRestart.set(false)); 
}

void loop()    
{   
  if (newLoop) { updateAll(); newLoop = false; } // make sure HA is aware of current state after powerdown
                                                 // and collect moisture levels
  if (!dayNight) { // if !dayNight = true, it's night time and good to go
    if (safeFlow) {
      // check time to see if we need a moisture reading - different delay during watering
      if (millis() < oldNoMoistureTime) { oldNoMoistureTime = 0; }  // millis() can roll over making it smaller than oldNoDataTime
      if ((millis()- oldNoMoistureTime) > ((isAnyValveOn()?1:0)*VALVES_ON_MOISTURE_MEASUREMENT_TIME + 
                                          (isAnyValveOn()?0:1)*VALVES_OFF_MOISTURE_MEASUREMENT_TIME) * 60000) { // send data to HA to let it know where still alive
        oldNoMoistureTime = millis();  // update time
        newLoop = true;  // forces an update to HA
        readMoisture();  // reads all sensors
        flowRate = flowMeasurement(); // this is not the only place where a flowrate measurement is made; check getLastFlowRate() - used when closing valves
      } 
      // open and close valves automatically when set by autoMan_State and send moisture readings to HA when appropriate
      // valves are turned on and off manually through the receive() function!!
      for (int i = 0; i < NUM_VALVES; i++) {
        if (abs(moisture_Level[i] - oldmoisture_Level[i]) > 50 ) { // only send data upon change of state
          send(msgAI[i].set(moisture_Level[i],1)); // will automatically send all moisture_levels on first newLoop
          oldmoisture_Level[i] = moisture_Level[i];
        }
        if (autoMan_State[i] == true) { // must be in auto mode to change valve state
          if ((moisture_Level[i] < TARGET_FULL_MOISTURE) && valveState(i) == true) {
            writeRegister(GPIO, 255); // turn all bits on, all valves off (valves are active low!)
            send(msgGPIO[i].set(valveState(i)));
            getLastFlowRate();  // valves are closed, make sure flow has stopped
          }
          if ((moisture_Level[i] > TARGET_MIN_MOISTURE) && !isAnyValveOn()) { // only turn on if all other valves are off
            writeNotGPIO(i, true); // turn on water (valves are active low, but writeNotGPIO does inversion)
            send(msgGPIO[i].set(valveState(i)));
            startMoistureLevel = moisture_Level[i]; // save for sensor check below
            oldValveOnTime = millis();  // set start time for valve[i] on duration.  only one valve can be on at a time, so we need only one oldValveOnTime!           
          }
        }  
      }
      // Check flow and moisture response if valve is on
      if (isAnyValveOn() == true) { // one of the valves is on, let's check to see if moisture sensor and the flow meter are responding
        if (millis() < oldValveOnTime) { oldValveOnTime = 0; }  // millis() can roll over making it smaller than oldNoDataTime            
        if ((millis()- oldValveOnTime) > MAX_VALVE_TIME * 60000) { // time to turn off the valve to prevent over watering
          if (flowRate < 5.0) { // oops low flow
            lowFlow = true;
            send(msgOpenSensor.set(true)); // let HA know the system is shutting down due to an open sensor
            safeFlowShutdown();
          }
          else if(startMoistureLevel >= oldmoisture_Level[whichValveIsOn()] ) { // if the measured moisture level hasn't dropped
            openSensor = true;
            send(msgLowFlow.set(true)); // send alarm to HA to indicate system shut down.             
            safeFlowShutdown(); // shutdown system and let HA know valves are all off 
          }
        }
      }
      else { oldValveOnTime = millis(); } // resets every loop cycle if no valve is on
    }
  }
}

void receive(const MyMessage &message) {
  byte relayPinIndex;
  
  if (message.isAck()) {
     // if (SERIAL_TEST) {Serial.println("This is an ack from gateway");}
  }
  if (message.type == V_STATUS) { // possible sensors are:
                                  //  Digital IO -> 10 to 17
                                  //  AutoMan Switches -> 20 - 27
                                  //  SafeFlow Switch -> 34
    if (message.sensor < 18 && message.sensor > 9) { // must be a relay
      // Change relay state
      // Calculate relay from sensor number
      relayPinIndex = message.sensor - CHILD_ID_START_OF_RELAYS; // relayPinIndex[0] thru relayPinIndex[7]
      if (!autoMan_State[relayPinIndex] && safeFlow) { // only change relay state if in manual mode and safeFlow
        writeNotGPIO(relayPinIndex, message.getBool()); // turns off all valves and sets state of valve(relayPinIndex) true or false
        if(!message.getBool()) getLastFlowRate(); // make sure we get a flow measurement after valves are turned off
        for (int i = 0; i < NUM_VALVES; i++) { // set all valveStates except valveState(relayPinIndx) to false.
            send(msgGPIO[i].set(valveState(i)));
        }
      }
      else if(!safeFlow) {
        for (int i = 0; i < NUM_VALVES; i++) { // turn off all valves
          writeNotGPIO(i, false); // turn off all valves
          send(msgGPIO[i].set(false)); // don't know why this is necessary
        }
        getLastFlowRate(); // make sure we get a flow measurement after valves are turned off
      }
    }
    else if (message.sensor < 28 && message.sensor > 19){ // must be an AutoMan Switch
      autoMan_State[message.sensor - 20] = message.getBool(); // this means state was changed by HASS
    }
    else if (message.sensor == 34) { // HA requesting SafeFlow reset
      if(message.getBool() == true) {
        safeFlow = true; // user can only turn switch on.  here it's turned off again after one second.
        lowFlow = false;
        openSensor = false;
        delay(1000);
        send(msgRestart.set(false)); // turn off the HA switch
        oldValveOnTime = millis(); 
      }    
    }
  }
  if (message.type == V_TRIPPED) { // recieved dayNight state from HA
    if (message.sensor == 31) { // this is the day night switch that HA sends to prevent daytime watering
        dayNight = message.getBool(); // if true, it's daytime, do not water
    }
  }
}

// Update All
void updateAll()
{
  digitalWrite(POWER_PIN, false); // turn off sensor power
  for (int i = 0; i < NUM_VALVES; i++) {
    send(msgGPIO[i].set(valveState(i)));
    send(msgAI[i].set(moisture_Level[i],1));
    oldmoisture_Level[i] = moisture_Level[i];
    send(msgAutoMan[i].set(autoMan_State[i]));
  }
  send(msgFlow.set(flowRate,2));
  send(msgDayNight.set(dayNight)); // used to prevent watering during the day
  send(msgLowFlow.set(lowFlow)); // used to let HA know that we have no water flow
  send(msgOpenSensor.set(openSensor)); // used to let HA know that we may have an open sensor
}

bool isAnyValveOn() { 
  bool result = true;

  if(readRegister(GPIO) == 255 ) {result = false;}
  // Serial.print("isAny = ");Serial.println(result);Serial.print("GPIO = ");Serial.println(readRegister(GPIO));
  return(result);
}

bool valveState(uint8_t whichBit) {
  return(!bitRead(readRegister(GPIO), whichBit));
}

byte whichValveIsOn() {
  for (byte i = 0; i < NUM_VALVES; i++) {
    if(valveState(i)) {return(i);}
  }
}

void readMoisture() {
  // read all the moisture levels, but don't send to HA
  digitalWrite(POWER_PIN, true); // Turn on sensor power; powers all sensors; active low
  delay(10); //delay 10 milliseconds to allow reading to settle - determined by testing
  for (int i = 0; i < NUM_VALVES; i++) { // read all sensors
    moisture_Level[i] = analogRead(AI_pin[i]);  // update moisture levels
  } 
}

float flowMeasurement() { // take a second to check flow rate
                          // this is a crude method used here because there are no interrupt pins left
                          // in this implementation.  At the max reported flow rate through 1/4" drip
                          // tubing 0f 20 GPH, we can anticipate at pulse frequency of 30 Hz.  That's
                          // slow enough to make this routine work well.
 
  bool state = digitalRead(FLOW_PIN);
  bool oldState = state;
  bool notDone = true;
  unsigned long int count = 0;
  unsigned long int oldMillis = millis();
  
  while (notDone) {
    state = digitalRead(FLOW_PIN);
    if (state != oldState) {
      oldState = state;
      count++;      
    }
    if( oldMillis > millis()) {oldMillis = 0;} // rollover event
    notDone = (millis() - oldMillis < 1000); // 1000 = 1 sec: when this is false, we are done
  }
  flowRate = count/(2*K); // gets two counts per square wave; but rising and falling!
  return(flowRate); // K is the pulses per liter (1380), so we are returning the flow rate in liters per second
                   // Should be 0.022 liters per second or there abouts.
}

void getLastFlowRate() {
  delay(2000); // delay to let valve close and flow rate to settle
  flowRate = flowMeasurement(); // this takes one second to execute
  send(msgFlow.set(flowRate, 2)); // Update HA after valves have closed.  Should be zero, but it no, there's a problem.
}

uint8_t readRegister(uint8_t regAddr) {
  uint8_t Data;
  uint8_t *pu8Data = &Data;
  
  I2CReadRegister(&bbi2c, MCP23008_ADDR, regAddr, pu8Data, 1);
  return(Data);
}

void writeRegister(uint8_t regAddr, uint8_t data) {
  uint8_t Data[2] = {regAddr, data};
  uint8_t *pu8Data;
  
  pu8Data = &Data[0];
  I2CWrite(&bbi2c, MCP23008_ADDR, pu8Data, 2); // write both the register addr and the data
}

void writeNotGPIO(uint8_t pin, bool state) { // set particular pin true or false (only one bit true at a time!!!
  uint8_t newState;

  writeRegister(GPIO, 255); // turn all bits on, all valves off
  if(state) {
    newState = ~bitValue[pin]; // inverts the binary number
    writeRegister(GPIO, newState);
  }
}

void safeFlowShutdown() {
  safeFlow = false; // system shuts down until attended to.  When repaired, turn system off and on again.
  writeRegister(GPIO, 255); // turn all bits on, all valves off (valves are active low!)
  for (int i = 0; i < NUM_VALVES; i++) {
    send(msgGPIO[i].set(valveState(i)));
  }
  getLastFlowRate(); // make sure flow has shut down.
  send(msgRestart.set(false)); // let HA know that we have a problem - this is a switch in HA
}
