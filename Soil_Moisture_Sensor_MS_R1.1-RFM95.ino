
/*  
 *  Soil Moisture Sensor
 *    4 channels
 *    Temperature + Humidity 
 *    1 Analog In
 *    1 Flow Counter In
 *    DS3232 RTC + Temperature
 *    DS18B20 Soil Sensor
 *  
 *  Water from a pressure transducer, 0 to 5v
 *    Input from sensor is scaled by .6577, 5.0v * .6577 = 3.288v
 *    (10.2k and a 19.6k resistor, filtered with a .1uf cap)
 *   
 *  Output: 0.5V – 4.5V linear voltage output. 0 psi outputs 0.5V, 50 psi outputs 2.5V, 100 psi outputs 4.5V 
 *    0   psi = 0.33v after scaling 5.0v to 3.3v
 *    50  psi = 1.65v
 *    100 psi = 2.97v
 *    3.3v/1023 = .003225806 volt per bit or 3.3V/4095 = .000805861 volts per bit
 *    
 *
 *    Max6816 is used as an option on Rev2a board to de-bounce the flow reed switch on the meter
 *    The switch must be stable for 40ms to get an output, this limits the max
 *    rate this device can support to less than 20Hz or so. (1,200 gpm)
 *
 *
 * CHANGE LOG:
 *
 *  DATE         REV  DESCRIPTION
 *  -----------  ---  ----------------------------------------------------------
 *  22-Dec-2016 1.0a  TRL - first 
 *  27-Dec-2016 1.1a  TRL - Added DS3232 and Moisture Mux code
 *  31-Dec-2016 1.1a  TRL - Changed freq to 928.5MHz
 *  04-Jan-2017 1.1b  TRL - Adding Sleep and (WDT code ??)
 *  07-Jan-2017 1.1c  TRL - Adding Soil Temp Sensor
 *  14-feb-2017 1.1d  TRL - Adding time of day request
 *  02-Mar-2017 1.1e  TRL - Added KeepAwake code and message
 *
 *  Notes:  1)  Tested with Arduino 1.8.1
 *          2)  Testing using RocketStream M0 with RFM95 & RFM95T
 *          3)  Sensor 2 board, Rev2a 
 *          4)  MySensor 2.1.1 30 Dec 2016
 *    
 *    
 *    MCP9800   base I2C address 0x40
 *    Si7021    base I2C address 0x48
 *    DS3231    base I2C address 0x68
 *    EUI64     base I2C address 0x50
 *    
 *    DS18B20   1Wire address 0x44?
 *    
 *    TODO:   done --> RFM95 and MySensor in sleep mode
 *            Flash in sleep mode
 *            done --> Sleep Mode
 *            WDT, issue is complicated due to sleep and MySensor
 *            done --> Set correct alarms time and functions
 *            done --> Send current time to board
 *            done --> DS18B20 Soil temp sensor
 *            done --> schedule update from controller
 *            done --> added KeepAwake code and message
 *    
 *    Based on the work of: Reinier van der Lee, www.vanderleevineyard.com
 */
/* ************************************************************************************** */

// Define's for the board options
#define R2a                     // Rev 2a of the board
//#define IDsize2                 // if using only ID0 and ID1. ID2 --> D6 is needed for LoRaWan code base on TTN with RFM95

// Select the Temperature and/or Humidity sensor on the board
//#define Sensor_SI7021         // if using the Si7021 Temp and Humidity sensor
//#define Sensor_MCP9800        // if using the MCP9800 temp sensor
//#define WaterPressure         // if we have a water pressure sensor
//#define WaterMeter            // if we have a water flow meter
#define MyDS18B20               // if using a Soil Temp sensor
//#define MyWDT                 // if using the Watch Dog Timer
#define MoistureSensor          // if using the Moisture Sensor, 4 channels
#define MyDS3231                // if using the DS3231 RTC
//#define SendHeartbeat           // if sending the heartbeat message
#define MY_SERIALDEVICE Serial  // this will override Serial port define in MyHwSAMD.h file

/* ************************************************************************************** */
#include <Arduino.h>
#include <Wire.h>
#include "LowPower.h"

#if defined MyDS3231
    #include <DS3231.h>
    #define DS3231Int 38            // Interrupt pin
#endif

#if defined Sensor_SI7021
    #include "i2c_SI7021.h"
    SI7021 si7021;
#endif

#if defined Sensor_MCP9800
    #include <MCP980X.h>            // http://github.com/JChristensen/MCP980X
    MCP980X MCP9800(0);
#endif

#if defined MyDS18B20
    #include <OneWire.h>
    #include <DallasTemperature.h>
    #define ONE_WIRE_BUS 18         // This is shared with AIN0 port
    OneWire oneWire(ONE_WIRE_BUS);
    DallasTemperature SoilTemp(&oneWire);
#endif

#if defined MyWDT
  // #include <avr/wdt.h>              // for watch-dog timer support
#endif

#if defined MoistureSensor
  #include <math.h>                 // Conversion equation from resistance to %
#endif 

/* *********************************************************************************************** 
 * A bit array to define the hour to wake up and send a sensor messages...
 *  (NodeID * 5 %) 60 is use for the alarm time within the hour, this offset TX time for each node
 ************************************************************************************************ */
 // we always take a sensor reading at MySendTime[0] --> or midnight
 //                    0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23
bool MySendTime[24] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
#define TXoffset 5            // used to set a TX offset so each node TX's at a different time

/* ********************************************************************************************** */
// Most of these items below need to be prior to #include <MySensor.h> 

/*  Enable debug prints to serial monitor on port 0 */
#define MY_DEBUG            // used by MySensor
//#define MY_SPECIAL_DEBUG
//#define MY_DEBUG_VERBOSE_RFM95 
#define MY_DEBUG1           // used in this program, level 1 debug
#define MY_DEBUG2           // used in this program, level 2 debug

#define SKETCHNAME      "Soil Moisture Sensor"
#define SKETCHVERSION   "1.1e"

/* ************************************************************************************** */
/* Enable and select radio type attached, coding rate and frequency
 *  
 *   Pre-defined MySensor radio config's
 *   
 * | CONFIG           | REG_1D | REG_1E | REG_26 | BW    | CR  | SF   | Comment
 * |------------------|--------|--------|--------|-------|-----|------|-----------------------------
 * | BW125CR45SF128   | 0x72   | 0x74   | 0x04   | 125   | 4/5 | 128  | Default, medium range SF7
 * | BW500CR45SF128   | 0x92   | 0x74   | 0x04   | 500   | 4/5 | 128  | Fast, short range     SF7
 * | BW31_25CR48SF512 | 0x48   | 0x94   | 0x04   | 31.25 | 4/8 | 512  | Slow, long range      SF9
 * | BW125CR48SF4096  | 0x78   | 0xC4   | 0x0C   | 125   | 4/8 | 4096 | Slow, long range      SF12
 */

#define MY_RADIO_RFM95
#define MY_RFM95_MODEM_CONFIGRUATION    RFM95_BW125CR45SF128
#define MY_RFM95_TX_POWER               23 // max is 23
//#define MY_RF95_TCXO                    // If using an RFM95 with a TCXO
//#define MY_RFM95_ATC_MODE_DISABLED
#define MY_RFM95_ATC_TARGET_RSSI        (-60)
#define MY_RFM95_FREQUENCY              (928.5f)

#define SendDelay                       250       // this is the delay after each send
#define AckFlag                         false     // if we are requesting an ACK from GW

/* ************************************************************************************** */
// Select correct defaults for the processor and board we are using
#ifdef __SAMD21G18A__                             // Using an ARM M0 Processor, Zero, Feather M0, RocketScream Mini Pro

//#define MY_RFM95_RST_PIN        0
#define MY_RFM95_IRQ_PIN          2               // IRQ
#define MY_RFM95_SPI_CS           5               // NSS
#define MY_DEFAULT_TX_LED_PIN     12              // Led's on the board
#define MY_DEFAULT_ERR_LED_PIN    10
#define MY_DEFAULT_RX_LED_PIN     11
#define OnBoardLed                13              // CPU Led
#define MY_WITH_LEDS_BLINKING_INVERSE
#define MY_OTA_FLASH_SS            4              // Flash chip on processor
#define MY_OTA_FLASH_JDECID       0x1F65          // Rocket M0?? 0x0101??

#define ID0                        0              // ID pin
#define ID1                        1
#define ID2                        6

#else
  #error ********* Processor not defined, Requires an ARM M0 SAMD21G18A
#endif
/* ************************************************************************************** */

//#define MY_REPEATER_FEATURE

/* ************************************************************************************** */
// Set node defaults
#define NodeID_Base          20         // My Node ID base... this plus IDx bits
int myNodeID =                0;        // Set at run time from jumpers on PCB
#define MY_NODE_ID myNodeID             // Set at run time from jumpers

#define MY_PARENT_NODE_ID     0         // GW ID

#define CHILD_ID0             0         // Id of my Sensor
#define CHILD_ID1             1         // Id of my Water sensor child
#define CHILD_ID2             2         // Id of my 2nd sensor child
#define CHILD_ID3             3         // Id of my 3rd sensor child
#define CHILD_ID4             4         // Id of my 4th sensor child

/* ************************************************************************************** */
/* These are use for local debug of code, hwDebugPrint is defined in MyHwATMega328.cpp */
#ifdef MY_DEBUG1
#define debug1(x,...) hwDebugPrint(x, ##__VA_ARGS__)
#else
#define debug1(x,...)
#endif

#ifdef MY_DEBUG2
#define debug2(x,...) hwDebugPrint(x, ##__VA_ARGS__)
#else
#define debug2(x,...)
#endif


/* ************************************************************************************** */
// All #define above need to be prior to the #include <MySensors.h> below
#include <MySensors.h>
/* ************************************************************************************** */
#include <core/MyTransport.h>

/* ************************************************************************************** */
#define PressPin      A4                            // Pressure sensor is on analog input, 0 to 100psi
#define BattVolt      A5                            // Battery Voltage on pin A5.  270k/270k divider = 1/2,  6.6v max

/* ************************************************************************************** */

MyMessage LEVEL1         (CHILD_ID1,V_LEVEL);       // 37 0x25      // Send Water Saturation Ch 1
MyMessage LEVEL2         (CHILD_ID2,V_LEVEL);       // 37 0x25      // Send Water Saturation Ch 2
MyMessage LEVEL3         (CHILD_ID3,V_LEVEL);       // 37 0x25      // Send Water Saturation Ch 3
MyMessage LEVEL4         (CHILD_ID4,V_LEVEL);       // 37 0x25      // Send Water Saturation Ch 4

MyMessage IMP1           (CHILD_ID1, V_IMPEDANCE);  // 14 0x0E      // Send Resistance Ch 1
MyMessage IMP2           (CHILD_ID2, V_IMPEDANCE);  // 14 0x0E      // Send Resistance Ch 2
MyMessage IMP3           (CHILD_ID3, V_IMPEDANCE);  // 14 0x0E      // Send Resistance Ch 3
MyMessage IMP4           (CHILD_ID4, V_IMPEDANCE);  // 14 0x0E      // Send Resistance Ch 4

MyMessage VBAT           (CHILD_ID0, V_VOLTAGE);    // 38 0x26      // Send Battery Voltage

MyMessage PressureMsg    (CHILD_ID0,V_PRESSURE);    // 04 0x04      // Send current Water Pressure
MyMessage TextMsg        (CHILD_ID0,V_TEXT);        // 47 0x2F      // Send status Messages
MyMessage HumMsg         (CHILD_ID0,V_HUM);         // 01 0x01      // Send current Humidity 
MyMessage TempMsg        (CHILD_ID0,V_TEMP);        // 00 0x00      // Send current Air Temperature
MyMessage ScheduleUpdate (CHILD_ID0,V_VAR1);        // 24 0x18      // Request a schedule update from controller

MyMessage SoilTempMsg    (CHILD_ID1,V_TEMP);        // 00 0x00      // Send current Soil Temperature
 

/* ************************************************************************************** */

#if defined MyDS3231
  DS3231 clock;
  RTCDateTime dt;
  boolean isAlarm = false;
  boolean alarmState = false;
  boolean alarmUpdateFlag = true;         // Its time to request an update for RTC if true
#endif

/* *************************** Forward Declaration ************************************* */
void SendPressure();
void SendKeepAlive();
void getTempSi7021();
void getTempMCP9800 ();
void receive(const MyMessage &message);
void receiveTime(unsigned long ts);
void hexdump(unsigned char *buffer, unsigned long index, unsigned long width);
int  GetMoisture(unsigned long read);
void soilsensors();
void measureSensor();
void addReading(long resistance);
long average();
void printCpuResetCause();
void systemSleep();
void systemWakeUp();

/* ************************************************************************************** */
unsigned long SEND_FREQUENCY        = 15000;      // Minimum time between send (in milliseconds). We don't want to spam the gateway.
unsigned long KEEPALIVE_FREQUENCY   = 120000;     // Send Keep Alive message at this rate

unsigned long currentTime         = 0;
unsigned long lastSendTime        = 0;
unsigned long keepaliveTime       = 0;

#define StayAwakeTime 5000                        // Stay Awake time before sleeping

bool KeepAwakeFlag = false;
#define KeepAliveID      0x5aa5
      
int pressure      = 0;                            // Current value from ATD
float PSI         = 0;                            // Current PSI
float PSI_CAL     = 2.0;                          // Calibration of sensor

int floatMSB      = 0;                            // used to convert float to int for printing
int floatR        = 0;

static float humi, temp;                          // used by Si7021, MCP9800, DS3231

/* ************************************************************************************** */

/* Pin assigments for Moisture Mux */
#define SensDY  15
#define SensDX  17
#define SensAY  A0  
#define SensAX  A2  
#define MuxA     8
#define MuxB     7
#define MuxINH   9

typedef struct {                  // Structure to be used in percentage and resistance values matrix to be filtered (have to be in pairs)
  int moisture;
  unsigned long resistance;
} values;

// Setting up format for reading 4 soil sensors
#define NUM_READS 12              // Number of sensor reads for filtering

const long knownResistor = 4700;  // Value of reference resistors in ohms, = reference for sensor

unsigned long supplyVoltage;      // Measured supply voltage
unsigned long sensorVoltage;      // Measured sensor voltage
int zeroCalibration = 138;        // calibrate sensor resistance to zero when input is short circuited
                                  // basically this is compensating for the mux switch resistance

values valueOf[NUM_READS];        // Calculated  resistances to be averaged
long buffer[NUM_READS];
int index2 = 0;
int i=0;                            // Simple index variable
int j=0;                          // Simple index variable

long resistance = 0;
long moisture =   0;

unsigned long read1 = 0;
unsigned long read2 = 0;
unsigned long read3 = 0;
unsigned long read4 = 0;

 
/* **************************************************************************** */
/*                                Before                                        */
/* **************************************************************************** */
 // Before is part of MySensor core 
void before() 
{ 
     debug1(PSTR("*** In before ***\n"));
 
 // need to set up pins prior to reading them...
     pinMode(ID0, INPUT_PULLUP);
     pinMode(ID1, INPUT_PULLUP);
     pinMode(ID2, INPUT_PULLUP);
     
     myNodeID  = !digitalRead (ID0);                     // ID bit are 0 = on, so we invert them
     myNodeID |= (!digitalRead(ID1) << 1);
    #ifndef IDsize2                                      // using only ID0 and ID1
      myNodeID |= (!digitalRead(ID2) << 2);
    #endif
     myNodeID += NodeID_Base;                            // set our node ID

    // Pin for onboard LED
    pinMode(OnBoardLed, OUTPUT);
    digitalWrite(OnBoardLed, LOW);

     // We no longer need these pins, so remove pullups to save power
     pinMode(ID0, INPUT);
     pinMode(ID1, INPUT);
     pinMode(ID2, INPUT);
}



/* **************************************************************************** */
/*                            Setup                                             */
/* **************************************************************************** */
void setup()  
{  
 //   wdt_enable(WDTO_8S);                // lets set WDT in case we have a problem...
      
    //send(TextMsg.set("Starting"), AckFlag);  wait(SendDelay);
    debug1(PSTR("*** In Setup ***\n"));
  
    // de-select on board Flash 
    pinMode(MY_OTA_FLASH_SS, OUTPUT);
    digitalWrite(MY_OTA_FLASH_SS, HIGH);

    // Pin for DS3231 alarm interrupt
    pinMode(38, INPUT_PULLUP);


/* ************************************************************************************** */
  const char compile_file[]  = __FILE__ ;
  debug1(PSTR(" %s %s\n"), SKETCHNAME, SKETCHVERSION);
  debug1(PSTR(" %s \n"), compile_file);
  
  const char compile_date[]  = __DATE__ ", " __TIME__;
  debug1(PSTR(" %s \n\n"), compile_date);
  debug1(PSTR(" My Node ID: %u\n\n"), myNodeID);

  printCpuResetCause();                         // this will tell us what causes CPU reset  
  
  // set up ATD and reference, for ATD to use:
  // options --> AR_DEFAULT, AR_INTERNAL, AR_EXTERNAL, AR_INTERNAL1V0, AR_INTERNAL1V65, AR_INTERNAL2V23
  
      analogReference(AR_DEFAULT);              // AR_DEFAULT is set to VDD -> +3.3v, AR_EXTERNAL is set to external pin on chip
      analogReadResolution(12);                 // we want 12 bits

#if defined Sensor_SI7021
    si7021.initialize();
#endif

#if defined Sensor_MCP9800
    MCP9800.writeConfig(ADC_RES_12BITS);       // max resolution, 0.0625 °C
#endif

#if defined MoistureSensor
  // setting up the sensor interface
  // initialize digital pins SensDX, SensDY as an high impedance input.
  // Pin SensDX,SensDY are for driving the soil moisture sensor
  pinMode(SensDX, INPUT);    
  pinMode(SensDY, INPUT);
   
  // Pin MuxINH is for enabling Mux switches
  pinMode(MuxINH, OUTPUT);

  // Pin MuxA,MuxB are for selecting sensor 1-4
  pinMode(MuxA, OUTPUT);  // Mux input A
  pinMode(MuxB, OUTPUT);  // Mux input B 
#endif 

/* ******** This setup the DS3231 and its alarms ********* */
#if defined MyDS3231
  clock.begin();
  clock.enableOutput(false);                                    // set for interrupt on DS3231
  
  // Disarm alarms and clear alarms
  // Under normal conditions, the settings should be reset after power and restart microcontroller.
  clock.armAlarm1(false);
  clock.armAlarm2(false);
  clock.clearAlarm1();
  clock.clearAlarm2();

  // Set sketch compiling time to the DS3231
  debug1(PSTR("*** Setting Time on DS3231 \n"));
  clock.setDateTime(__DATE__, __TIME__);

/* ************** This sets our Alarms... **************** */

/* Alarm 1 is set to wake us up once a week to request time updated for RTC 
   It will set a flag, but wait until its time to send a normal sensor message to request time */
  // Set Alarm1 - At  (myNodeID*TXoffset%60) second pass the minute    // <--- for testing
  // setAlarm1(Date or Day, Hour, Minute, Second, Mode, Armed = true)
  // clock.setAlarm1(0, 0, 0, (myNodeID*TXoffset)%60, DS3231_MATCH_S);

  // Set Alarm1 - 3h:00m:00s on every Sunday (1 - Mon, 7 - Sun)         // Set to request time-update for RTC each week
  // setAlarm1(Date or Day, Hour, Minute, Second, Mode, Armed = true)
  clock.setAlarm1(7, 3, 0, 0, DS3231_MATCH_DY_H_M_S);
  
/* Alarm 2 is set to wake us up every hour to see if it time to make a sensor reading */
  // Set Alarm2 - At "myNodeID" minute past the hour                   // this allow for pseudo-random TX time each hour
  // setAlarm2(Date or Day, Hour, Minute, Mode, Armed = true)
  clock.setAlarm2(0, 0, (myNodeID*TXoffset)%60, DS3231_MATCH_M);       // using NodeID to offset wake time and TX

  // Attach Interrupt.  DS3231 INT is connected to Pin 38
  attachInterrupt(DS3231Int, ClockAlarm, LOW);                         // please note that RISING and FALLING do NOT work on current Zero code base in sleep 

#endif

#if defined MyDS18B20
  SoilTemp.begin();
#endif

  // Request time from controller on startup
    requestTime();
    wait(5000);

} // end setup()


/* **************************************************************************** */
/* *********************** Presentation *************************************** */
/* **************************************************************************** */
void presentation()  
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo(SKETCHNAME, SKETCHVERSION, AckFlag);   wait(SendDelay);
 
  // Register this device as Water Moisture Sensor
  present(CHILD_ID1, S_MOISTURE, "Water Moisture", AckFlag); wait(SendDelay);
 }


/* **************************************************************************** 
 * 
 * Send's and print the cause of the last reset
 * It uses the M0 PM->RCAUSE register to detect the cause of the last reset.
 *
 * **************************************************************************** */

static const char *ResetCause[] = {"POR", "BOD12", "BOD33", "", "External", "Watchdog", "Software"};
 
void printCpuResetCause()
{
    unsigned int CauseIndex = 0; 

    if (PM->RCAUSE.bit.SYST)  {CauseIndex = 6;}

    // Syntax error due to #define WDT in CMSIS/4.0.0-atmel/Device/ATMEL/samd21/include/samd21j18a.h
    // if (PM->RCAUSE.bit.WDT) {
    if ((PM->RCAUSE.reg & PM_RCAUSE_WDT) != 0) {CauseIndex = 5;}

    if (PM->RCAUSE.bit.EXT)   {CauseIndex = 4;}

    if (PM->RCAUSE.bit.BOD33) {CauseIndex = 2;}

    if (PM->RCAUSE.bit.BOD12) {CauseIndex = 1;}

    if (PM->RCAUSE.bit.POR)   {CauseIndex = 0;}
    
    char txtBuffer[40];
    debug1(PSTR("*** CPU reset by %s [%u]\n"), ResetCause[CauseIndex], PM->RCAUSE.reg);
    sprintf(txtBuffer,"Reset by %s [%u]", ResetCause[CauseIndex], PM->RCAUSE.reg);
 
    send(TextMsg.set(txtBuffer), AckFlag);  wait(SendDelay);        // sending reset info to controller
}

/* **************** System Sleep ******************* */
void systemSleep()
{
    debug1(PSTR("\n*** Going to Sleep ***\n"));
    wait (100);
    // put led's, radio and flash to sleep
    // Turn off LED's
    pinMode (MY_DEFAULT_TX_LED_PIN, INPUT);
    pinMode (MY_DEFAULT_RX_LED_PIN, INPUT);
    pinMode (MY_DEFAULT_ERR_LED_PIN, INPUT);
    pinMode (OnBoardLed, INPUT); 
    
    // Put Flash to sleep
    

    // Put Radio and transport to Sleep
    transportPowerDown();               // Shut down radio and MySensor transport
    
    interrupts();                       // make sure interrupts are on...
    LowPower.standby();                 // SAMD sleep from LowPower systems
    
       //  .... we will wake up from sleeping here if triggered from an interrupt
    interrupts();                       // make sure interrupts are on...                                      
}

/* **************** System Wake-up from Sleep ******************* */
void systemWakeUp() 
{                                          
    
//  re enable LED's if needed
//    pinMode (MY_DEFAULT_TX_LED_PIN, OUTPUT);
//    pinMode (MY_DEFAULT_RX_LED_PIN, OUTPUT);
//    pinMode (MY_DEFAULT_ERR_LED_PIN, OUTPUT);
//    pinMode (OnBoardLed, OUTPUT);  

    // wake up Flash if needed

    // wake up MySensor transport and Radio from Sleep
    //transportInit();
    hwSleep(1);                         // as MySensor had NO sleep or Watch Dog for SAMD, this will
                                        // wake us up so that we can send and receive messages
    while (!isTransportReady()) {       // Make sure transport is ready
    _process(); }

    interrupts();                       // make sure interrupts are on...
}

/* **************** DS3231 Alarm Interrupt ******************* */
// We do nothing here except to reset the interrupt
#if defined MyDS3231
void ClockAlarm()
{
  if (clock.isAlarm1(false))              // if set to true, will also clear the alarm..
  {
    clock.clearAlarm1();
    alarmUpdateFlag = true;               // request time of day flag
    debug1(PSTR("*** Alarm 1 ***\n"));
  }

  if (clock.isAlarm2(false))
  {
    clock.clearAlarm2();
    debug1(PSTR("*** Alarm 2 ***\n"));
  }
}
#endif


/* ***************** Send Water Pressure ***************** */
void SendPressure()
{
#ifdef WaterPressure
/* We will read the analog input from the pressure transducer 
 *  and convert it from an analog voltage to a pressure in PSI
 *  
 *  Water pressure transducer requires 5V, may not be available in Solar, Battery systems
 * 
 *  Output: 0.5V – 4.5V linear voltage output. 0 psi outputs 0.5V, 50 psi outputs 2.5V, 100 psi outputs 4.5V 
 *  0   psi = .33v after scaling 5.0v to 3.3v
 *  50  psi = 1.65v
 *  100 psi = 2.97v
 *
 *  3.3v/1023 = .003225806 volt per bit or 3.3V/4095 = .000805861 volts per bit
 */
    pressure  = analogRead    (PressPin) ;                    // this is a junk read to clear ATD
    wait(25);
 
    pressure  = analogRead    (PressPin) ;

    if (pressure < 106) pressure = 106;                       // we have a minimum of .5v = 0 PSI
    PSI = (pressure - 106 ) * .1246;                          // where did we get this?? was .119904
    PSI = PSI + PSI_CAL;                                      // calibration adjustment if needed
    
    floatMSB = PSI * 100;
    floatR = floatMSB % 100;
    debug1(PSTR("PSI:  %0u.%02u\n"), floatMSB/100, floatR);

    send(PressureMsg.set(PSI, 2), AckFlag);  wait(SendDelay); // Send water pressure to gateway 
#endif  
}


/* ***************** Send Si7021 Temp & Humidity ***************** */
void getTempSi7021()
{
#if defined Sensor_SI7021
      si7021.triggerMeasurement();
      wait (25);
      si7021.getHumidity    (humi);
      si7021.getTemperature (temp);
      
      temp = (temp * 1.8) + 32.0;                                // to get deg F

      floatMSB = humi * 100;                                     // we don't have floating point printing in debug print
      floatR = floatMSB % 100; 
      debug1(PSTR("Humi: %0u.%02u%% \n"), floatMSB/100, floatR);
      
      send(HumMsg.set(humi, 2), AckFlag);  wait(SendDelay);

      floatMSB = temp * 100;                                     // we don't have floating point printing in debug print
      floatR = floatMSB % 100; 
      debug1(PSTR("Temp Si: %0u.%02uF \n"), floatMSB/100, floatR);
      
      send(TempMsg.set(temp, 2), AckFlag);  wait(SendDelay);
#endif
}


/* ***************** Send MCP9800 Temp ***************** */
void  getTempMCP9800 ()
{
#if defined Sensor_MCP9800
 
//    temp = MCP9800.readTempC16(AMBIENT) / 16.0;               // In deg C
      temp = MCP9800.readTempF10(AMBIENT) / 10.0;               // In deg F
      
      floatMSB = temp * 100;                                    // we don't have floating point printing in debug print
      floatR = floatMSB % 100; 
      debug1(PSTR("Temp MCP: %0u.%02uF \n"), floatMSB/100, floatR);
      
      send(TempMsg.set(temp, 2), AckFlag);  wait(SendDelay);
#endif
}

/* ***************** Send DS3231 Temp ***************** */
void getTempDS3231()
{
   #if defined  Sensor_MCP9800 || defined Sensor_SI7021        // we will used other Temp sensor if available
        // nothing here
   #elif defined MyDS3231
      clock.forceConversion();                                 // Start conversion of Temp sensor
      wait(25);
      temp = clock.readTemperature();
      temp = (temp * 1.8) + 32.0;                              // to get deg F
      floatMSB = temp * 100;                                   // we don't have floating point printing in debug print
      floatR = floatMSB % 100; 
      debug1(PSTR("Temp: %0u.%02uF \n"), floatMSB/100, floatR);
        
      send(TempMsg.set(temp, 2), AckFlag);  wait(SendDelay);

   #endif
}


/* ***************** Send Soil Temp DS18B20 ***************** */
void getSoilTemp()
{
  #if defined MyDS18B20
    SoilTemp.requestTemperaturesByIndex( 0 );                  // requestTemperaturesByIndex
    temp = SoilTemp.getTempFByIndex( 0 );
    if (temp > 150.0) temp = 150.0;                           // lets do a bounds check...
    if (temp < -32.0) temp = -32.0;
  
    floatMSB = temp * 100;                                     // we don't have floating point printing in debug print
    if (floatMSB < 0) floatMSB = 0.0;                          // we can't deal with negative numbers
    floatR = floatMSB % 100; 
    debug1(PSTR("Soil Temp: %0u.%02u F \n"), floatMSB/100, floatR);

    send(SoilTempMsg.set(temp, 2), AckFlag);  wait(SendDelay);
     
  #endif
}


/* ***************** Send Keep Alive status ***************** */
void SendKeepAlive()
{       
#ifdef SendHeartbeat         
          sendHeartbeat();  wait(SendDelay);
          debug1(PSTR("*** Sending Heart Beat ***\n\n"));
#endif
          SendPressure();                                               // send water pressure to GW if we have it
          getTempSi7021();                                              // send Temp and Humidity to GW if we have it
          getTempMCP9800();                                             // send Temp to GW if we have it
          getTempDS3231();                                              // send Temp to GW if we have it
          getSoilTemp();                                                // send Soil Temp if we have it
          keepaliveTime = currentTime;                                  // reset timer 
}


/* **************************************************************************** */
/*                               Loop                                           */
/* **************************************************************************** */
void loop()     
{ 
  //wdt_reset();
   
    currentTime = millis();                                    

 /* ***************** Send Sensor Data ***************** */
#ifndef  MyDS3231   // this is used for debug only, it allows us to send data at a faster rate

    if (currentTime - lastSendTime >= SEND_FREQUENCY)          // Only send values at a maximum rate (used for debug)
    {
      debug1(PSTR("\n*** Sending Sensor Data\n")); 
     
#else
            
    if ((MySendTime[dt.hour] == 1)  || (dt.hour == 12))         // See if it time to send data, we always send at noon
    {
      if (dt.hour == 12) KeepAwakeFlag = false;                // we only want to keep awake till noon
      systemWakeUp();                                          // we have work to do, so wake up radio and transport
      debug1(PSTR("\n*** Sending Sensor Data at: %u:%02u:%02u\n"), dt.hour, dt.minute, dt.second); 

        if (alarmUpdateFlag == true)                           // if its time, request time update from controller
        {
          debug1(PSTR("\n*** Requesting Time\n")); 
          alarmUpdateFlag = false;
          requestTime();  wait(SendDelay);                      // Ask controller for time of day
          wait (2000);
        }
#endif

      lastSendTime = currentTime;
       
      // Request a TX Schedule update at this time...
      debug1(PSTR("\n*** Requesting Schedule Update\n")); 
      send(ScheduleUpdate.set(1,0), AckFlag);  wait(SendDelay);
      wait (2000);
      
      soilsensors(); 
      int vbat = analogRead(BattVolt);                        // we will do it twice, junk the 1st read
      vbat = analogRead(BattVolt);
      float Vsys =  vbat * 0.000805664 * 1.97;                // read the battery voltage, 12bits = 0 -> 4095, divider is 1/2, so max = 6.6v
      send(VBAT.set(Vsys, 2), AckFlag);  wait(SendDelay);
      sendBatteryLevel(vbat/41);  wait (SendDelay);           // Send MySensor battery in %, count / 41 = 4095/41 = 99%
      floatMSB = Vsys * 100;                                  // we don't have floating point printing in debug print
      floatR = floatMSB % 100; 
      debug1(PSTR("Vbat: %0u.%02uV \n"), floatMSB/100, floatR);

      SendKeepAlive(); 
      
      wait (StayAwakeTime);                                    // Stay awake time
    }
     
#ifdef MyDS3231
     if ( KeepAwakeFlag == false)                              // see if keep alive flag is not set
     {
      systemSleep();                                           // ok, it bedtime, go to sleep
         // Ok, its wake up time....
      dt =  clock.getDateTime();                               // get current time from DS3231 
      debug1(PSTR("\n*** Wakeing From Sleep at: %u:%02u:%02u\n"), dt.hour, dt.minute, dt.second);
     }   
#endif
                       
}   // end of loop 



/****************** Message Receive Loop ***************************
 * 
 * This is the message receive loop, here we look for messages address to us 
 * 
 * ***************************************************************** */
void receive(const MyMessage &message) 
{
   debug2(PSTR("*** Received message from gw:\n"));
   //debug2(PSTR("Last: %u, Sender: %u, Dest: %u, Type: %u, Sensor: %u\n"), message.last, message.sender, message.destination, message.type, message.sensor);
   
// Make sure its for our child ID
  if (message.sensor == CHILD_ID0 )
  {
    if  (message.type==V_VAR1)                                        // (24) This will received a new schedule time to send sensor data
      {
       unsigned long newSchedule = message.getULong();
       debug2(PSTR("*** Received V_Var1 message: 0x%x\n"), newSchedule );
       for (i=0; i< 23; i++) 
        {  
          MySendTime[i] = ((newSchedule >> i) & 0x00000001);
        }          
      }
    
     if ( message.type==V_VAR2)                                       // (25) This will received a "do not sleep flag", that keep us awake until noon each day
      {
        unsigned int flag = message.getUInt();
        if (flag == KeepAliveID) KeepAwakeFlag = true;          
        debug2(PSTR("*** Received V_VAR2 message: 0x%x\n"), flag );
      }
    
     if ( message.type==V_VAR3)                                       // (26)
      {
      
        debug2(PSTR("*** Received V_VAR3 message from gw"));
      }

     if ( message.type==V_VAR4)                                       // (27)
      {
        debug2(PSTR("*** Received V_VAR4 message from gw\n") );
      }
    }  // end if (message.sensor == CHILD_ID1 )


 // Check for any messages for child ID = 1
  if (message.sensor == CHILD_ID1 )
    {
      debug2(PSTR("*** Received Child ID-1 message from gw. \n"));
    }
}  // end of received()

void receiveTime(unsigned long ts)
{
  debug1(PSTR("*** Received Time from gw: %lu \n"), ts);

#ifdef MyDS3231
  // Set from UNIX timestamp
    clock.setDateTime(ts);                              // Set time in DS3231
    dt =  clock.getDateTime();                          // Set time in system clock
#endif
}


/* ******************************************************** 
 *  
 *  Based on the work of: Reinier van der Lee, www.vanderleevineyard.com
 *     
 ********************************************************** */
void soilsensors()
{
// Select sensor 1, and enable MUX
  digitalWrite(MuxA, LOW); 
  digitalWrite(MuxB, LOW); 
  digitalWrite(MuxINH, LOW); 
  measureSensor();
  read1 = average();

    moisture = GetMoisture(read1);
    debug1(PSTR("Moisture 1: %u Res: %u\n"), moisture, read1);
    send(LEVEL1.set(moisture), AckFlag);  wait(SendDelay);
    send(IMP1.set(read1), AckFlag);  wait(SendDelay);

// Select sensor 2, and enable MUX
  digitalWrite(MuxA, HIGH); 
  digitalWrite(MuxB, LOW); 
  digitalWrite(MuxINH, LOW); 
  measureSensor();
  read2 = average();

    moisture = GetMoisture(read2);
    debug1(PSTR("Moisture 2: %u Res: %u\n"), moisture, read2);
    send(LEVEL2.set(moisture), AckFlag);  wait(SendDelay);
    send(IMP2.set(read2), AckFlag);  wait(SendDelay);

    // Select sensor 3, and enable MUX
  digitalWrite(MuxA, LOW); 
  digitalWrite(MuxB, HIGH); 
  digitalWrite(MuxINH, LOW); 
  measureSensor();
  read3 = average();

    moisture = GetMoisture(read3);
    debug1(PSTR("Moisture 3: %u Res: %u\n"), moisture, read3);
    send(LEVEL3.set(moisture), AckFlag);  wait(SendDelay);
    send(IMP3.set(read3), AckFlag);  wait(SendDelay);


  // Select sensor 4, and enable MUX
  digitalWrite(MuxA, HIGH); 
  digitalWrite(MuxB, HIGH); 
  digitalWrite(MuxINH, LOW); 
  measureSensor();
  read4 = average();

    moisture = GetMoisture(read4);
    debug1(PSTR("Moisture 4: %u Res: %u\n"), moisture, read4);
    send(LEVEL4.set(moisture), AckFlag);  wait(SendDelay);
    send(IMP4.set(read4), AckFlag);  wait(SendDelay);

  digitalWrite(MuxINH, HIGH);             // Disable Mux, this will isolate CPU from Sensor

  return;
}


/* ******************************************************** */ 
int GetMoisture(unsigned long read)
{
    // this equation changes based on calibration of sensors
    int moisture = min( int( pow( read / 31.65 , 1.0 / -1.695 ) * 400 + 0.5 ) , 100 ); 
    if (moisture < 0)   moisture = 0;
  //  if (moisture > 100) moisture = 100;
  return moisture;
}


/* ******************************************************** */
void measureSensor()
{
  for (i=0; i< NUM_READS; i++) 
      {
        pinMode(SensDX, OUTPUT);              // this will be used as current source
        pinMode(SensDY, INPUT);               // we will read from this channel 
        digitalWrite(SensDX, LOW);            // clear any stray current
        wait (10);  
        digitalWrite(SensDX, HIGH);           // set it high to supply current to sensor 
        delayMicroseconds(250);
        sensorVoltage = analogRead(SensAY);   // read the sensor voltage
        supplyVoltage = analogRead(SensAX);   // read the supply voltage
               
        digitalWrite(SensDX, LOW);            // clear any stray current
        wait (10);   
        pinMode(SensDX, INPUT);               // HiZ the input channel
        pinMode(SensDY, INPUT);               // HiZ the input channel
        resistance = (knownResistor * (supplyVoltage - sensorVoltage ) / sensorVoltage) - zeroCalibration ;
        if (resistance < 0) resistance = 0;   // do some reasonable bounds checking
             
        addReading(resistance);               // save it
        delayMicroseconds(250);
        
    // Invert the current and do it again...
        pinMode(SensDY, OUTPUT);              // this will be used as current source
        pinMode(SensDX, INPUT);               // we will read from this channel  
        digitalWrite(SensDY, LOW);            // clear any stray current
        wait (10);    
        digitalWrite(SensDY, HIGH);           // set it high to supply current to sensor  
        delayMicroseconds(250);
        sensorVoltage = analogRead(SensAX);   // read the sensor voltage
        supplyVoltage = analogRead(SensAY);   // read the supply voltage
        
        digitalWrite(SensDY, LOW);            // clear any stray current
        wait (10); 
        pinMode(SensDY, INPUT);               // HiZ the input channel
        pinMode(SensDX, INPUT);               // HiZ the input channel
        
        resistance = (knownResistor * (supplyVoltage - sensorVoltage ) / sensorVoltage) - zeroCalibration ;
        if (resistance < 0) resistance = 0;   // do some reasonable bounds checking
        
       addReading(resistance);                // save it
       wait(100);
      } 
}

/* ******************************************************** */
// Averaging algorithm
void addReading(long resistance)
{
    buffer[index2] = resistance;
    index2++;
    if (index2 >= NUM_READS) index2 = 0;
}

/* ******************************************************** */
  long average()
  {
    long sum = 0;
    for (int i = 0; i < NUM_READS; i++)
    {
      sum += buffer[i];
    }
    return (long)(sum / NUM_READS);
}


/* *********************** The End ************************ */
/* ******************************************************** */ 

