//-------------------------------------------------------------------
// Nerdkiez Brew Controller
// Maintainer: Christoph Gerneth
//
// Inspired from Sous Vide Controller
// Bill Earl - for Adafruit Industries
//
// Based on the Arduino PID and PID AutoTune Libraries 
// by Brett Beauregard
//------------------------------------------------------------------

// PID Library and Autotune
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

// Libraries for Liquid Crystal (Arduino Keypad Shield from DFRobot)
#include <LiquidCrystal.h>

// Libraries for the DS18B20 Temperature Sensor
#include <OneWire.h>
#include <DallasTemperature.h>

// Stores K-Values in EEPROM
#include <EEPROM.h>

#define DEBUG

// ************************************************
// Pin definitions
// ************************************************

// Output Relay
#define RelayPin 13

// One-Wire Temperature Sensor
#define ONE_WIRE_BUS 2
#define ONE_WIRE_PWR 3
#define ONE_WIRE_GND 11

// ************************************************
// PID Variables and constants
// ************************************************

//Define Variables we'll be connecting to
double Setpoint;
double Input;
double Output;

volatile long onTime = 0;

// pid tuning parameters
double Kp;
double Ki;
double Kd;

// EEPROM addresses for persisted data
const int SpAddress = 0;
const int KpAddress = 8;
const int KiAddress = 16;
const int KdAddress = 24;

// Button Settings
int adc_key_in;
#define BUTTON_ADC_PIN 0
#define BUTTON_SELECT 7 
#define BUTTON_LEFT 1
#define BUTTON_RIGHT 2
#define BUTTON_UP 3
#define BUTTON_DOWN 4
#define BUTTON_NONE 10


//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// 10 second Time Proportional Output window
int WindowSize = 10000; 
unsigned long windowStartTime;

// ************************************************
// Auto Tune Variables and constants
// ************************************************
byte ATuneModeRemember=2;

double aTuneStep=500;
double aTuneNoise=1;
unsigned int aTuneLookBack=20;

boolean tuning = false;

PID_ATune aTune(&Input, &Output);

// ************************************************
// DiSplay Variables and constants
// ************************************************

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

unsigned long lastInput = 0; // last button press

byte degree[8] = // define the degree symbol 
{ 
 B00110, 
 B01001, 
 B01001, 
 B00110, 
 B00000,
 B00000, 
 B00000, 
 B00000 
}; 

const int logInterval = 10000; // log every 10 seconds
long lastLogTime = 0;

// ************************************************
// States for state machine
// ************************************************
enum operatingState { OFF = 0, SETP, RUN, TUNE_P, TUNE_I, TUNE_D, AUTO};
operatingState opState = OFF;

// ************************************************
// Sensor Variables and constants
// Data wire is plugged into port 2 on the Arduino

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress tempSensor;

// ************************************************
// Setup and diSplay initial screen
// ************************************************
void setup()
{
   Serial.begin(9600);

   // Initialize Relay Control:

   pinMode(RelayPin, OUTPUT);    // Output mode to drive relay
   digitalWrite(RelayPin, LOW);  // make sure it is off to start

   // Set up Ground & Power for the sensor from GPIO pins

   pinMode(ONE_WIRE_GND, OUTPUT);
   digitalWrite(ONE_WIRE_GND, LOW);

   pinMode(ONE_WIRE_PWR, OUTPUT);
   digitalWrite(ONE_WIRE_PWR, HIGH);

   // Initialize LCD DiSplay 
   lcd.setBacklightPin(10, POSITIVE);
   lcd.begin(16, 2);
   lcd.createChar(1, degree); // create degree symbol from the binary
   
   lcd.setCursor(0,0);
   lcd.print(F("    Nerdkiez"));
   lcd.setCursor(0, 1);
   lcd.print(F("  Brew Control!"));

   // Start up the DS18B20 One Wire Temperature Sensor
   sensors.begin();
   if (!sensors.getAddress(tempSensor, 0)) 
   {
      lcd.setCursor(0, 1);
      lcd.print(F("Sensor Error"));
   }
   sensors.setResolution(tempSensor, 12);
   sensors.setWaitForConversion(false);


   // Initialize the PID and related variables
   LoadParameters();
   myPID.SetTunings(Kp,Ki,Kd);
   lcd.setCursor(0,1);

   myPID.SetSampleTime(1000);
   myPID.SetOutputLimits(0, WindowSize);

  // Run timer2 interrupt every 15 ms 
  TCCR2A = 0;
  TCCR2B = 1<<CS22 | 1<<CS21 | 1<<CS20;

  //Timer2 Overflow Interrupt Enable
  TIMSK2 |= 1<<TOIE2;
}

// ************************************************
// Timer Interrupt Handler
// ************************************************
SIGNAL(TIMER2_OVF_vect) 
{
  if (opState == OFF)
  {
    digitalWrite(RelayPin, LOW);  // make sure relay is off
  }
  else
  {
    DriveOutput();
  }
}

// ************************************************
// Main Control Loop
//
// All state changes pass through here
// ************************************************
void loop()
{
   #ifdef DEBUG
    Serial.print("State is: ");
    Serial.println(opState);
   #endif

   // wait for button release before changing state
   while(ReadButtons() == BUTTON_NONE) {}

   lcd.clear();

   switch (opState)
   {
   case OFF:
      Off();
      break;
   case SETP:
      Tune_Sp();
      break;
    case RUN:
      Run();
      break;
   case TUNE_P:
      TuneP();
      break;
   case TUNE_I:
      TuneI();
      break;
   case TUNE_D:
      TuneD();
      break;
   }
   delay(200);
}

// ************************************************
// Initial State - press RIGHT to enter setpoint
// ************************************************
void Off()
{
   myPID.SetMode(MANUAL);
   lcd.setBacklight(0);
   digitalWrite(RelayPin, LOW);  // make sure it is off
   lcd.print(F("    Nerdkiez"));
   lcd.setCursor(0, 1);
   lcd.print(F("  Brewcontrol!"));
   uint8_t buttons = 0;
   
   while(!(buttons & (BUTTON_RIGHT)))
   {
      buttons = ReadButtons();
   }
   // Prepare to transition to the RUN state
   sensors.requestTemperatures(); // Start an asynchronous temperature reading

   //turn the PID on
   myPID.SetMode(AUTOMATIC);
   windowStartTime = millis();
   opState = RUN; // start control
}

// ************************************************
// Setpoint Entry State
// UP/DOWN to change setpoint
// RIGHT for tuning parameters
// LEFT for OFF
// SHIFT for 10x tuning
// ************************************************
void Tune_Sp()
{
   lcd.print(F("Set Temperature:"));
   uint8_t buttons = 0;
   float increment = 0.1;
   while(true)
   {
      buttons = ReadButtons();

      switch (buttons)
      {
      case BUTTON_NONE:
         // do nothing
         break;
      case BUTTON_SELECT:
         increment *= 10;
         delay(200);
         break;
      case BUTTON_LEFT:
         opState = RUN;
         return;
      case BUTTON_RIGHT:
         opState = TUNE_P;
         return;
      case BUTTON_UP:
         Setpoint += increment;
         delay(200);
         break;
      case BUTTON_DOWN:
         Setpoint -= increment;
         delay(200);
         break;
      default:
         break;
      }
    
      if ((millis() - lastInput) > 3000)  // return to RUN after 3 seconds idle
      {
         opState = RUN;
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(Setpoint);
      lcd.setCursor(10,1);
      lcd.print(increment);
      lcd.setCursor(0,1);
      DoControl();
   }
}

// ************************************************
// Proportional Tuning State
// UP/DOWN to change Kp
// RIGHT for Ki
// LEFT for setpoint
// SHIFT for 10x tuning
// ************************************************
void TuneP()
{
   lcd.print(F("Set Kp"));

   uint8_t buttons = 0;
   float increment = 1.0;
   while(true)
   {
      buttons = ReadButtons();

      switch (buttons)
      {
      case BUTTON_NONE:
         // do nothing
         break;
      case BUTTON_SELECT:
         increment *= 10;
         delay(200);
         break;
      case BUTTON_LEFT:
         opState = SETP;
         return;
      case BUTTON_RIGHT:
         opState = TUNE_I;
         return;
      case BUTTON_UP:
         Kp += increment;
         delay(200);
         break;
      case BUTTON_DOWN:
         Kp -= increment;
         delay(200);
         break;
      default:
         break;
      }

      if ((millis() - lastInput) > 3000)  // return to RUN after 3 seconds idle
      {
         opState = RUN;
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(Kp);
      lcd.print(" ");
      DoControl();
   }
}

// ************************************************
// Integral Tuning State
// UP/DOWN to change Ki
// RIGHT for Kd
// LEFT for Kp
// SHIFT for 10x tuning
// ************************************************
void TuneI()
{
   lcd.print(F("Set Ki"));

   uint8_t buttons = 0;
   float increment = 0.01;
   while(true)
   {
      buttons = ReadButtons();

      switch (buttons)
      {
      case BUTTON_NONE:
         break; 
      case BUTTON_SELECT:
         increment *= 10;
         delay(200);
         break;
      case BUTTON_LEFT:
         opState = TUNE_P;
         return;
      case BUTTON_RIGHT:
         opState = TUNE_D;
         return;
      case BUTTON_UP:
         Ki += increment;
         delay(200);
         break;
      case BUTTON_DOWN:
        Ki -= increment;
        delay(200);
        break; 
      default:
         break;
      }

      if ((millis() - lastInput) > 3000)  // return to RUN after 3 seconds idle
      {
         opState = RUN;
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(Ki);
      lcd.print(" ");
      DoControl();
   }
}

// ************************************************
// Derivative Tuning State
// UP/DOWN to change Kd
// RIGHT for setpoint
// LEFT for Ki
// SHIFT for 10x tuning
// ************************************************
void TuneD()
{
   lcd.print(F("Set Kd"));

   uint8_t buttons = 0;
   float increment = 0.01;
   while(true)
   {
      buttons = ReadButtons();

      switch (buttons)
      {
      case BUTTON_NONE:
         break;
      case BUTTON_LEFT:
         opState = TUNE_I;
         return;
      case BUTTON_RIGHT:
         opState = RUN;
         return;
      case BUTTON_UP:
         Kd += increment;
         delay(200);
         break;
      case BUTTON_DOWN:
         Kd -= increment;
         delay(200);
         break; 
      default:
         break;
      }

      if ((millis() - lastInput) > 3000)  // return to RUN after 3 seconds idle
      {
         opState = RUN;
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(Kd);
      lcd.print(" ");
      DoControl();
   }
}

// ************************************************
// PID Control State
// SHIFT for autotune
// RIGHT - Setpoint
// LEFT - OFF
// ************************************************
void Run()
{
   // set up the LCD's number of rows and columns: 
   lcd.setBacklight(1);
   lcd.print(F("Sp: "));
   lcd.print(Setpoint);
   lcd.write(1);
   lcd.print(F("C : "));

   SaveParameters();
   myPID.SetTunings(Kp,Ki,Kd);

   uint8_t buttons = 0;
   while(true)
   {
      buttons = ReadButtons();
      if ((buttons == BUTTON_SELECT) 
         && (abs(Input - Setpoint) < 0.5))  // Should be at steady-state
      {
         StartAutoTune();
      }
      else if (buttons == BUTTON_RIGHT)
      {
        opState = SETP;
        return;
      }
      else if (buttons == BUTTON_LEFT)
      {
        opState = OFF;
        return;
      }
      
      DoControl();
      
      lcd.setCursor(0,1);
      lcd.print(Input);
      lcd.write(1);
      lcd.print(F("C : "));
      
      float pct = map(Output, 0, WindowSize, 0, 1000);
      lcd.setCursor(10,1);
      lcd.print(F("      "));
      lcd.setCursor(10,1);
      lcd.print(pct/10);
      //lcd.print(Output);
      lcd.print("%");

      lcd.setCursor(15,0);
      if (tuning)
      {
        lcd.print("T");
      }
      else
      {
        lcd.print(" ");
      }
      
      // periodically log to serial port in csv format
      if (millis() - lastLogTime > logInterval)  
      {
        lastLogTime = millis();
        Serial.print(Input);
        Serial.print(",");
        Serial.println(Output);
      }

      delay(100);
   }
}

// ************************************************
// Execute the control loop
// ************************************************
void DoControl()
{
  // Read the input:
  Input = sensors.getTempC(tempSensor);
  sensors.requestTemperatures(); // prime the pump for the next one - but don't wait
  
  if (tuning) // run the auto-tuner
  {
     if (aTune.Runtime()) // returns 'true' when done
     {
        FinishAutoTune();
     }
  }
  else // Execute control algorithm
  {
     myPID.Compute();
  }
  
  // Time Proportional relay state is updated regularly via timer interrupt.
  onTime = Output; 
}

// ************************************************
// Called by ISR every 15ms to drive the output
// ************************************************
void DriveOutput()
{  
  long now = millis();
  // Set the output
  // "on time" is proportional to the PID output
  if(now - windowStartTime>WindowSize)
  { //time to shift the Relay Window
     windowStartTime += WindowSize;
  }
  if((onTime > 100) && (onTime > (now - windowStartTime)))
  {
     digitalWrite(RelayPin,HIGH);
  }
  else
  {
     digitalWrite(RelayPin,LOW);
  }
}

// ************************************************
// Start the Auto-Tuning cycle
// ************************************************

void StartAutoTune()
{
   // REmember the mode we were in
   ATuneModeRemember = myPID.GetMode();

   // set up the auto-tune parameters
   aTune.SetNoiseBand(aTuneNoise);
   aTune.SetOutputStep(aTuneStep);
   aTune.SetLookbackSec((int)aTuneLookBack);
   tuning = true;
}

// ************************************************
// Return to normal control
// ************************************************
void FinishAutoTune()
{
   tuning = false;

   // Extract the auto-tune calculated parameters
   Kp = aTune.GetKp();
   Ki = aTune.GetKi();
   Kd = aTune.GetKd();

   // Re-tune the PID and revert to normal control mode
   myPID.SetTunings(Kp,Ki,Kd);
   myPID.SetMode(ATuneModeRemember);
   
   // Persist any changed parameters to EEPROM
   SaveParameters();
}

// ************************************************
// Check buttons and time-stamp the last press
// ************************************************
uint8_t ReadButtons()
{
  // read the value from the sensor
  adc_key_in = analogRead(BUTTON_ADC_PIN);  // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
  
  if (adc_key_in <= 1000 ){
    lastInput = millis();
  }

  // we add approx 50 to those values and check to see if we are close
  if (adc_key_in > 1000) return BUTTON_NONE; // We make this the 1st option for speed reasons since it will be the most likely result
  if (adc_key_in < 50)   return BUTTON_RIGHT;
  if (adc_key_in < 195)  return BUTTON_UP;
  if (adc_key_in < 380)  return BUTTON_DOWN;
  if (adc_key_in < 555)  return BUTTON_LEFT;
  if (adc_key_in < 790)  return BUTTON_SELECT;
  return 9;
}


// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void SaveParameters()
{ 
   if (Setpoint != EEPROM_readDouble(SpAddress))
   {
      EEPROM_writeDouble(SpAddress, Setpoint);
   }
   if (Kp != EEPROM_readDouble(KpAddress))
   {
      EEPROM_writeDouble(KpAddress, Kp);
   }
   if (Ki != EEPROM_readDouble(KiAddress))
   {
      EEPROM_writeDouble(KiAddress, Ki);
   }
   if (Kd != EEPROM_readDouble(KdAddress))
   {
      EEPROM_writeDouble(KdAddress, Kd);
   }
}

// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters()
{
  // Load from EEPROM
   Setpoint = EEPROM_readDouble(SpAddress);
   Kp = EEPROM_readDouble(KpAddress);
   Ki = EEPROM_readDouble(KiAddress);
   Kd = EEPROM_readDouble(KdAddress);
   
   // Use defaults if EEPROM values are invalid
   if (isnan(Setpoint))
   {
     Setpoint = 60;
   }
   if (isnan(Kp))
   {
     Kp = 850;
   }
   if (isnan(Ki))
   {
     Ki = 0.5;
   }
   if (isnan(Kd))
   {
     Kd = 0.1;
   }  
}


// ************************************************
// Write floating point values to EEPROM
// ************************************************
void EEPROM_writeDouble(int address, double value)
{
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      EEPROM.write(address++, *p++);
   }
}

// ************************************************
// Read floating point values from EEPROM
// ************************************************
double EEPROM_readDouble(int address)
{
   double value = 0.0;
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      *p++ = EEPROM.read(address++);
   }
   return value;
}
