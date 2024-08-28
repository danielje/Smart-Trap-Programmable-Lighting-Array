/*
  Trap_Light_Programmable1_0_01 programmable variation of Trap_Light_Basic
  where light "waves" sinusoidally through all 6 LEDs
  
  Daniel M. Jenkins
  October 19, 2015
  
  updated November 3, 2015- slow baud down to 38400, and handle communication of each control parameter independently to prevent garbling
  
  updated December 15, 2015 (version 1_0_02), to only read the photoconductor divider voltage (not compared to an adjustable reference); user
  now has option to hit a button on interface to update the ambient light threshold for day/ night into EEPROM

  updated June 21, 2017 (version 1_0_03), to turn off LEDs during ambient light measurement (in the Smart-Trap, the higher wavelength LEDs especially
  result in reflections from the trap back into the photocell to trick the device into thinking it is still daytime (with the result that LED state will toggle every minute
  or LIGHT_CHECK_INTERVAL during low light conditions, and night duration will not be recorded correctly. Also replaced micros() with millis() in ambient light sensor
  data type overrun check at start of ALS_Analysis(). Also made cumulativeIntensities a global variable that is only updated at startup, and when new intensity values are written
  to controller (not during every program cycle and/or "OntheGoLightCheck"- which might slow down modulation)

  updated December 20, 2017 (version 1_0_04b) to fix issues resulting from previous update when modulation occurs (i.e. frequency and duty cycle is no longer programmable / behaviour
  is not correct for Pulse_LEDs and Wave_LEDs); reverted back to version 1_0_02 to edit from (as this was the last correctly working version), to add in correct code to
  do intended updates from version 1_0_03...

  updated May 28, 2018, (version 1_0_05) to put MCU to sleep in ~1 minute increments when lights not operating, to conserve power... Also offloaded the ALS_Analysis timing
  to Watchdog Timer (so no need for cruder software timing with LIGHT_CHECK_INTERVAL, lastLightCheck, etc...
 */
 
#include <EEPROM.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
 
const int BRIDGE = 2;  // power to "ambient light" voltage divider circuit
const int ENABLE = 4;  // dout to enable pin on (all) AS1101 LED drivers

const int RED = 3;
const int AMBER = 5;
const int YELLOW = 6;
const int GREEN = 9;
const int BLUE = 10;
const int UV = 11;  // Digital pin assignments controlling
                    // different color LEDs
                    
// EEPROM memory assignments
const int INTENS = 0;  // occupies registers 0 - 5, contains PWM match value for each respective LED
const int CONFIGURE = 6;  // bits 7 & 6 reserved; 5 & 4 modulation solid, pulse, or wave; 3 sequence (simultaneous or one at a time); 2, 1, & 0 operation
                                      // 24/7 operation, night only, sunrise, sunset, or sunrise + sunset
const int FREQ = 7;  // can range from 1 - 100; gives frequency for pulsing/ waving between 0.1 and 10.0 Hz in 0.1Hz increments
const int DUTY = 8;  // 0 - 100% in 1% increments...
const int NIGHT_T = 9;  // duration, in minutes, of "dark" (occupies two registers)
const int SUNSET_T = 11;  // duration in minutes to run after sunset
const int SUNRISE_T = 13;  // duration in minutes to run before sunrise
const int ALS_THRESHOLD = 15;  // ambient light threshold- 2 bytes store 16 bit unsigned integer for light threshold to determine night or day

// options for modulation/ operational cycle
const byte SOLID = 0x00;
const byte PULSE = 0x01;
const byte WAVE = 0x02;

const byte OFF = 0x00;
const byte TWENTYFOURSEVEN = 0x01;
const byte NOCTURNAL = 0x02;
const byte SUNSET = 0x03;
const byte SUNRISE = 0x04;
const byte SUNSET_SUNRISE = 0x05;
                    
int spectrum[6] = {UV, BLUE, GREEN, YELLOW, AMBER, RED};  // reordered array to coincide with same order in Android interface...
byte intensities[6];  // intensities of each LED (PWM match values)
float frequency;  // frequency in hertz for LED modulation
byte duty_cycle;  // duty cycle for modulation (PULSE or WAVE) of LEDs- ranges from 0 to 100% in integer increments
unsigned int night_duration, sunset_duration, sunrise_duration;  // in minutes, duration of night, and durations of LED operation after sunset and before sunrise

unsigned int light_threshold;  // threshold for determination of day/ night transitions

unsigned int cumulativeIntensities; // sum of the value of programmed intensities for all LEDs (if 0, then no need to worry about turning anything on)
boolean lightsOn = false;

byte modulation;  // options are SOLID (on all the time), PULSE, WAVE
byte cycle;  // operational cycle; options are OFF, TWENTYFOURSEVEN i.e. on all time, NOCTURNAL, SUNSET, SUNRISE, SUNRISE_SUNSET
boolean SEQ = false;  // boolean variable- if true and modulation == PULSE or WAVE, enabled LEDs light in succession instead of simultaneously
boolean DARK = false;  // last observed ambient light condition- either DARK (night time), or not
boolean SUNSET_OBSERVED = false;  // boolean to record if a transition from day to night has been observed (in order to count minutes of night)
int dark_count = 0;  // count of minutes that it has been dark; sun is not considered set unless dark_count exceeds some threshold
int light_count = 0;  // count of minutes that it has been light; sun is not considered risin unless light_count exceeds threshold (prevents incorrect operation due to transient illumination or occlusion of photosensor)
int persistence = 0;  // count in minutes of persistence of the last observed ambient light condition (used to detect sunset/ sunrise events)
char command;  // command received on serial port

long serialTimeout = 0; // keep track of serial communication- don't let MCU sleep or enable WDT unless 1 minute elapsed from last serial comm; ensures smooth programming
volatile boolean WDT_event = false;
int WDT_count = 0;  // count for number of WDT elapsed (modulus 15- for WDT events every 4 seconds, will count 1 minute every 15 events)

void LEDs_Enable() {
  PORTD |= 0x10;  // force Digital pin 4 high, leave all others unchanged (controls enable pin of every LED driver)
}

void LEDs_Disable() {
  PORTD &= 0xef;  // force Digital pin 4 low, leave all others unchanged
}

void ALS_On() {  // enable ambient light sensing by energizing voltage divider photoconductor circuit
  PORTD |= 0x04;  // force digital pin 2 high, leave all others unchanged
}

void ALS_Off() {  // de-energize photoconductor circuit
  PORTD &= 0xfb;   // by forcing digital pin 2 low, leave all others unchanged
}

/*
  Returns the unsigned integer (2 bytes of data) starting at address addr of EEPROM- addr has least significant byte (LSB), and addr+1 has MSB)
*/
unsigned int ROM_Read (int addr)
{
  unsigned int Mem_Read = 0;
  Mem_Read = EEPROM.read(addr) + (EEPROM.read(addr+1)<<8);
  return Mem_Read;
}

/*
  Writes 2 bytes of data to EEPROM, in two consecutive registers starting at address addr (MSB goes in second data register address)
*/
void ROM_Write (int addr, unsigned int val)
{
  int MSBaddr = addr + 1;
  byte rite_bite = val & 0x00ff;
  EEPROM.write(addr, rite_bite);
  rite_bite = ((val&0xff00)>>8);
  EEPROM.write(MSBaddr, rite_bite);
}

/* Routine used by LED modulating functions Pulse_LEDs(), Wave_LEDs(), or Solid_LEDs() to determine if routines need to exit based on settings and analysis of diurnal cycle
returns "true" if conditions dictate that LEDs should be shut off- i.e. subroutine needs to be exited...*/
boolean On_The_Go_Light_Check() {
  boolean exit_decision = false;  // flag returned to signal exit from LED modulating function (either Pulse_LEDs(), Wave_LEDs(), or Solid_LEDs, based on settings and ambient light
  ALS_Analysis();  // check to see if time to read ambient light levels and update current conditions/ identify critical diurnal events
  if (cycle == OFF) exit_decision = true;  // if configuration is OFF
  else if (cycle != TWENTYFOURSEVEN) {  // otherwise, if not 24/7 operation must be some variant of nocturnal operation...
    if (!DARK) exit_decision = true;  // so exit if not dark
    else if ((cycle == SUNSET) && (dark_count > sunset_duration)) exit_decision = true;
    else if ((cycle == SUNRISE) && (dark_count < (night_duration - sunrise_duration))) exit_decision = true;
    else if ((cycle == SUNSET_SUNRISE) && (dark_count > sunset_duration) && (dark_count < (night_duration - sunrise_duration))) exit_decision = true; 
  }
  return exit_decision;
}

/* Evaluate light conditions and configuration settings to determine appropriate LED modulation (only called when "CYCLE != OFF",
if not already in a subroutine where LEDs are illuminated)*/
void Check_Operation() {
  //int cumulative_intensities = 0;
  //for (int i = 0; i < 6; i++) cumulative_intensities += intensities[i];  // count up/ sum all of the intensities of all LEDs   // only evaluate if LED's should turn on if at least one intensity is non-zero...
  if (cumulativeIntensities > 0) { // only evaluate if LED's should turn on if at least one intensity is non-zero...
    if (cycle == TWENTYFOURSEVEN) Implement_Modulation();  // turn on LEDs if configuration is 24/7,
    else if ((cycle == NOCTURNAL) && DARK) Implement_Modulation();  // or if nocturnal configuration and it is dark,
    else if (((cycle == SUNSET) || (cycle == SUNSET_SUNRISE)) && (dark_count <= sunset_duration) && DARK) Implement_Modulation(); // or the first "sunset_duration" after dark if config == SUNSET
    else if (((cycle == SUNRISE) || (cycle == SUNSET_SUNRISE)) && (dark_count >= (night_duration - sunrise_duration)) && DARK) Implement_Modulation();  // or "sunrise_duration" before sunrise...
  }
}

/* Called to determine which modulation settings should be implemented when evaluation of cycle settings and ambient light conditions determine LEDs should be illuminated*/
void Implement_Modulation() {
  PRR &= 0xb7; // edit power reduction register- make sure timers 1 and 2 turned on so we can control red green blue UV lights with PWM (leave other bits unchanged)...
  lightsOn = true;
  switch (modulation) {
    case SOLID:  // request from Android to report current device configuration
      Solid_LEDs();
      break;
    case PULSE:  // command from Android that new configuration data is incoming...
      Pulse_LEDs();
      break;
    case WAVE:
      Wave_LEDs();
      break;
    default:
      break;
  }
}

/* if LIGHT_CHECK_INTRERVAL elapsed, read ambient light and record appropriate statistics/ counts */
void ALS_Analysis() {
  if (!WDT_count && WDT_event) {  // WDT timout count has cycled back to zero and we haven't cleared the last event, check ALS and analyze for critical events (sunset, sunrise/ end of night_duration)
    DARK = !ALS_Check();  // read ambient light; ALS_check returns boolean indicating light/ daytime- invert for boolean indication of night
    if (DARK && (persistence >= 5)) {  // if it's been dark for at least 5 minutes...
      if (light_count >= 30) SUNSET_OBSERVED = true;  // then sunset has been observed it it was previously light for at least 30 minutes
      light_count = 0;  // in any case, reset the light count if dark persists 5 minutes
    }
    else if (!DARK && (persistence >= 5)) {  // if it's light for at least 5 minutes...
      if ((dark_count >= 30) && SUNSET_OBSERVED) {
        night_duration = dark_count;  // record night duration as the new dark_count if sunset was observed, and night duration is at least 30 minutes
        ROM_Write(NIGHT_T, night_duration);  // make sure the new night_duration gets recorded in EEPROM...
      }
      dark_count = 0;  // in any case reset dark_count,
      SUNSET_OBSERVED = false;  // and clear the SUNSET_OBSERVED flag (get ready to look for a new sunset to count next night duration)
    }
    WDT_event = false;  // clear last watchdog event (so we don't repeat analysis multiple times in ~0.5 s before count increments again)
  }
}

/* Check the ambient light, and increment/ restart light_count and dark_count appropriately; returns true if day, false if night/ dark*/
boolean ALS_Check() {
  LEDs_Disable();
  PRR &= 0xfe; // enable clock to ADC so ADC can start operating again (bit 0 of PRR, leave others unchanged)...
  ADCSRA |= 0x80; // enable Analog to Digital Converter (need it to measure ambient light level)
  ALS_On();  // energize ambient light circuit
  boolean luz;
  delay(10); // brief delay for possible transient (shouldn't be any in simple voltage divider)
  unsigned int darq = 0;
  for (int summation = 0; summation < 64; summation++) {
    darq += analogRead(1);  // sum up 64 successive 10 bit "light" readings on analog channel 1
    delayMicroseconds(260);  // as a crude digital filter to reject any noise in the system, especially when ambient light is near the threshold
  }
  ALS_Off();  // then turn off current to ALS circuit to save energy...
  ADCSRA &= 0x7f; // disable ADC (only use it once a minute to determine ambient light level- otherwise it's wasting energy...; enable ADC with ADC |= 0x80;
  PRR |= 0x01; // disable clock to ADC / power reduction register (leave other bits unchanged)
  if (lightsOn) LEDs_Enable(); // if we came in under conditions where LEDs were on, (re)activate the LED enable pin(s)...
  if (darq >= light_threshold) {  // photoconductor divider is on Analog 1, so when A1 > recorded threshold value, it is dark...
    if (DARK) {
      dark_count++;  // only increment DARK counter if previous observation was also dark
      persistence++;
    }
    else persistence = 0;  // reset persistence if daylight condition changes
    luz = false;
    //light_count = 0;
  }
  else {  // it is light/ daytime...
    if (!DARK) { 
      light_count++;  // only increment light counter if previous observation was also light...
      persistence++;
    }
    else persistence = 0;  // reset persistence if daylight condition changes...
    luz = true;
  }
  return luz;
}

/*
Read configuration byte programmed by Android
*/
void Read_Config_Byte() {
  byte configuration = (byte) Serial.parseInt();
  if ((configuration & 0x08) == 0x08) {
    SEQ = true;  // check the SEQ bit- bit 3 of configuration...
  }
  else {
    SEQ = false;
  }
  modulation = (configuration & 0x30) >> 4;  // mask off bits and right shift to determine modulation scheme (bits 5 & 4 of configuration)
  cycle = configuration & 0x07;  // mask off extra bits to determine cycle configuration (last three bits of configuration)
}

/*
Read updated intensity value for given LED index
*/
void Intensity_Update() {
  int channel = Serial.parseInt();
  intensities[channel] = Serial.parseInt();
  cumulativeIntensities = 0;
  for (int i = 0; i < 6; i++)
    cumulativeIntensities += intensities[i];
}

/* Report the device configuration to Android device (i.e., operating conditions, modulation scheme, etc)
The composition of the CONFIG register in EEPROM is D7 D6 reserved; D5 D4 modulation settings 00 SOLID, 01 PULSE, 10 WAVE;
D3 is SEQUENCE bit (i.e. when on enabled LEDs light up sequentially instead of simultaneously), 
D2 D1 D0 are cycle configuration 000 OFF, 001 TWENTYFOURSEVEN, 010 NOCTURNAL, 011 SUNSET, 100 SUNRISE, 101 SUNSET_SUNRISE
*/
void Report_Configuration() {
  for (int i = 0; i < 6; i++) {
    Serial.print('i');
    Serial.print(i);
    Serial.print('\t');
    Serial.print(intensities[i]);
    Serial.print('\t');
  }
  int configuration = cycle | (modulation << 4);
  if (SEQ) configuration |= 0x08;  // add on the "SEQ" bit into the configuration data
  Serial.print('b');
  Serial.print(configuration);
  Serial.print('\t');
  byte f = (byte) (frequency * 10);
  Serial.print('f');
  Serial.print(f);
  Serial.print('\t');
  Serial.print('d');
  Serial.print(duty_cycle);
  Serial.print('\t');
  Serial.print('n');
  Serial.print(night_duration);
  Serial.print('\t');
  Serial.print('s');
  Serial.print(sunset_duration);
  Serial.print('\t');
  Serial.print('r');
  Serial.print(sunrise_duration);
  Serial.print('\t');
}

/* Record the device configuration into EEPROM (i.e., operating conditions, modulation scheme, etc)
The composition of the CONFIG register in EEPROM is D7 D6 reserved; D5 D4 modulation settings 00 SOLID, 01 PULSE, 10 WAVE;
D3 is SEQUENCE bit (i.e. when on enabled LEDs light up sequentially instead of simultaneously), 
D2 D1 D0 are cycle configuration 000 OFF, 001 TWENTYFOURSEVEN, 010 NOCTURNAL, 011 SUNSET, 100 SUNRISE, 101 SUNSET_SUNRISE
*/
void Write_Configuration() {
  for (int i = 0; i < 6; i++) EEPROM.write(INTENS + i, intensities[i]);
  byte configuration = cycle | (modulation << 4);  // compose the configuration byte by ANDing the cycle with left shifted modulation settings
  if (SEQ) configuration |= 0x08;  // add on the "SEQ" bit into the configuration data
  EEPROM.write(CONFIGURE, configuration);
  byte f = (byte) (frequency * 10);  // code frequency as 10 x frequency (floating point with resolution 0.1 converted to whole number/ byte representation)
  EEPROM.write(FREQ, f);
  EEPROM.write(DUTY, duty_cycle);
  ROM_Write(NIGHT_T, night_duration);
  ROM_Write(SUNSET_T, sunset_duration);
  ROM_Write(SUNRISE_T, sunrise_duration);
  ROM_Write(ALS_THRESHOLD, light_threshold);
}

/* Read the configuration register to determine the operational parameters, i.e. modulation scheme and lighting, + 
other operating conditions i.e. duration on after sunset, respective LED intensities, etc...*/
void Read_Configuration() {
  cumulativeIntensities = 0;
  for (int i = 0; i < 6; i++) {
    intensities[i] = EEPROM.read(INTENS + i);  // read off match register values for PWM on each LED
    cumulativeIntensities += intensities[i];
  }
  byte configuration = EEPROM.read(CONFIGURE);
  SEQ = ((configuration & 0x08) >> 3) && 0x01;  // check the SEQ bit- bit 3 of configuration...
  modulation = (configuration & 0x30) >> 4;  // mask off bits and right shift to determine modulation scheme (bits 5 & 4 of configuration)
  cycle = configuration & 0x07;  // mask off extra bits to determine cycle configuration (last three bits of configuration)
  frequency = (float) (EEPROM.read(FREQ) / 10.0);
  duty_cycle = EEPROM.read(DUTY);
  night_duration = ROM_Read(NIGHT_T);
  sunset_duration = ROM_Read(SUNSET_T);
  sunrise_duration = ROM_Read(SUNRISE_T);
  light_threshold = ROM_Read(ALS_THRESHOLD);
}

/*
Function to pulse LEDs
*/
void Pulse_LEDs()
{
  boolean quit = false;
  LEDs_Enable();
  long Period = (long) (1000.0 / frequency); // period of pulses in ms
  long On_period = (long) ((Period * duty_cycle) / 100.0); // period of "on time"
  long last_pulse_start = millis();
  int j = 0;  // counter to keep track of which LED to turn on 
  while (!quit) {  // continue pulsing until something triggers quit, i.e. Serial communication
    if (SEQ) {
      int zeros = 0;
      while ((intensities[j] == 0) && !quit) {  // scroll through to find 
        j++;  // scroll down to the next
        zeros++;
        if (zeros >= 6) quit = true;  // if all intensities are set to 0, exit subroutine
        j %= 6;  // modulus of 6- i.e. wrap around to the next LED with non-zero intensity setting
      }
      analogWrite(spectrum[j], intensities[j]);  // if operating in sequence, only illuminate one LED at a time
      while(((millis() - last_pulse_start) < On_period) && !quit) {
        if (last_pulse_start > millis()) last_pulse_start = millis();  // wait until end of on period; don't let clock roll over
        quit = handleSerialTraffic();
      }
      digitalWrite(spectrum[j], LOW);
    }
    else {
      //int cumulative_intensities = 0;
      for (int i = 0; i < 6; i++) {
        analogWrite(spectrum[i], intensities[i]);  // set all of the LEDs on at designated intensity simultaneously (if !SEQ)
        //cumulative_intensities += intensities[i];  // count up/ sum all of the intensities of all LEDs
      }
      while(((millis() - last_pulse_start) < On_period) && !quit) {
        if (last_pulse_start > millis()) last_pulse_start = millis();  // wait until end of on period; don't let clock roll over
        quit = handleSerialTraffic();
      }
      for (int i = 0; i < 6; i++) digitalWrite(spectrum[i], LOW);  // then turn them all off...
    } 
    j++;  // for sequential operation of LEDs, increment the counter to activate the appropriate LED
    j %= 6;  // cycle back to first LED- don't let index exceed array size for intensities.
    while(((millis() - last_pulse_start) < Period) && !quit) {
      if (last_pulse_start > millis()) last_pulse_start = millis();  // wait until end of on period; don't let clock roll over
      quit = handleSerialTraffic(); // see if lighting conditions or serial instructions require us to exit
    }
    last_pulse_start += Period; // update new pulse start time at start of new cycle
  }
  for (int i = 0; i < 6; i++) digitalWrite(spectrum[i], LOW);  // make sure current setting is zero to each LED before exiting
  lightsOn = false;
  LEDs_Disable();  // disable all LED drivers- lower power mode
}

/*
Function to "wave" LEDs
*/
void Wave_LEDs()
{
  boolean quit = false;
  LEDs_Enable();
  long Period = 10000.0 / frequency; // period between successive intensity updates in microseconds
                    // numerator is 1000000 micros per second, divided by 100 discrete time steps per cycle
  long last_update = micros();
  int j = 0;  // counter to keep track of which LED to turn on
  int count = 0;  // counter to keep track of location on wave (cycles from 0 to 100 discrete time steps over the half "wave")
  float wave_factor = 0.0;  // factor between 0 and 1 to multiply intensity/ PWM match value- based on sin of the weighted count to achieve a half-wave
  float intense = 0.0;
  byte intensity = 0;
  while (!quit && (cumulativeIntensities != 0)) {  // continue pulsing until something triggers exit, i.e. Serial communication
    if (count == 0) {
      for (int i = 0; i < 6; i++) digitalWrite(spectrum[i], LOW);  // make sure LEDs off at start of cycle
      j++;  // for sequential operation of LEDs, increment the index to activate the appropriate LED at start of each wave
      j %= 6;  // cycle back to first LED- don't let index exceed array size for intensities.
    }
    wave_factor = sin(count * PI / 100);  // calculation of the wave factor- enables half wave behaviour over the 100 discrete update steps
    if (SEQ) {
      int zeros = 0;
      while ((intensities[j] == 0) && !quit) {  // scroll through to find next LED with non-zero intensity
        j++;  // scroll down to the next
        zeros++;  // count up number of consecutive zero intensities
        if (zeros >= 6) quit = true;  // if all intensities (6 consecutive values) are set to 0, exit subroutine
        j %= 6;  // modulus of 6- i.e. wrap around to the next LED with non-zero intensity setting
      }
      intense = intensities[j] * wave_factor + 0.5;  // Add 0.5 to value so that it rounds to nearest integer when truncated...
      intensity = (byte) intense;
      analogWrite(spectrum[j], intensity);  // if operating in sequence, only illuminate one LED at a time
    }
    else {  // if not sequenced, turn on every enabled LED at it's respective wave corrected intensity
      //int cumulative_intensities = 0;
      for (int i = 0; i < 6; i++) {
        intense = intensities[i] * wave_factor + 0.5;  // Add 0.5 to value so that it rounds to nearest integer when truncated...
        intensity = (byte) intense;
        analogWrite(spectrum[i], intensity);  // set all of the LEDs on at designated intensity simultaneously (if !SEQ)
        //cumulative_intensities += intensities[i];  // count up/ sum all of the intensities of all LEDs
      }
      if (cumulativeIntensities == 0) quit = true; // condition to exit if intensities are all zero...
    }
    if (!quit) quit = handleSerialTraffic();// this function also checks lighting conditions for any changes
    count++;
    count %= 100;  // reset count after 100- half wave is complete so start the next one
    while(((micros() - last_update) < Period) && !quit) {
      if (last_update > micros()) last_update = micros();  // wait until end of on period; don't let clock roll over
      // don't do any checks for serial traffic here because we should exit this loop within 100 ms max
    }
    last_update += Period; // update new pulse start time at start of new cycle
  }
  for (int i = 0; i < 6; i++) digitalWrite(spectrum[i], LOW);  // make sure current setting is zero to each LED before exiting
  lightsOn = false;
  LEDs_Disable();  // disable all LED drivers- lower power mode
}

/*
Function to turn keep LEDs on unmodulated at their designated intensities
*/
void Solid_LEDs()
{
  boolean quit = false;
  LEDs_Enable();
  //int cumulative_intensities = 0;
  if (cumulativeIntensities != 0) {
    for (int i = 0; i < 6; i++) analogWrite(spectrum[i], intensities[i]);  // set all of the LEDs on at designated intensity simultaneously (if !SEQ)
  }
  else quit = true; // exit if intensities are all zero...
  while (!quit) {  // keep LED's on until something happens
    quit = On_The_Go_Light_Check();  // routine to evaluate settings and ambient light conditions to see if LEDs should be shut off...
    if (Serial.available()) {
      byte preCycle = cycle;  // keep track of what cycle we're in- need to exit for new operation check if instruction changes cycle
      readInstruction(); //quit = true;  // exit if Serial data is incoming- communications handled in main loop()
      if (cumulativeIntensities != 0) { // update intensities if these were overwritten
        for (int i = 0; i < 6; i++) analogWrite(spectrum[i], intensities[i]);  // set all of the LEDs on at designated intensity simultaneously (if !SEQ)
      }
      else quit = true; // exit if intensities are all zero...
      if ((modulation != SOLID) || (preCycle != cycle)) quit = true; // if modulation scheme changed, exit this modulation scheme
    }
  }
  for (int i = 0; i < 6; i++) digitalWrite(spectrum[i], LOW);  // make sure current setting is zero to each LED before exiting
  lightsOn = false;
  LEDs_Disable();  // disable all LED drivers- lower power mode
}

/* 
 *  function to check any incoming serial traffic, and identify changes in configuration
 *  that would require exiting from Pulse_LEDs or Wave_LEDs (i.e. change in cycle
 */
boolean handleSerialTraffic() {
  boolean finished = On_The_Go_Light_Check(); // check if lighting conditions want us to exit first...
  if (Serial.available()) {// then check for instructions that would require us to exit...
      byte preCycle = cycle;  // keep track of what cycle we're in- need to exit for new operation check if instruction changes cycle
      byte preModulation = modulation;
      float preFrequency = frequency;  // frequency in hertz for LED modulation
      byte preDutyCycle = duty_cycle;
      readInstruction(); //quit = true;  // exit if Serial data is incoming- communications handled in main loop()
      if (cumulativeIntensities == 0) finished = true; // exit if intensities are all zero...
      if ((modulation != preModulation) || (preCycle != cycle)) finished = true; // if modulation scheme changed, exit this modulation scheme
      if (((modulation == PULSE) || (modulation == WAVE)) && (frequency != preFrequency)) finished = true;
      if ((modulation == PULSE) && (duty_cycle != preDutyCycle)) finished = true;
  }
  return finished;
}

void readInstruction() {
    command = Serial.read();
    /*Serial.print("mReporting incoming serial instruction: ");
    Serial.print(command);
    Serial.print('\t');*/
    switch (command) {
    case 'c':  // request from Android to report current device configuration
      Report_Configuration();
      break;
    case 'b':  // read configuration byte programmed from Android
      Read_Config_Byte();
      break;
    case 'd':  // read duty cycle programmed from Android
      duty_cycle = Serial.parseInt();
      break;
    case 'e':  // write full configuration into EEPROM
      Write_Configuration();
      break;
    case 'f':  // read frequency programmed from Android
      frequency = Serial.parseFloat();
      break;
    case 'i':  // command from Android that new intensity value is incoming...
      Intensity_Update();
      break;
    case 'n':  // read night duration programmed by Android
      night_duration = Serial.parseInt();
      break;
    case 'r':  // read sunrise duration programmed by Android
      sunrise_duration = Serial.parseInt();
      break;
    case 's':  // read sunset duration programmed by Android
      sunset_duration = Serial.parseInt();
      break;
    case 't': // use current lighting level as ambient light threshold to determine whether it is day or night
      LEDs_Disable();
      PRR &= 0xfe; // enable clock to ADC so ADC can start operating again (bit 0 of PRR, leave others unchanged)...
      ADCSRA |= 0x80; // enable Analog to Digital Converter (need it to measure ambient light level)
      ALS_On();  // energize ambient light circuit
      delay(10);  // let transients die from turning LEDs off and energizing sensing circuit
      light_threshold = 0;
      for (int summation = 0; summation < 64; summation++) {
        light_threshold += analogRead(1);  // sum up 64 successive 10 bit "light" readings on analog channel 1
        delayMicroseconds(260);  // as a crude digital filter to reject any noise in the system, especially when ambient light is near the threshold
      }
      ROM_Write(ALS_THRESHOLD, light_threshold);
      if (lightsOn) LEDs_Enable();
      ADCSRA &= 0x7f; // disable ADC (only use it once a minute to determine ambient light level- otherwise it's wasting energy...; enable ADC with ADC |= 0x80;
      PRR |= 0x01; // disable clock to ADC / power reduction register (leave other bits unchanged)
      ALS_Off();  // de-energize ambient light measurement
      break;
    default:
      break;
    }
    serialTimeout = millis() + 60000; // don't let device go to sleep within 1 minute of serial communication (facilitate programming device)
}

void setup() {
  lightsOn = false; // by default assume LEDs are off at start...                
  DDRD &= 0x7f;
  DDRD |= 0x7c;  // set D7 as input, D2, D3, D4, D5, D6 as output, and leave 0 & 1 (serial port) unchanged
  DDRB &= 0xce;
  DDRB |= 0x0e;  // set bits 2, 3, & 4 high, or D9, D10, D11 (output), D8, D12, & D13 as input, others on port B unchanged

  DIDR1 |= 0x03;  // digital input buffer disable on ADC0 and ADC1 (save power/ we're not using this hardware)
  ACSR &= 0xf7; // disable interrupt on analog comparator (need to do it before disabling comparator, or interrupt may occur)
  ACSR |= 0x80; // disable analog comparator (save power)
  ADCSRA &= 0x7f; // disable ADC (only use it once a minute to determine ambient light level- otherwise it's wasting energy...; enable ADC with ADCRA |= 0x80;
  PRR = 0xcd; // power reduction register- disable TWI/i2c interface, timer 2, timer 1, and ADC. to also disable SPI use 0xcd (0xc9 to leave SPI enabled); leave on timer 0 (bit 5 = 0) and USART(bit 1 = 0)
    // make sure to enable clock to ADC before enabling ADC; PRR = 0xc8 to enable clock to ADC
  
  //PORTD |= 0x14;  // set D2 and D4 (LED enable and energize photocell bridge)
  Read_Configuration();
  Serial.begin(38400);
  /*Serial.print("$$$");  // enter command mode on bluetooth; commented out- preference is to enter command mode from Android side- because usually bluetooth not powered on startup...
  delay(100);  // delay 65,535/64 ms; about 1 second (otherwise will exit from command mode)
  Serial.print("ST,255\r"); // set bluetooth configuration timer to 255 (never time out- allow device reset remotely through bluetooth by entering command mode and controlling GPIO)
  delay(100);
  Serial.print("SU,57\r");  // set baud rate on bluetooth to 57600 baud
  Serial.begin(57600);  // update MCU baud to same (MCU with 8 MHz clock can't keep up with steady stream of incoming data at 115200 baud
  delay(100);
  Serial.print("---");  // exit bluetooth configuration mode (return to data mode)*/
  dark_count = 0;  // initially don't know if it's light or dark; start counts at 0..
  light_count = 0;
  LEDs_Disable();
  /*modulation = WAVE;  // override EEPROM settings- legacy of debugging
  cycle = TWENTYFOURSEVEN;
  for (int i = 0; i < 6; i++) intensities[i] = 0xff;  
  SEQ = true;
  night_duration = 720;
  sunset_duration = 90;
  sunrise_duration = 120;
  frequency = 4.0;
  duty_cycle = 40;*/
  serialTimeout = millis(); // initialize serial timeout to current time...
  
  WDT_event = true;
  WDT_count = 0;  // set up conditions at start to force ALS_Analysis to check light settings (don't wait for a whole minute)

  //WDTCSR = 0x10; // set Watchdog timer change enable bit (need high to change timer prescalar bits; reverts low after 4 clock cycles so hurry!
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  WDTCSR = 0x05;  // set clock prescalar bits for WDTimer (WDT expires every ~2.048s for 0x07, ~4.096s for 0x20, ~0.512s for 0x05)
  WDTCSR |= 0x40; // enable WDT interrupt...
}

void loop() {
  while (Serial.available()) readInstruction();
  ALS_Analysis();  // Update ambient light information (at appropriate intervals)
  if (cycle != OFF) Check_Operation();  // if configuration is not OFF check to see if LEDs should be turned on and how...
  /*  
   *   Following code only executes if lights are not due to turn on under current settings / predicted location in diurnal cycle
   *   so put MCU in idle mode to save power until ready to check ambient light level and serial port again
  */
  PRR |= 0x48; // power reduction register- disable timer 2, timer 1, and ADC- if we get this far in loop lights are off, don't need timers 1 & 2 (leave others unchanged)
  //if (millis() < (serialTimeout - 60001)) serialTimeout = millis(); // reset serialTimeout when millis() clock wraps available size...
  if (millis() > serialTimeout) goToSleep();
}

void goToSleep(void)
{
  SMCR = 0x01;  // configures sleep as "IDLE", and sleep mode is enabled...
  /* Now enter sleep mode. */
  sleep_mode();
  SMCR = 0x00;  // disable sleep mode... (exits from interrupt here...)

}

// ISR occuring on watchdog timer elapse
ISR (WDT_vect) {
  SMCR = 0x00;  // bring MCU out of sleep mode to handle stuff!
  WDT_event = true;
  WDT_count++;
  WDT_count %= 117;  // increment WDT count and take modulus 117 (cycles count ~ every minute for 0.512 second timeout)
  /*if (!WDT_count) {
      LEDs_Enable();
      digitalWrite(RED, HIGH);
      delay(2);
      digitalWrite(RED, LOW);
      LEDs_Disable();
  } // just an indication of an expired minute...*/
}

