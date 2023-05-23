
//Firmware written for "D1 master lock 1 MHz loop filter eom driver RE047-01"
//written by Max Kolanz
//For hardware and schematic diagram see newRb oneNote page
#include <Wire.h> //for I2C communication with screen
#include <EEPROM.h> //to save the parameters
#include "SparkFun_5P49V60.h" //sparkfun clock gen library. 
//!!!!make sure to get the updated files off of github comments becuase master branch code doesn't work!!!
//I also made some changes myself so I'll just post the files on onenote too. put them in the src directory

//pinout definitions
#define AMP_ROT_B 23        //phase rotary encode B pin
#define AMP_ROT_A 22        //phase rotary encode A pin
#define AMP_ROT_SW 13       //phase rotary encode SW pin
#define AMP_ROT_LEDR 14      //phase rotary encode red LED pin
#define AMP_ROT_LEDB 16     //phase rotary encode blue LED pin
#define AMP_ROT_LEDG 15    //phase rotary encode green LED pin

#define PHASE_ROT_B 20        //phase rotary encode B pin
#define PHASE_ROT_A 17        //phase rotary encode A pin
#define PHASE_ROT_SW 6        //phase rotary encode SW pin
#define PHASE_ROT_LEDR 21      //phase rotary encode red LED pin
#define PHASE_ROT_LEDB 30     //phase rotary encode blue LED pin
#define PHASE_ROT_LEDG 28    //phase rotary encode green LED pin

//In the future should use a converter to reduce the number of digital pins needed for attenuators
#define DRIVE_ATTENUATOR_HALF 0       //0.5 dB digital pin for EOM driver
#define DRIVE_ATTENUATOR_ONE 1        //1 dB digital pin for EOM driver
#define DRIVE_ATTENUATOR_TWO 2        //2 dB digital pin for EOM driver
#define DRIVE_ATTENUATOR_FOUR 3       //4 dB digital pin for EOM driver
#define DRIVE_ATTENUATOR_EIGHT 4      //8 dB digital pin for EOM driver
#define DRIVE_ATTENUATOR_SIXTEEN 5    //16 dB digital pin for EOM driver

#define DEMOD_ATTENUATOR_HALF 7       //0.5 dB digital pin for mixer
#define DEMOD_ATTENUATOR_ONE 8        //1 dB digital pin for EOM mixer
#define DEMOD_ATTENUATOR_TWO 9        //2 dB digital pin for EOM mixer
#define DEMOD_ATTENUATOR_FOUR 10      //4 dB digital pin for EOM mixer
#define DEMOD_ATTENUATOR_EIGHT 11     //8 dB digital pin for EOM mixer
#define DEMOD_ATTENUATOR_SIXTEEN 12   //16 dB digital pin for EOM mixer

#define CLOCK_GEN_ON_OFF 26           //When the global reset switch is used we need to call global reset


// RGB LED colors (for common anode LED, 0 is on, 1 is off)
#define OFF B111
#define RED B110
#define GREEN B101
#define YELLOW B100
#define BLUE B011
#define PURPLE B010
#define CYAN B001
#define WHITE B000
#define PHASE_ID B1
#define AMP_ID B0

#define DISPLAY_ADDRESS1 0x72 //This is the default address of the OpenLCD
#define CLOCKGEN_ADDRESS1 0x6A

SparkFun_5P49V60 clockGen;

void      setLED(unsigned char, unsigned char);   //sets the rotary encoder color as defined above
uint8_t   readAmpData(bool);                      //returns the integer representing the attenuation from EEPROM
float     getAmpSetting(bool);                    //converts the attenuation to a float to print to the LCD screen
void      incrementAmp(bool);                     //checks bundary conditions and increments the attenuation by 0.5
void      writeAmpData(uint8_t, bool);            //writes the attenuation integer to the EEPROM to be recovered on boot up
void      updateAmpScreen(bool);                  //writes the value of the attenuation to the LCD
void      updateAttenuators(bool);                //Updates the pins that go to attenuators with the integer representation
void      writePhaseData(uint16_t);               //Writes the integer representing the phase to the EEPROM
uint16_t  readPhaseData();                        //returns the integer representing phase from the EEPROM
void      setPhase();                             //Gives the command to skew the phase of channels 1 and 2 on the clock gen
void      incrementPhase();                       //Changes the phase based on the encoder input
void      displayMultiplier();                    //when changing the phase display the multipler on the LCD, otherwise blank it out
void      clearDisplayMultiplier();               //the opposite of above
void      updatePhaseScreen(int phase_setting);   //displays the phase on the LCD  

static uint8_t  adjustment_multiplier = 1;    //to change how fast the phase is incremented by 

static uint8_t demod_attenuation;        //tens place attenuation for demod 
static uint8_t drive_attenuation;        //ones place attenuation for demod 

void setup() {
  // put your setup code here, to run once:
  //initialize pins as defined above

  pinMode(AMP_ROT_B, INPUT);
  //digitalWrite(AMP_ROT_B, HIGH); //USES EXTERNAL PULLUP FROM TEENSY PIN 24
  pinMode(AMP_ROT_A, INPUT);
  //digitalWrite(AMP_ROT_A, HIGH); //USES EXTERNAL PULLUP FROM TEENSY PIN 24
  pinMode(AMP_ROT_SW, INPUT);
  // The rotary switch is common anode with external pulldown, write pin low for LED on
  pinMode(AMP_ROT_LEDR, OUTPUT);
  pinMode(AMP_ROT_LEDG, OUTPUT);
  pinMode(AMP_ROT_LEDB, OUTPUT);
  
  pinMode(PHASE_ROT_B, INPUT);
  //digitalWrite(ROT_B, HIGH); //USES EXTERNAL PULLUP FROM TEENSY PIN 24
  pinMode(PHASE_ROT_A, INPUT);
  //digitalWrite(ROT_A, HIGH); //USES EXTERNAL PULLUP FROM TEENSY PIN 24
  pinMode(PHASE_ROT_SW, INPUT);
  // The rotary switch is common anode with external pulldown, write pin low for LED on
  pinMode(PHASE_ROT_LEDR, OUTPUT);
  pinMode(PHASE_ROT_LEDG, OUTPUT);
  pinMode(PHASE_ROT_LEDB, OUTPUT);

  pinMode(DRIVE_ATTENUATOR_HALF, OUTPUT);
  pinMode(DRIVE_ATTENUATOR_ONE, OUTPUT);
  pinMode(DRIVE_ATTENUATOR_TWO, OUTPUT);
  pinMode(DRIVE_ATTENUATOR_FOUR, OUTPUT);
  pinMode(DRIVE_ATTENUATOR_EIGHT, OUTPUT);
  pinMode(DRIVE_ATTENUATOR_SIXTEEN, OUTPUT);

  pinMode(DEMOD_ATTENUATOR_HALF, OUTPUT);
  pinMode(DEMOD_ATTENUATOR_ONE, OUTPUT);
  pinMode(DEMOD_ATTENUATOR_TWO, OUTPUT);
  pinMode(DEMOD_ATTENUATOR_FOUR, OUTPUT);
  pinMode(DEMOD_ATTENUATOR_EIGHT, OUTPUT);
  pinMode(DEMOD_ATTENUATOR_SIXTEEN, OUTPUT);

  pinMode(CLOCK_GEN_ON_OFF, INPUT);                         //interrupt (see below)

  attachInterrupt(AMP_ROT_SW, ampButtonIRQ, CHANGE);        //handles button press on amplitude encoder
  attachInterrupt(PHASE_ROT_SW, phaseButtonIRQ, CHANGE);    //handles button press on phase encoder
  attachInterrupt(AMP_ROT_A, ampRotaryIRQ, CHANGE);         //handles rotation on amplitude encoder
  attachInterrupt(PHASE_ROT_A, phaseRotaryIRQ, CHANGE);     //handles rotation on phase encoder
  attachInterrupt(CLOCK_GEN_ON_OFF, resetClock, FALLING);   //handles "function gen" (clock gen) on/off switch

  Serial.begin(9600); // Use serial for debugging

  Wire.begin(); //Join the bus as master
  Wire.setSDA(18);
  Wire.setSCL(19);
  
  if (clockGen.begin() == true){
    Serial.println("Clock Generator Ready.");
  }
  else {
    Serial.println("Could not communicate with the SparkFun Clock Generator.");
    while(1);
  }

  static float CLOCK_GEN_FREQUENCY  = 4.2;
  // Fist, Setting the internal oscillator to a known value.
  //The VCO frequency should be an integer multiple of 16 (in this case 16 * 157)
  //The output frequnecy has the least amount of noise when it is an integer multiple of the VCO frequency.
  //4.2 is fine after looking at the singal on spectrum analyzer. (70dB from central peak power)-->see one note  
  Serial.println("Setting Internal Clock Frequency to 2512MHz.");
  clockGen.setVcoFrequency(2512.0); // Give float value in MHz 

  // Clock One----------------------------------------------------
  // Use internal phase lock loop for clock output calculation.
  clockGen.muxPllToFodOne();
  Serial.println("Setting Output Mode for Clock One to CMOS.");
  clockGen.clockOneConfigMode(CMOS_MODE);//for 50 ohm termination
  Serial.println("Setting Clock One Frequency to 4MHz.");
  clockGen.setClockOneFreq(CLOCK_GEN_FREQUENCY); // Give float value in MHz, 4.0 = 4000000Hz or 4MHz
  // --------------------------------------------------------------

  // Clock Two--------------------------------------------------
  // Use internal phase lock loop for clock output calculation.
  clockGen.muxPllToFodTwo();
  Serial.println("Setting Output Mode for Clock Two to CMOS.");
  clockGen.clockTwoConfigMode(CMOS_MODE);//for 50 ohm termination
  Serial.println("Setting Clock Two Frequency to 4MHz.");
  clockGen.setClockTwoFreq(CLOCK_GEN_FREQUENCY); // Give float value in MHz, 4.0 = 4000000Hz or 4MHz

  clockGen.skewClockOneByDegrees(static_cast<double>(readPhaseData()));

  drive_attenuation = readAmpData(true);
  demod_attenuation = readAmpData(false);//initialize these form EEPROM
  Serial.print("begin drive setting: ");Serial.println(getAmpSetting(true));
  Serial.print("begin demod setting: ");Serial.println(getAmpSetting(false));  
  //program attenuators
  updateAttenuators(true);
  updateAttenuators(flase);


  //Start sreen message
    setLED(WHITE, AMP_ID); //flash LED on startup
    setLED(WHITE, PHASE_ID);
    Wire.beginTransmission(DISPLAY_ADDRESS1);
    Wire.write(254);
    Wire.write(128 + 0 + 0);//first line position zero
    Wire.print("  Master Lock   ");//first line
    Wire.endTransmission(); delay(2);

    Wire.beginTransmission(DISPLAY_ADDRESS1);
    Wire.write(254);
    Wire.write(128 + 64 + 0);//first line position zero
    Wire.print("    RC047-01    ");//For 16x2 LCDs
    Wire.endTransmission();
    delay(3000);

    setLED(OFF, PHASE_ID);
    setLED(OFF, AMP_ID);
    
    Wire.beginTransmission(DISPLAY_ADDRESS1);
    Wire.write(254); //Put LCD into setting mode
    Wire.write(128 + 0 + 0);//first line position zero
    Wire.print("     dB       dB");//first line
    Wire.endTransmission();
    delay(2);
    Wire.beginTransmission(DISPLAY_ADDRESS1);
    Wire.write(254);
    Wire.write(128 + 64 + 0);//first line position zero
    Wire.print("    degree      ");//For 16x2 LCDs
    Wire.endTransmission();
    delay(2);
    
    updatePhaseScreen();
    updateAmpScreen(true); updateAmpScreen(false);
    Serial.println("End of screen initialization");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "Static" variables are initalized once the first time the loop runs, but keep their values through successive loops.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static boolean amp_button_down = false;
static boolean phase_button_down = false;
static unsigned long int amp_button_down_start, amp_button_down_time, phase_button_down_start, phase_button_down_time;
static unsigned long int amp_change_mode_show_number = 0;
static unsigned long int amp_change_mode_hide_number = 0; //used to strobe the selected value on the screen with above
//above are all used in the loop to catch interrupt flags below are additional flags for added functionality

bool change_amp = false;   //flag for user changing value of demod or drive amplitude
bool change_phase = false;   //flag for user changing value of phase amplitude
bool cursor_pos = false;        //tracks whether cahnging drive or phase attenuation

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Global variables for interrupt routines
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
volatile int amp_rotary_counter = 0;                // current "position" of amplitude rotary encoder (increments CW)
volatile int phase_rotary_counter = 0;              // current "position of phase rotart encode (increments CW)
volatile int amp_up_or_down = 0;                    // to change the value of the digit
volatile int phase_up_or_down = 0;                    // to change the value of the digit
volatile boolean amp_rotary_change = false;         // will turn true if amp_rotary_counter has changed
volatile boolean phase_rotary_change = false;       // will turn true if phase_rotary_counter has changed
volatile boolean amp_button_pressed = false;        // will turn true if the amp button has been pushed
volatile boolean phase_button_pressed = false;      // will turn true if the phase button has been pushed
volatile boolean amp_button_released = false;       // will turn true if the amp button has been released (sets button_downtime)
volatile boolean phase_button_released = false;     // will turn true if the phase button has been released (sets button_downtime)
volatile unsigned long amp_button_downtime = 0L;    // ms the amplitude button was pushed before release
volatile unsigned long phase_button_downtime = 0L;  // ms the phase button was pushed before release




void ampButtonIRQ()
{
  // Process rotary encoder button presses and releases, including
  // debouncing (extra "presses" from noisy switch contacts).
  // If button is pressed, the button_pressed flag is set to true.
  // (Manually set this to false after handling the change.)
  // If button is released, the button_released flag is set to true,
  // and button_downtime will contain the duration of the button
  // press in ms. (Set this to false after handling the change.)

  // The pin state of the button being pushed is high

  static boolean button_state = false;
  static unsigned long start, end;

  if ((digitalRead(AMP_ROT_SW) == HIGH) && (button_state == false))
    // Button was up, but is currently being pressed down
  {
    // Discard button presses too close together (debounce)
    start = millis();
    if (start > (end + 10)) // 10ms debounce timer
    {
      button_state = true;
      amp_button_pressed = true;
    }
  }
  else if ((digitalRead(AMP_ROT_SW) == LOW) && (button_state == true))
    // Button was down, but has just been released
  {
    // Discard button releases too close together (debounce)
    end = millis();
    if (end > (start + 10)) // 10ms debounce timer
    {
      button_state = false;
      amp_button_released = true;
      amp_button_downtime = end - start;
    }
  }
}

void phaseButtonIRQ()
{
  // Process rotary encoder button presses and releases, including
  // debouncing (extra "presses" from noisy switch contacts).
  // If button is pressed, the button_pressed flag is set to true.
  // (Manually set this to false after handling the change.)
  // If button is released, the button_released flag is set to true,
  // and button_downtime will contain the duration of the button
  // press in ms. (Set this to false after handling the change.)

  // Raw information from PinChangeInt library:

  // Serial.print("pin: ");
  // Serial.print(PCintPort::arduinoPin);
  // Serial.print(" state: ");
  // Serial.println(PCintPort::pinState);

  static boolean button_state = false;
  static unsigned long start, end;

  if ((digitalRead(PHASE_ROT_SW) == HIGH) && (button_state == false))
    // Button was up, but is currently being pressed down
  {
    // Discard button presses too close together (debounce)
    start = millis();
    if (start > (end + 10)) // 10ms debounce timer
    {
      button_state = true;
      phase_button_pressed = true;
    }
  }
  else if ((digitalRead(PHASE_ROT_SW) == LOW) && (button_state == true))
    // Button was down, but has just been released
  {
    // Discard button releases too close together (debounce)
    end = millis();
    if (end > (start + 10)) // 10ms debounce timer
    {
      button_state = false;
      phase_button_released = true;
      phase_button_downtime = end - start;
    }
  }
}

void ampRotaryIRQ()
{
  // Process input from the rotary encoder.
  // The rotary "position" is held in rotary_counter, increasing for CW rotation (changes by one per detent).
  // If the position changes, rotary_change will be set true. (You may manually set this to false after handling the change).

  // This function will automatically run when rotary encoder input A transitions in either direction (low to high or high to low)
  // By saving the state of the A and B pins through two interrupts, we'll determine the direction of rotation
  // int rotary_counter will be updated with the new value, and boolean rotary_change will be true if there was a value change
  // Based on concepts from Oleg at circuits@home (http://www.circuitsathome.com/mcu/rotary-encoder-interrupt-service-routine-for-avr-micros)
  // Unlike Oleg's original code, this code uses only one interrupt and has only two transition states;
  // it has less resolution but needs only one interrupt, is very smooth, and handles switchbounce well.

  static unsigned char rotary_state = 0; // current and previous encoder states

  rotary_state <<= 2;  // remember previous state
  rotary_state |= (digitalRead(AMP_ROT_A) | (digitalRead(AMP_ROT_B) << 1));  // mask in current state
  rotary_state &= 0x0F; // zero upper nybble

  //Serial.println(rotary_state,HEX);

  if (rotary_state == 0x06)//0x09) // from 10 to 01, increment counter. Also try 0x06 if unreliable
  {
    amp_rotary_counter++;
    amp_up_or_down = 1;
    amp_rotary_change = true;
  }
  else if (rotary_state == 0x0C)//0x03) // from 00 to 11, decrement counter. Also try 0x0C if unreliable
  {
    amp_rotary_counter--;
    amp_up_or_down = -1;
    amp_rotary_change = true;
  }
}

void phaseRotaryIRQ()
{
  // Process input from the rotary encoder.
  // The rotary "position" is held in rotary_counter, increasing for CW rotation (changes by one per detent).
  // If the position changes, rotary_change will be set true. (You may manually set this to false after handling the change).

  // This function will automatically run when rotary encoder input A transitions in either direction (low to high or high to low)
  // By saving the state of the A and B pins through two interrupts, we'll determine the direction of rotation
  // int rotary_counter will be updated with the new value, and boolean rotary_change will be true if there was a value change
  // Based on concepts from Oleg at circuits@home (http://www.circuitsathome.com/mcu/rotary-encoder-interrupt-service-routine-for-avr-micros)
  // Unlike Oleg's original code, this code uses only one interrupt and has only two transition states;
  // it has less resolution but needs only one interrupt, is very smooth, and handles switchbounce well.

  static unsigned char rotary_state = 0; // current and previous encoder states

  rotary_state <<= 2;  // remember previous state
  rotary_state |= (digitalRead(PHASE_ROT_A) | (digitalRead(PHASE_ROT_B) << 1));  // mask in current state
  rotary_state &= 0x0F; // zero upper nybble

  //Serial.println(rotary_state,HEX);

  if (rotary_state == 0x06) //0x09) // from 10 to 01, increment counter. Also try 0x06 if unreliable
  {
    phase_rotary_counter++;
    phase_up_or_down = 1;
    phase_rotary_change = true;
  }
  else if (rotary_state == 0x0C) //0x03) // from 00 to 11, decrement counter. Also try 0x0C if unreliable
  {
    phase_rotary_counter--;
    phase_up_or_down = -1;
    phase_rotary_change = true;
  }
}

void resetClock(){clockGen.globalReset();}//if this isn't called the signal gets noisey!!!!!!

void loop()
{
  // The rotary IRQ sets the flag rotary_counter to true
  // if the knob position has changed. We can use this flag
  // to do something in the main loop() each time there's
  // a change. We'll clear this flag when we're done, so
  // that we'll only do this if() once for each change.

  if (amp_rotary_change)
  {
    amp_rotary_change = false; // Clear flag

    //only change the number if in change mode
    if(change_amp){
      incrementAmp(!cursor_pos);
    }
  }

  if (phase_rotary_change)
  {
    phase_rotary_change = false; // Clear flag

    //only change the number if in change mode
    if(change_phase){
        incrementPhase();
        updatePhaseScreen();
        //clockGen.skewClockOne(readPhaseData());
        //had to write custom function in clock gen library that goes by phase so this is kindof pointless
        clockGen.skewClockOneByDegrees(static_cast<double>(readPhaseData()));
    }
  }

  // The button IRQ also sets flags to true, one for
  // button_pressed, one for button_released. Like the rotary
  // flag, we'll clear these when we're done handling them.

  if (amp_button_pressed)
  {
    amp_button_pressed = false; // Clear flag

    // We'll set another flag saying the button is now down
    // this is so we can keep track of how long the button
    // is being held down. (We can't do this in interrupts,
    // because the button state is not changing).

    amp_button_down = true;
    amp_button_down_start = millis();
  }
  
  if (phase_button_pressed)
  {
    phase_button_pressed = false; // Clear flag

    // We'll set another flag saying the button is now down
    // this is so we can keep track of how long the button
    // is being held down. (We can't do this in interrupts,
    // because the button state is not changing).

    phase_button_down = true;
    phase_button_down_start = millis();
  }
  
  ////////////////////////////////////////////////////
  //set led color to blue to indicate that the user is in amplitude toggle mode and
  //flash the selection to indicate whether drive or demod is being changed
  if(change_amp){
    setLED(CYAN,AMP_ID);
    if((millis() - amp_change_mode_show_number) > 300){
      printSpace(!cursor_pos);
      amp_change_mode_show_number = millis();
      amp_change_mode_hide_number = millis();
    }
    else if((millis() - amp_change_mode_hide_number) > 100){
      updateAmpScreen(!cursor_pos);
      amp_change_mode_hide_number = millis();
    }
  }
  else{setLED(OFF,AMP_ID);}
  if(change_phase){setLED(CYAN,PHASE_ID);displayMultiplier();}
  else{setLED(OFF,PHASE_ID);}
  ///////////////////////////////////////////////////
  
  if (amp_button_released)
  {
    amp_button_released = false; // Clear flag
    
    if(amp_button_down_time > 1000){
      change_amp = !change_amp;
      if(change_amp){amp_change_mode_show_number = millis();}
      else{updateAmpScreen(true);updateAmpScreen(false);}//so screen doesn't get stuck on blank
    }
    
    // Clear our button-being-held-down flag
    amp_button_down = false;
    //blink when you release the button press
    if(change_amp && (amp_button_downtime <= 1000)){
      cursor_pos = !cursor_pos;
      updateAmpScreen(cursor_pos);//so doesn't get stuck in blank mode on screen
    }
  }

  if (phase_button_released)
  {
    phase_button_released = false; // Clear flag
    
    if(phase_button_down_time > 1000){
      if(change_phase){clearDisplayMultiplier();}//were toggling but aren't now
      change_phase = !change_phase; 
      adjustment_multiplier = 1;
    }
    
    // Clear our button-being-held-down flag
    phase_button_down = false;
    //blink when you release the button press

    //if the user presses the phase button shortly it changes the adjustment multiplier
    if(change_phase && (phase_button_downtime <= 1000)){
        if(adjustment_multiplier == 1){adjustment_multiplier = 2;}
        else if(adjustment_multiplier == 2){adjustment_multiplier = 4;}
        else{adjustment_multiplier = 1;}
    }
  }


  // Now we can keep track of how long the button is being
  // held down, and perform actions based on that time.
  // This is useful for "hold down for five seconds to power off"
  // -type functions.

  if (amp_button_down)
  {
    amp_button_down_time = millis() - amp_button_down_start;
    if(amp_button_down_time > 1000)//if button held down for longer than one second
    {
      for (int i = 0; i < 4; i++) {//blink green
        setLED(GREEN, AMP_ID);
        delay(50);
        setLED(OFF, AMP_ID);
        delay(50);
      }
    }
    //if we are switching between drive and demod
    else if(change_amp){
      setLED(WHITE,AMP_ID);
    }
  }

  if (phase_button_down)
  {
    phase_button_down_time = millis() - phase_button_down_start;
    if(phase_button_down_time > 1000)//if button held down for longer than one second
    {
      for (int i = 0; i < 4; i++) {//blink green
        setLED(BLUE, PHASE_ID);
        delay(50);
        setLED(OFF, PHASE_ID);
        delay(50);
      }
    }
    //if we are switching to a new adjustment_multiplier
    else if(change_phase){
      setLED(WHITE,PHASE_ID);
      delay(50);
    }
  } 
}

void incrementPhase(){
  int phase_integer = readPhaseData();
  if(phase_up_or_down==1){
    phase_integer += (adjustment_multiplier);
    if(phase_integer > 360){phase_integer = phase_integer%360;}
  }
  else if(phase_up_or_down == -1){
    phase_integer -= adjustment_multiplier;
    if(phase_integer < 0){
      phase_integer += 360;
    }
  }
  writePhaseData(phase_integer);
}

void updatePhaseScreen(){
   int value = readPhaseData();
   //value = round_to_digits(double(readPhaseData() * smallest_phase_change), 4);
   //I used to display phase as float but changed to int
   Wire.beginTransmission(DISPLAY_ADDRESS1);
   Wire.write(254);
   Wire.write(128 + 64 + 0);
   if(value >= 100){Wire.print(value);}
   else if(value >= 10){Wire.print(" ");Wire.print(value);}
   else{Wire.print("  ");Wire.print(value);}// so the position of the number doesn't change based on the value
   Wire.endTransmission();delay(2);  
}

void displayMultiplier(){
   Wire.beginTransmission(DISPLAY_ADDRESS1);
   Wire.write(254);
   Wire.write(128 + 64 + 11);//write to second row(64) position 14
   Wire.print("step");Wire.print(adjustment_multiplier);
   Wire.endTransmission();delay(2);  
}

void clearDisplayMultiplier(){
Wire.beginTransmission(DISPLAY_ADDRESS1);
   Wire.write(254);
   Wire.write(128 + 64 + 10);//write to second row(64) position 10
   Wire.print("      ");//just two spaces
   Wire.endTransmission();delay(2);    
}

//reading and writing data to the EEPROM is done by changing individual bits in the memory
void writePhaseData(uint16_t setting){
  //the amplitudes were two 6 bit numbers so start writing the phase number in EEPROM spot 12
  bool one = 1;
  bool zero = 0; //EEPROM function needs to know the data type? so do this to avoid confusion
  for(int i = 0; i < 16; i++){
    if((setting >> i) & 1){EEPROM.put(27 - i, one);}//least significant is spot 27 MS is 12
    else{EEPROM.put(27 - i, zero);}
  }
}


uint16_t  readPhaseData(){
  uint16_t total = 0;
  int BIT = 0;
  for(int i=0; i < 16; i++){
    BIT = EEPROM.read(i+12);//first 12 bits taken by attenuations
    if(BIT!=0){total = total + (1 << (15-i));}
  }
  return total;
}




void writeAmpData(uint8_t setting, bool drive_setting){
    //integer is 6 bit resolution and the bool is to set the drive (true) or demod(flase)
    bool one = 1;
    bool zero = 0;//EEPROM function needs to know the data type? so do this to avoid confusion
    int address_start;
    if(drive_setting){address_start=5;}else{address_start=11;}
    for (int i = 0; i < 6; i++) {
        int k = setting >> i;
        if(k & 1){EEPROM.put(address_start-i, one);}//number goes in backwards
        else{EEPROM.put(address_start-i, zero);}
    }
}

//reads the places in the EEprom and converts the 12 bits of binary to the integer
uint8_t readAmpData(bool drive_setting){
  int total = 0;
  int BIT = 0;
  int address_start;
  if(drive_setting){address_start = 0;}else{address_start = 6;}
  for(int i=0; i < 6; i++){
    BIT = EEPROM.read(i+address_start);
    if(BIT!=0){total = total + (1 << (5-i));}
  }
  return total;
}

void incrementAmp(bool drive_setting){
  if(drive_setting){
    if((amp_up_or_down == 1) && (drive_attenuation < 63)){
      drive_attenuation += 1;
    }
    else if((amp_up_or_down == -1) && drive_attenuation != 0){
      drive_attenuation -= 1;
    }
    writeAmpData(drive_attenuation, true);
    updateAmpScreen(true);
    updateAttenuators(true);
  }
  else{
    if((amp_up_or_down == 1) && (demod_attenuation < 63)){
      demod_attenuation += 1;
    }
    else if((amp_up_or_down == -1) && (demod_attenuation != 0)){
      demod_attenuation -= 1;
    }
    writeAmpData(demod_attenuation, false);
    updateAmpScreen(false);
    updateAttenuators(false);
  }
  
}

void updateAmpScreen(bool drive_setting){
   int screen_position = 0;  
   Wire.beginTransmission(DISPLAY_ADDRESS1);
   Wire.write(254);
   if(drive_setting){
      Wire.write(128 + 0 + screen_position);// 0 is the first row
      Wire.print(getAmpSetting(true),1);
   }  
   else{
      screen_position = 9;
      Wire.write(128 + 0 + screen_position);
      Wire.print(getAmpSetting(false),1);
   }
   Wire.endTransmission();delay(2);
}

void updateAttenuators(bool drive_setting){
  //high turns off attenuator
  if(drive_setting){
    //could do this in less lines but I think this is faster
      digitalWrite(DRIVE_ATTENUATOR_HALF,    !(drive_attenuation & 1));
      digitalWrite(DRIVE_ATTENUATOR_ONE,     !((drive_attenuation >> 1) & 1));
      digitalWrite(DRIVE_ATTENUATOR_TWO,     !((drive_attenuation >> 2) & 1));
      digitalWrite(DRIVE_ATTENUATOR_FOUR,    !((drive_attenuation >> 3) & 1));
      digitalWrite(DRIVE_ATTENUATOR_EIGHT,   !((drive_attenuation >> 4) & 1));
      digitalWrite(DRIVE_ATTENUATOR_SIXTEEN, !((drive_attenuation >> 5) & 1));   
  }
  else{
    //could do this in less lines but I think this is faster
      digitalWrite(DEMOD_ATTENUATOR_HALF,     !(demod_attenuation & 1));
      digitalWrite(DEMOD_ATTENUATOR_ONE,     !((demod_attenuation >> 1) & 1));
      digitalWrite(DEMOD_ATTENUATOR_TWO,     !((demod_attenuation >> 2) & 1));
      digitalWrite(DEMOD_ATTENUATOR_FOUR,    !((demod_attenuation >> 3) & 1));
      digitalWrite(DEMOD_ATTENUATOR_EIGHT,   !((demod_attenuation >> 4) & 1));
      digitalWrite(DEMOD_ATTENUATOR_SIXTEEN, !((demod_attenuation >> 5) & 1));   
  }
}

float getAmpSetting(bool drive_setting){
  //this is not general purpose but only works for this specific attenuator and 6 bit number
  float setting = 00.0;
  if(drive_setting){
    setting += ((drive_attenuation >> 1)/10);
    setting += ((drive_attenuation >> 1) - ((drive_attenuation >> 1)/10));
    setting += (int(drive_attenuation & 0x01)*0.5);
  }
  else{
    setting += ((demod_attenuation >> 1)/10);
    setting += ((demod_attenuation >> 1) - ((demod_attenuation >> 1)/10));
    setting += (int(demod_attenuation & 0x01)*0.5);
  }
  return setting;
}

void setLED(unsigned char color, unsigned char switch_id)
// Set RGB LED to one of eight colors (see #defines above)
{
  if(switch_id){//phase is id = 1
    digitalWrite(PHASE_ROT_LEDR, color & B001);
    digitalWrite(PHASE_ROT_LEDG, color & B010);
    digitalWrite(PHASE_ROT_LEDB, color & B100);
  }
  else{
    digitalWrite(AMP_ROT_LEDR, color & B001);
    digitalWrite(AMP_ROT_LEDG, color & B010);
    digitalWrite(AMP_ROT_LEDB, color & B100);
  }
}

//using this to flash the attenuation when modifying. 
void printSpace(bool drive_setting){//to print blank spaces on the screen
  Wire.beginTransmission(DISPLAY_ADDRESS1);
  Wire.write(254);
  if(drive_setting){Wire.write(128 + 0 + 0);}
  else{Wire.write(128 + 0 + 9);}
  Wire.print("    ");
  Wire.endTransmission();
  delay(2);
}
