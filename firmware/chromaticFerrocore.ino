/*
	Chromatic Ferrocore Control	
	Reuben Strangelove
	3/10/2017
	
	Description:
		This is a simplified user interface and auto-control system
		for the highly experimental chromatic ferrocore. 
	
	Microcontroller:
		Arduino Nano (ATmega328)
		
	Hardware:
		PCA9685 I2C PWM controller
		NeoPixel LEDs (WS2811b)
		SG90 analog servo
		
	Notes:
		This is prototype for a future panel 9x the size!
*/

// operational parameters
#define TIME_UNTIL_SYSTEM_STATE_AUTO 10000 // ms
#define TIME_BETWEEN_PROCESSESS 10 //ms - delay between executing processes

// system parameters
#define BUTTON_PRESSED 0
#define TOGGLE_ON 0
#define TOGGLE_OFF 1

// hardware pins
#define P_INTENSITY_PIN A0
#define T_AUTO_PIN 11
#define B_FLUSH_PIN 10
#define B_FILTER1_PIN 6
#define B_FILTER2_PIN 4
#define B_FILTER3_PIN 5
#define SERVO_PIN 9

#define SERVO_POS_RED 104
#define SERVO_POS_GREEN 71
#define SERVO_POS_BLUE 45
#define SERVO_POS_UP 168

#define BRIGHTNESS_BACKLIGHT 3000
#define INDICATOR_MAX 4094 // see notes below for the why of these values
#define INDICATOR_OFF 4095 // see notes below for the why of these values


// Library for the PCA9685 16-bit PWM driver
// https://github.com/NachtRaveVL/PCA9685-Arduino
// Required library fixes:
// Around line 91 change to: _i2cAddress = PCA9685_I2C_BASE_ADDRESS | ((i2cAddress & 0x3F) << 0);
// Around line 193 change to: if (begChannel + numChannels > 16) numChannels -= (begChannel + numChannels) - 16;

#include <Wire.h>

// Library for the PCA9685 16-bit PWM driver
// https://github.com/NachtRaveVL/PCA9685-Arduino
// Required library fixes:
// Around line 91 change to: _i2cAddress = PCA9685_I2C_BASE_ADDRESS | ((i2cAddress & 0x3F) << 0);
// Around line 193 change to: if (begChannel + numChannels > 16) numChannels -= (begChannel + numChannels) - 16;
#include "PCA9685.h" // the PCA9685 16-bit PWM driver
// the following pwm input values have these effects:
// pwm = 0 : ON
// pwm = 1 : most dim
// pwm = 4094 : most bright
// pwm = 4095 : OFF
PCA9685 pwmController(Wire, PCA9685_PhaseBalancer_None);  // Phase balance causes flickering
PCA9685 pwmController2(Wire, PCA9685_PhaseBalancer_None); // Phase balance causes flickering
word pwms[2][16];
word pwmsOldValue[2][16];

// NeoPixel (WS2811b LED Strips)
#include <Adafruit_NeoPixel.h>
// numLeds, PIN, speed
#define NUM_PIXELS_CORE 9
Adafruit_NeoPixel stripArrow = Adafruit_NeoPixel(2, 2, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripCore = Adafruit_NeoPixel(NUM_PIXELS_CORE, 3, NEO_GRB + NEO_KHZ800);


#include <PWMServo.h>
// http://arduiniana.org/libraries/pwmservo/
PWMServo muxServo;


// setup system
// system operational state
enum systemStateEnum {MANUAL, AUTO, DEMO};
enum systemStateEnum systemState = MANUAL;
// system status
int systemStatus;
int systemStatusBuffer;
int systemStateAutoCounter; // system into AUTO by lack of user input helper
int systemStatusWaitUntilIncrease;

int coreFillState;

// setup indicators
enum indicatorStateEnum {OFF, SLOW, MED, FAST, RAND, ON, NUM_INDICATOR_STATES};
enum indicatorNames {I_F1, I_F2, I_F3, I_FLUSH, I_RED, I_GREEN, I_BLUE, I_AMBER, I_W1, I_W2, I_W3, I_BACKLIGHT, NUM_INDICATORS}; // must match NUM_INDICATORS
enum indicatorStateEnum indicatorState[NUM_INDICATORS] = {OFF};
signed int indicatorValues[NUM_INDICATORS]; // needs to be signed for brightness tests
static int indicatorBlinkCounter[NUM_INDICATORS]; // indicator blink helper
static int indicatorBlinkState[NUM_INDICATORS];  // indicator blink helper


// setup functions (function states vary between functions and use ints for state names)
enum functionNames {F_BOOTUP, F_FILTER1, F_FILTER2, F_FILTER3, F_FLUSH, F_CRASH, NUM_FUNCTIONS};
int functionState[NUM_FUNCTIONS];
int functionCounter[NUM_FUNCTIONS];


// setup LED strips
int stripArrowColorRGB[3];
enum stripArrowColorNames {STRIP_ARROW_OFF, STRIP_ARROW_RED, STRIP_ARROW_GREEN, STRIP_ARROW_BLUE, NUM_STRIP_ARROW_COLORS};
int stripArrowColor;

signed int pot;
int potTarget; // used in AUTO systemState
int potOld;



// buttons
enum buttonStates {B_FILTER1, B_FILTER2, B_FILTER3, B_FLUSH, NUM_BUTTONS};
enum buttonStates button[NUM_BUTTONS];
int buttonOld[NUM_BUTTONS];
bool buttonReleasedFlag[NUM_BUTTONS];
int buttonDebounceCounter[NUM_BUTTONS];

// toggles
enum toggleStates {T_AUTO, NUM_TOGGLES};
enum toggleStates toggle[NUM_TOGGLES];
int toggleOld[NUM_TOGGLES];

// LED strip
// Chromatic Core
signed int stripColor[NUM_PIXELS_CORE];
int stripCounter[NUM_PIXELS_CORE];
int stripDelay[NUM_PIXELS_CORE];
int stripDirection[NUM_PIXELS_CORE];
int stripRedActive, stripGreenActive, stripBlueActive;



void setup() {

  // setup inputs & outputs

  pinMode(T_AUTO_PIN, INPUT_PULLUP);
  pinMode(B_FLUSH_PIN, INPUT_PULLUP);
  pinMode(B_FILTER1_PIN, INPUT_PULLUP);
  pinMode(B_FILTER2_PIN, INPUT_PULLUP);
  pinMode(B_FILTER3_PIN, INPUT_PULLUP);
  pinMode(P_INTENSITY_PIN, INPUT);

  Wire.begin();                       // Wire must be started first


  Serial.begin(230400);


  pwmController.resetDevices();       // Software resets all PCA9685 devices on Wire line
  pwmController.init(0x00);        // Address pins A5-A0 set to B010101
  pwmController2.init(0x01);        // Address pins A5-A0 set to B010101
  pwmController.setPWMFrequency(1600); // Default is 200Hz, supports 24Hz to 1526Hz
  pwmController2.setPWMFrequency(1600); // Default is 200Hz, supports 24Hz to 1526Hz

  /*
    pwm.begin();
    pwm.setPWMFreq(1600);  // This is the maximum PWM frequency
     pwm2.begin();
    pwm2.setPWMFreq(1600);  // This is the maximum PWM frequency
  */
  Wire.setClock(800000);

  // NeoPixel
  stripArrow.begin();
  stripArrow.show();
  stripCore.begin();
  stripCore.show();

  // servo
  muxServo.attach(SERVO_PIN_A);

  randomSeed(analogRead(1));

  // start up values and functions
  functionState[F_BOOTUP] = 1;


  // special perminate state for demo and pictures
  button[B_FILTER1] = digitalRead(B_FILTER1_PIN);
  if (button[B_FILTER1] == BUTTON_PRESSED) {
    stripRedActive = 1;
    stripGreenActive = 1;
    stripBlueActive = 1;
    muxServo.write(SERVO_POS_GREEN);
    stripArrowColor = STRIP_ARROW_GREEN;
    indicatorValues[I_W1] = INDICATOR_MAX;
    indicatorValues[I_W3] = INDICATOR_MAX;
    indicatorValues[I_GREEN] = INDICATOR_MAX;
    indicatorValues[I_F2] = INDICATOR_MAX;
    indicatorValues[I_FLUSH] = INDICATOR_MAX;
    indicatorValues[I_BACKLIGHT] = INDICATOR_MAX;
    updateIndicators();
    while (1) {
      updateCounters();
      updateLedStrips();
      updatePWM();
    }
  } // send special state



}




void loop() {


  processButtons();

  //indicatorState[I_BACKLIGHT] =  RAND;

  // long tb = millis();
  updatePWM();
  // Serial.println(millis() - tb);


  /*  */
  static unsigned long millisBuffer;

  if (millis() > millisBuffer + 10) {

    millisBuffer = millis();

    updateCounters();
    processToggles();
    updateFunctions();
    updateIndicators();
    updateLedStrips();
    updateSystemStatus();

  }


  /*
    // Debug output
    Serial.print(functionState[F_BOOTUP]);
    Serial.print(" - ");
    Serial.print(functionState[F_CRASH]);
    Serial.print(" - ");
    Serial.print(functionState[F_FLUSH]);
    Serial.print(" - ");
    Serial.print(systemState);
    Serial.print(" - ");
    Serial.println(systemStatus);
  */

} // end loop()


void processButtons() {

  button[B_FILTER1] = digitalRead(B_FILTER1_PIN);
  button[B_FILTER2] = digitalRead(B_FILTER2_PIN);
  button[B_FILTER3] = digitalRead(B_FILTER3_PIN);
  button[B_FLUSH] = digitalRead(B_FLUSH_PIN);

  // check for button presses and implement debouncing and anti-hold
  for (int i = 0; i < NUM_BUTTONS; i++) {

    if (buttonOld[i] != button[i]) {
      systemStateAutoCounter = 0;
      buttonOld[i] = button[i];
    }


    if (button[i] == BUTTON_PRESSED && buttonReleasedFlag[i] == true) {
      buttonReleasedFlag[i] = false;
      buttonDebounceCounter[i] = 0;

      // perform actions
      /////////////////////////////////////

      // suppress input during system crash
      if (functionState[F_CRASH] == 0) {

        if (i == B_FILTER1) functionState[F_FILTER1] = 1;
        if (i == B_FILTER2) functionState[F_FILTER2] = 1;
        if (i == B_FILTER3) functionState[F_FILTER3] = 1;

        if (i == B_FLUSH && systemStatus != 0) {
          if (functionState[F_FLUSH] == 0) functionState[F_FLUSH] = 1; // do not restart function if button is pressed again
        }
        else if (i != B_FLUSH) {

          // allow three button presses before attempting to increment systemStatus
          systemStatusWaitUntilIncrease++;
          if (systemStatusWaitUntilIncrease > 3)  if (random(0, 3) == 0) systemStatus++;

          //systemStatus++;
        }

      }

      /////////////////////////////////////

    } else {
      // button depressed
      if (button[i] == !BUTTON_PRESSED && buttonDebounceCounter[i] > 5)  buttonReleasedFlag[i] = true;
    }
  }
} // end processButtons()



void updateFunctions() {

  const int stripDimRate = 3;

  // add pot effect
  // max pot = 1023
  int filterDelay =  (1023 - pot) / 3 + 150;
  int functionFlushDelay = 500;
  int functionCrashDelay = 800;
  int functionBootUpDelay = 600;

  // update function counters
  for (int i = 0; i < NUM_FUNCTIONS; i++)
  {
    functionCounter[i]++;
  }

  // update function states based on time elapsed

  // TODO : LOOP THESE

  if (functionState[F_FILTER1] != 0 && functionCounter[F_FILTER1] > (filterDelay / 10)) {
    functionCounter[F_FILTER1] = 0;
    functionState[F_FILTER1]++;
  }
  if (functionState[F_FILTER2] != 0 && functionCounter[F_FILTER2] > (filterDelay / 10)) {
    functionCounter[F_FILTER2] = 0;
    functionState[F_FILTER2]++;
  }
  if (functionState[F_FILTER3] != 0 && functionCounter[F_FILTER3] > (filterDelay / 10)) {
    functionCounter[F_FILTER3] = 0;
    functionState[F_FILTER3]++;
  }
  if (functionState[F_FLUSH] != 0 && functionCounter[F_FLUSH] > (functionFlushDelay / 10)) {
    functionCounter[F_FLUSH] = 0;
    functionState[F_FLUSH]++;
  }
  if (functionState[F_CRASH] != 0 && functionCounter[F_CRASH] > (functionCrashDelay / 10)) {
    functionCounter[F_CRASH] = 0;
    functionState[F_CRASH]++;
  }
  if (functionState[F_BOOTUP] != 0 && functionCounter[F_BOOTUP] > (functionBootUpDelay / 10)) {
    functionCounter[F_BOOTUP] = 0;
    functionState[F_BOOTUP]++;
  }

  // add toggle (AUTO FLUX) effect
  // randomly

  if (indicatorState[I_AMBER] == ON) {

  }



  // process functions

  // functions have priority, supress all other functions base on priorty
  // TODO : convert to a subroutine
  if (functionState[F_BOOTUP] != 0) {
    // supress all other functions
    for (int i = 0; i < NUM_FUNCTIONS; i++) {
      if (i != F_BOOTUP) functionState[i] = 0;
    }
  }
  if (functionState[F_CRASH] != 0) {
    // supress all other functions
    for (int i = 0; i < NUM_FUNCTIONS; i++) {
      if (i != F_CRASH) functionState[i] = 0;
    }
  }
  if (functionState[F_FLUSH] != 0) {
    // supress all other functions
    for (int i = 0; i < NUM_FUNCTIONS; i++) {
      if (i != F_FLUSH) functionState[i] = 0;
    }
  }



  // bootup
  switch (functionState[F_BOOTUP]) {
    case 0:
      functionCounter[F_BOOTUP] = 0;
      break;
    case 1:
      muxServo.write(SERVO_POS_UP);
      stripArrowColor = STRIP_ARROW_OFF;
      stripRedActive = 0;
      stripGreenActive = 0;
      stripBlueActive = 0;
      indicatorState[I_BACKLIGHT] = OFF;
      for (int i = 0; i < NUM_INDICATORS; i++) {
        indicatorState[i] = OFF;
      }
      break;
    case 5:
      indicatorState[I_F1] = ON;
      indicatorState[I_BLUE] = ON;
      indicatorState[I_W1] = ON;
      stripArrowColor = STRIP_ARROW_BLUE;
      break;

    case 6:
      indicatorState[I_F2] = ON;
      indicatorState[I_GREEN] = ON;
      indicatorState[I_W2] = ON;
      stripArrowColor = STRIP_ARROW_GREEN;
      break;

    case 7:
      indicatorState[I_F3] = ON;
      indicatorState[I_RED] = ON;
      indicatorState[I_W3] = ON;
      stripArrowColor = STRIP_ARROW_RED;
      break;

    case 9:
      for (int i = 0; i < NUM_INDICATORS; i++) {
        indicatorState[i] = OFF;
      }
      indicatorState[I_BACKLIGHT] = ON;
      stripArrowColor = STRIP_ARROW_OFF;
      functionState[F_BOOTUP] = 0;
      coreFillState = 0;
      systemStatus = 0;
      break;

  }


  // crash
  switch (functionState[F_CRASH]) {
    case 0:
      functionCounter[F_CRASH] = 0;
      break;
    case 1:
      indicatorState[I_W1] = RAND;
      indicatorState[I_W2] = RAND;
      indicatorState[I_W3] = RAND;
      break;
    case 2:
      indicatorState[I_F1] = RAND;
      indicatorState[I_F2] = RAND;
      indicatorState[I_F3] = RAND;
      break;
    case 3:
      indicatorState[I_RED] = RAND;
      indicatorState[I_GREEN] = RAND;
      indicatorState[I_BLUE] = RAND;
      break;
    case 4:
      indicatorState[I_BACKLIGHT] = RAND;
      break;
    case 7:
      functionState[F_CRASH] = 0;
      functionState[F_BOOTUP] = 1;
      break;
  }

  // add actions to crash function to control features
  if (functionState[F_CRASH] > 1) {
    static int functionCrashServoCounter;
    functionCrashServoCounter++;
    if (functionCrashServoCounter > 10) {
      functionCrashServoCounter = 0;
      muxServo.write(random(SERVO_POS_BLUE, SERVO_POS_RED));
      int r = random(0, 4);
      if (r == 0) stripArrowColor = STRIP_ARROW_RED;
      if (r == 1) stripArrowColor = STRIP_ARROW_GREEN;
      if (r == 2) stripArrowColor = STRIP_ARROW_BLUE;
    }
  }



  // flush
  switch (functionState[F_FLUSH]) {
    case 0:
      functionCounter[F_FLUSH] = 0;
      break;
    case 1:
      muxServo.write(SERVO_POS_UP);
      stripArrowColor = STRIP_ARROW_OFF;
      indicatorState[I_FLUSH] = ON;
      indicatorState[I_W1] = OFF;
      indicatorState[I_W2] = OFF;
      indicatorState[I_W3] = OFF;
      break;
    case 2:
      indicatorState[I_F1] = ON;
      break;
    case 3:
      indicatorState[I_F2] = ON;
      break;
    case 4:
      indicatorState[I_F3] = ON;
      break;
    case 5:
      indicatorState[I_RED] = ON;
      indicatorState[I_GREEN] = ON;
      indicatorState[I_BLUE] = ON;
      break;
    case 6:
      indicatorState[I_F1] = OFF;
      indicatorState[I_F2] = OFF;
      indicatorState[I_F3] = OFF;
      indicatorState[I_FLUSH] = OFF;
      indicatorState[I_RED] = OFF;
      indicatorState[I_GREEN] = OFF;
      indicatorState[I_BLUE] = OFF;
      functionState[F_FLUSH] = 0;
      stripRedActive = 0;
      stripGreenActive = 0;
      stripBlueActive = 0;
      systemStatus = 0;
      coreFillState = 0;
      systemStatusWaitUntilIncrease = 0;
      break;

  }


  switch (functionState[F_FILTER1]) {
    case 0:
      functionCounter[F_FILTER1] = 0;
      break;
    case 1:
      indicatorState[I_F1] = ON;
      break;
    case 2:
      muxServo.write(SERVO_POS_RED);
      stripArrowColor = STRIP_ARROW_RED;
      break;
    case 3:
      indicatorState[I_RED] = ON;
      break;
    case 4:
      stripRedActive = 1;
      stripColor[0] -= stripDimRate;
      if (stripColor[0] < 0) stripColor[0] = 0;
      stripColor[1] -= stripDimRate;
      if (stripColor[1] < 0) stripColor[1] = 0;
      stripColor[2] -= stripDimRate;
      if (stripColor[2] < 0) stripColor[2] = 0;
      break;
    case 5:
      indicatorState[I_RED] = OFF;
      indicatorState[I_F1] = OFF;
      functionState[F_FILTER1] = 0;
      break;
  }

  switch (functionState[F_FILTER2]) {
    case 0:
      functionCounter[F_FILTER2] = 0;
      break;
    case 1:
      indicatorState[I_F2] = ON;
      break;
    case 2:
      muxServo.write(SERVO_POS_GREEN);
      stripArrowColor = STRIP_ARROW_GREEN;
      break;
    case 3:
      indicatorState[I_GREEN] = ON;
      break;
    case 4:
      stripGreenActive = 1;
      stripColor[3] -= stripDimRate;
      if (stripColor[3] < 0) stripColor[3] = 0;
      stripColor[4] -= stripDimRate;
      if (stripColor[4] < 0) stripColor[4] = 0;
      stripColor[5] -= stripDimRate;
      if (stripColor[5] < 0) stripColor[5] = 0;
      break;
    case 5:
      indicatorState[I_GREEN] = OFF;
      indicatorState[I_F2] = OFF;
      functionState[F_FILTER2] = 0;
      break;
  }

  switch (functionState[F_FILTER3]) {
    case 0:
      functionCounter[F_FILTER3] = 0;
      break;
    case 1:
      indicatorState[I_F3] = ON;
      break;
    case 2:
      muxServo.write(SERVO_POS_BLUE);
      stripArrowColor = STRIP_ARROW_BLUE;
      break;
    case 3:
      indicatorState[I_BLUE] = ON;
      break;
    case 4:
      stripBlueActive = 1;
      stripColor[6] -= stripDimRate;
      if (stripColor[6] < 0) stripColor[6] = 0;
      stripColor[7] -= stripDimRate;
      if (stripColor[7] < 0) stripColor[7] = 0;
      stripColor[8] -= stripDimRate;
      if (stripColor[8] < 0) stripColor[8] = 0;
      break;
    case 5:
      indicatorState[I_BLUE] = OFF;
      indicatorState[I_F3] = OFF;
      functionState[F_FILTER3] = 0;
      break;
  }

}


void updateIndicators() {

  // see notes about PWM controller value quirks at the top of the file

  int slow = 50;
  int med = 25;
  int fast = 10;
  int turnOnOffSpeed = 500;

  for (int i = 0; i < NUM_INDICATORS; i++) {


    switch (indicatorState[i]) {

      case OFF:
        if (indicatorValues[i] != INDICATOR_OFF)
        {
          indicatorValues[i] = indicatorValues[i] - turnOnOffSpeed;
          if (indicatorValues[i] < 1) indicatorValues[i] = INDICATOR_OFF;
        }
        break;

      case ON:
        if (indicatorValues[i] == INDICATOR_OFF) indicatorValues[i] = 1;
        if (indicatorValues[i] != INDICATOR_MAX) {
          indicatorValues[i] = indicatorValues[i] + turnOnOffSpeed;
          if (indicatorValues[i] > 4094) indicatorValues[i] = INDICATOR_MAX;

        }
        // special case for backlight brightness
        if (i == I_BACKLIGHT && indicatorValues[i] > BRIGHTNESS_BACKLIGHT) indicatorValues[i] = BRIGHTNESS_BACKLIGHT;
        break;

      case SLOW :
      case MED :
      case FAST :

        int bDelay;
        if (indicatorState[i] == SLOW) bDelay = slow;
        if (indicatorState[i] == MED) bDelay = med;
        if (indicatorState[i] == FAST) bDelay = fast;


        switch (indicatorBlinkState[i]) {
          case 0: // turn on
            indicatorValues[i] = indicatorValues[i] + turnOnOffSpeed;
            if (indicatorValues[i] >= 4094) {
              indicatorBlinkState[i] = 1;
              indicatorValues[i] = INDICATOR_MAX;
              indicatorBlinkCounter[i] = 0;
            }
            break;
          case 1: // wait
            if (indicatorBlinkCounter[i] > bDelay) indicatorBlinkState[i] = 2;
            break;
          case 2: // turn off
            indicatorValues[i] = indicatorValues[i] - turnOnOffSpeed;
            if (indicatorValues[i] <= 1) {
              indicatorBlinkState[i] = 3;
              indicatorValues[i] = INDICATOR_OFF;
              indicatorBlinkCounter[i] = 0;
            }
            break;
          case 3: // wait
            if (indicatorBlinkCounter[i] > bDelay) indicatorBlinkState[i] = 0;
            indicatorValues[i] = 1; // prime the value
            break;
        }
        break;

      case RAND:

        if (indicatorBlinkCounter[i] > 5) {
          indicatorBlinkCounter[i] = 0;
          if (random(0, 2) == 0) indicatorValues[i] = random(1, 500);
          else indicatorValues[i] = random(3000, 4095);
        }

        break;

    }
  }
}


void processToggles() {

  int toggleRead[NUM_TOGGLES];

  toggleRead[T_AUTO] = digitalRead(T_AUTO_PIN);


  for (int i = 0; i < NUM_TOGGLES; i++) {

    if (toggleOld[i] != toggleRead[i]) {
      systemStateAutoCounter = 0;
      toggleOld[i] = toggleRead[i];
    }
    if (systemState == MANUAL) toggle[i] = toggleRead[i];
  }

  if (toggle[T_AUTO] == TOGGLE_ON) indicatorState[I_AMBER] = ON;
  else indicatorState[I_AMBER] = OFF;

}



void updateLedStrips() {

  // strip arrow, not optimized
  for (int i = 0; i < stripArrow.numPixels(); i++) {
    if (stripArrowColor == STRIP_ARROW_RED) {
      stripArrow.setPixelColor(i, stripArrow.Color(255, 0, 0));
    } else if (stripArrowColor == STRIP_ARROW_GREEN) {
      stripArrow.setPixelColor(i, stripArrow.Color(0, 255, 0));
    } else if (stripArrowColor == STRIP_ARROW_BLUE) {
      stripArrow.setPixelColor(i, stripArrow.Color(0, 0, 255));
    } else {
      stripArrow.setPixelColor(i, stripArrow.Color(0, 0, 0));
    }
  }

  stripArrow.show();

  // process Chromatic Core strip

  // add pot effect to
  int jumpSpeed = map(pot, 0, 1023, 1, 7);

  static bool doOnceStripCoreFlag = false;
  if (doOnceStripCoreFlag == false) {
    doOnceStripCoreFlag = true;
    for (int i = 0; i < NUM_PIXELS_CORE; i++) {
      stripDelay[i] = random(0, 4);
      stripColor[i] = random(0, 256);
    }
  }

  for (int i = 0; i < NUM_PIXELS_CORE; i++) {

    // function takes priority over strip pattern
    // brightens strip section briefly
    if (functionState[F_FILTER1] != 0 && i < 3) continue;
    if (functionState[F_FILTER2] != 0 && i > 2 && i < 6) continue;
    if (functionState[F_FILTER3] != 0 && i > 5) continue;
    // dim and turn off strip section during flush
    if (functionState[F_FLUSH] != 0) {
      stripDirection[i] = 0;
      jumpSpeed = 5;
    }

    stripCounter[i]++;

    if (stripCounter[i] >= stripDelay[i]) {

      stripCounter[i] = 0;

      // increment or decrement brightness of individual LED
      if (stripDirection[i] == 1) {
        stripColor[i] += jumpSpeed;
        if (stripColor[i] >= 255) {
          stripDirection[i] = 0;
          stripColor[i] = 255;
        }
      } else {
        stripColor[i] -= jumpSpeed;
        if (stripColor[i] <= 0) {
          stripDirection[i] = 1;
          stripColor[i] = 0;
        }
        stripDelay[i] = random(0, 4);
      }

      // change display pattern/colors based on warning indicators
      if (indicatorState[I_W3] != OFF) { // SPECTRUM BREAKDOWN

        if (i < 3) stripCore.setPixelColor(i, stripCore.Color(stripColor[i], ~stripColor[i], 0 ));
        if (i > 2 && i < 6) stripCore.setPixelColor(i, stripCore.Color(0,  stripColor[i], ~stripColor[i] ));
        if  (i > 5) stripCore.setPixelColor(i, stripCore.Color(~stripColor[i], 0, stripColor[i]));

        // remove middle pixels, reduces color wash allowing greater color diversity
        stripCore.setPixelColor(1, stripCore.Color(0, 0, 0));
        stripCore.setPixelColor(4, stripCore.Color(0, 0, 0));
        stripCore.setPixelColor(7, stripCore.Color(0, 0, 0));

      } else if (indicatorState[I_W2] != OFF) { // QUANTUM DELINEATION

        if (i < 3) stripCore.setPixelColor(i, stripCore.Color(stripColor[i] , stripColor[i]  , 0 ));
        if (i > 2 && i < 6) stripCore.setPixelColor(i, stripCore.Color(0, stripColor[i], stripColor[i] ));
        if  (i > 5) stripCore.setPixelColor(i, stripCore.Color(stripColor[i], 0, stripColor[i]));

      } else if (indicatorState[I_W1] != OFF) { // DORMISON'S PARADOX

        if (i < 3) stripCore.setPixelColor(i, stripCore.Color(0, stripColor[i], 0 ));
        if (i > 2 && i < 6) stripCore.setPixelColor(i, stripCore.Color(0, 0, stripColor[i]));
        if  (i > 5) stripCore.setPixelColor(i, stripCore.Color(stripColor[i], 0, 0));

      } else { // normal operation

        if (i < 3) stripCore.setPixelColor(i, stripCore.Color(stripColor[i], 0, 0));
        if (i > 2 && i < 6) stripCore.setPixelColor(i, stripCore.Color(0, stripColor[i], 0));
        if  (i > 5) stripCore.setPixelColor(i, stripCore.Color(0, 0, stripColor[i]));

      }

      // add function crash consideration
      // add random blink to indivdual LEDs
      if (functionState[F_CRASH] > 1) {
        if (random(0, 10) == 0) {
          stripCore.setPixelColor(i, stripCore.Color(random(0, 255), random(0, 255), random(0, 255)));
        }
      }

      /*
            // add function flush consideration
            // dim all LEDs
            if (functionState[F_FLUSH] > 1) {
              int dimValue = 65;
              for (int i = 0; i < 3; i++) {
              stripCore.setPixelColor(i, dimValue, 0, 0);
              stripCore.setPixelColor(3 + i, 0, dimValue, 0);
              stripCore.setPixelColor(6 + i, 0, 0, dimValue);
              }
            }
      */

      // check if strip section is active
      if (i < 3 && stripRedActive == 0) stripCore.setPixelColor(i, stripCore.Color(0, 0, 0));
      if (i > 2 && i < 6 && stripGreenActive == 0) stripCore.setPixelColor(i, stripCore.Color(0, 0, 0));
      if  (i > 5 && stripBlueActive == 0) stripCore.setPixelColor(i, stripCore.Color(0, 0, 0));
    }

  } // end for

  stripCore.show();

}



void updatePWM() {

  // front panel PWM controller

  pwms[0][2] = indicatorValues[I_BACKLIGHT];
  pwms[0][4] = indicatorValues[I_RED];
  pwms[0][3] = indicatorValues[I_GREEN];
  pwms[0][5] = indicatorValues[I_BLUE];
  pwms[0][6] = indicatorValues[I_AMBER];
  pwms[0][8] = indicatorValues[I_FLUSH];
  pwms[0][9] = indicatorValues[I_F1];
  pwms[0][10] = indicatorValues[I_F3];
  pwms[0][11] = indicatorValues[I_F2];

  // if indicator is at 4094 (max brightness) turn indicator fully on by using the zero value
  for (int i = 0; i < 16; i++)
  {
    if ( pwms[0][i] == 4094) pwms[0][i] = 0;
  }


  bool updateFlag = false;

  // check if pwms value has changed
  for (int i = 0; i < 16; i++)
  {
    if (pwms[0][i] != pwmsOldValue[0][i]) {
      pwmsOldValue[0][i] = pwms[0][i];
      updateFlag = true;
    }
  }

  if (updateFlag == true) pwmController2.setChannelsPWM(0, 16, pwms[0]);


  // back panel PWM controller

  // convert potentiometer input into bargraph display
  // with dimmed bars to enhance visual resolution

  int potRead, hPot, pPot;
  static signed int potBuffer;

  potRead = analogRead(P_INTENSITY_PIN);

  if (potRead > potOld + 10 || potRead < potOld - 10) {
    potOld = potRead;
    systemStateAutoCounter =  0;
  }

  // during crash or AUTO, randomize pot
  if (functionState[F_CRASH] > 1) {
    pot = random(1, 1023);

  } else if (systemState == AUTO) {
    static int subCount = 0;
    if (subCount++ > 3) {
      subCount = 0;
      if (pot < potTarget) pot++;
      if (pot > potTarget) pot--;
      if (pot <= 0) pot = 0;
      if (pot > 1023) pot = 1023;
    }
    hPot = map(pot, 0, 1023, 0, 10);
    pPot = map(pot, 0, 1023, 0, 1000) % 100;
    pPot = map(pPot, 0, 100, 100, 4095);

  }  else if (functionState[F_BOOTUP] > 1 || functionState[F_FLUSH] > 1 )  {
    hPot = 0;
    pPot = INDICATOR_OFF;
    pot = 1;
    potBuffer = 2;

  } else  {
    // normal operation
    pot = potRead;

    // prevent ADC jitter from tiggering PWM refresh
    if (pot > potBuffer + 3 || pot < potBuffer - 3) {
      potBuffer = pot;
    } else {
      pot = potBuffer;
    }

    hPot = map(pot, 0, 1023, 0, 10);
    pPot = map(pot, 0, 1023, 0, 1000) % 100;
    pPot = map(pPot, 0, 100, 100, 4095);
  }


  for (int i = 0; i < 11; i++) {
    if (i <= hPot)
      pwms[1][i + 1] = 0;
    else
      pwms[1][i + 1] = 4095;
  }


  pwms[1][hPot + 1] = pPot;

  pwms[1][12] = indicatorValues[I_W2];
  pwms[1][13] = indicatorValues[I_BACKLIGHT];
  pwms[1][14] = indicatorValues[I_W1];
  pwms[1][15] = indicatorValues[I_W3];

  // if indicator is at 4094 (max brightness) turn indicator fully on by using the zero value
  for (int i = 0; i < 16; i++)
  {
    if ( pwms[1][i] == 4094) pwms[1][i] = 0;
  }

  updateFlag = false;

  // check if pwms value has changed
  for (int i = 0; i < 16; i++)
  {
    if (pwms[1][i] != pwmsOldValue[1][i]) {
      pwmsOldValue[1][i] = pwms[1][i];
      updateFlag = true;
    }
  }

  if (updateFlag == true) pwmController.setChannelsPWM(0, 16, pwms[1]);

}


void updateCounters() {

  // update button debounce counter
  for (int i = 0; i < NUM_BUTTONS; i++) {
    buttonDebounceCounter[i]++;
  }

  for (int i = 0; i < NUM_INDICATORS; i++) {
    indicatorBlinkCounter[i]++;
  }

  systemStateAutoCounter++;
  if (systemStateAutoCounter > 30000) systemStateAutoCounter = 30000;

  Serial.println(systemStateAutoCounter);

}


void updateSystemStatus() {

  // user input resets counter
  if (systemStateAutoCounter > (TIME_UNTIL_SYSTEM_STATE_AUTO / TIME_BETWEEN_PROCESSESS)) systemState = AUTO;
  if (systemStateAutoCounter < 5) systemState = MANUAL;

  switch (systemState) {

    case MANUAL:

      break;

    case AUTO:

      systemStatusWaitUntilIncrease = 0;

      // AUTO FLUX
      if (functionState[F_CRASH] == 0 && functionState[F_BOOTUP] == 0) {
        static int delayBetweenFluxCounter;
        static int delayBetwenFluxDelay;

        delayBetweenFluxCounter++;
        if (delayBetweenFluxCounter > delayBetwenFluxDelay) {
          delayBetweenFluxCounter = 0;
          delayBetwenFluxDelay = random(100, 300);

          // TODO: maybe toggle the indicator
          indicatorState[I_AMBER] = ON;
        }

        // HYPERSPIN INTENSITY
        static int delayBetweenIntensityCounter;
        static int delayBetwenIntensityDelay;

        delayBetweenIntensityCounter++;
        if (delayBetweenIntensityCounter > delayBetwenIntensityDelay) {
          delayBetweenIntensityCounter = 0;
          delayBetwenIntensityDelay = random(50, 100);
          potTarget = random(0, 1023);
        }

        // OMMICRON FILTER functions
        static int delayBetweenFunctionsCounter;
        static int delayBetwenFunctionsDelay;



        delayBetweenFunctionsCounter++;

        if (delayBetweenFunctionsCounter > delayBetwenFunctionsDelay) {
          delayBetweenFunctionsCounter = 0;

          delayBetwenFunctionsDelay = random(150, 300); // 200 400


          //  fill the core in series after bootup or flush
          coreFillState++;
          if (coreFillState < 4) {

            if (coreFillState == 1) functionState[F_FILTER1] = 1;
            if (coreFillState == 2) functionState[F_FILTER2] = 1;
            if (coreFillState == 3) functionState[F_FILTER3] = 1;


          } else {


            if (coreFillState == 5) systemStatus++; // TEMP
            if (coreFillState == 6) systemStatus++; // TEMP
            if (coreFillState == 7) systemStatus++; // TEMP
            if (coreFillState == 8) systemStatus++; // TEMP

            static int r = 2;
            static int lastR = 2;
            while (r == lastR) {
              r = random(0, 3);
            }
            lastR = r;

            if (r == 0) functionState[F_FILTER1] = 1;
            if (r == 1) functionState[F_FILTER2] = 1;
            if (r == 2) functionState[F_FILTER3] = 1;

            // systemStatus
            //if (random(0, 100) > 60) {
            if (random(0, 100) > 40) { // TEMP
              //systemStatus++;
            } else {
              //if (systemStatus != 0) systemStatus--;
            }

          }

        }
      }

      break;
  }

  // systemStatus is health/defcon/stability of the system
  // the higher the systemStatus value, the more unstable the system is
  // once the systemStatus becomes great a system crash will occur

  static bool doOnceFlag1 = false;
  static bool doOnceFlag2 = false;
  static bool doOnceFlag3 = false;
  static bool doOnceFlag4 = false;

  // allow systemStatus to increase and decrease in AUTO
  if (systemStatus != systemStatusBuffer) {
    systemStatusBuffer = systemStatus;
    doOnceFlag1 = false;
    doOnceFlag2 = false;
    doOnceFlag3 = false;
    doOnceFlag4 = false;
  }

  static int memWarningIndicatorSelected;

  switch (systemStatus) {
    case 0:
      memWarningIndicatorSelected = random(0, 3);
      indicatorState[I_FLUSH] = OFF;
      break;
    case 1:
      if (doOnceFlag1 == false) {
        doOnceFlag1 = true;
        indicatorState[I_FLUSH] = SLOW;
        indicatorState[I_W1] = OFF;
        indicatorState[I_W2] = OFF;
        indicatorState[I_W3] = OFF;
      }
      break;
    case 2:
      if (doOnceFlag2 == false) {
        doOnceFlag2 = true;
        indicatorState[I_FLUSH] = MED;
        indicatorState[I_W1] = OFF;
        indicatorState[I_W2] = OFF;
        indicatorState[I_W3] = OFF;
        if (memWarningIndicatorSelected == 0) indicatorState[I_W1] = MED;
        if (memWarningIndicatorSelected == 1) indicatorState[I_W2] = MED;
        if (memWarningIndicatorSelected == 2) indicatorState[I_W3] = MED;
      }
      break;
    case 3:
      if (doOnceFlag3 == false) {
        doOnceFlag3 = true;
        indicatorState[I_FLUSH] = FAST;
        if (indicatorState[I_W1] != OFF) {
          if (random(0, 2) == 0) indicatorState[I_W2] = FAST;
          else indicatorState[I_W3] = FAST;
        } else if (indicatorState[I_W2] != OFF) {
          if (random(0, 2) == 0) indicatorState[I_W1] = FAST;
          else indicatorState[I_W3] = FAST;
        } else if (indicatorState[I_W3] != OFF) {
          if (random(0, 2) == 0) indicatorState[I_W1] = FAST;
          else indicatorState[I_W2] = FAST;
        }
      }
      break;
    case 4:
      if (doOnceFlag4 == false) {
        doOnceFlag4 = true;
        // system crash
        functionState[F_CRASH] = 1;
      }
      break;
  } // end switch (systemStatus)

}

