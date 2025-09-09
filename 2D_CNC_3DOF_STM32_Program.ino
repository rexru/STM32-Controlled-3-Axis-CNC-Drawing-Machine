#include "Stepper.h"

//
// ====================== CONFIGURATION CONSTANTS ======================
//

// Rotary encoder pins (adjust for your STM32 board)
const int PinCLK = PA0;   // Rotary encoder CLK (interrupt source)
const int PinDT  = PA3;   // Rotary encoder DT (direction input)
const int PinSW  = PA4;   // Rotary encoder push button

// Microstepping mode control pins (for driver such as A4988)
const int MS1 = PA1;
const int MS2 = PB1;

// Stepper motor 1 (driven with Arduino Stepper library)
const int motorPin1 = PA5;
const int motorPin2 = PA6;
const int motorPin3 = PA7;
const int motorPin4 = PC4;

// Stepper motor 2 (manual stepping, controlled with IN1–IN4 pins)
const int IN1 = PF13;
const int IN2 = PF14;
const int IN3 = PF15;
const int IN4 = PG0;

// LED pins (for status/debug)
#define LED_PIN  PG13   // Onboard green LED
#define LED_PIN2 PG14   // Onboard red LED (currently unused)

// User button pins
#define BUTTON_PIN PB0   // Toggles LED + direction
const int buttonPin = PE10; // Controls motor 2 start/stop

// Stepper motor parameters
#define STEPS 32   // Steps per revolution (internal shaft)
                   // 2048 steps for external shaft

// Speed settings
const int NORMAL_SPEED = 700;  // Default operation speed
const int HOMING_SPEED = 300;  // Slower speed for homing/reset

//
// ====================== GLOBAL VARIABLES ======================
//

// Rotary encoder state
volatile boolean TurnDetected = false;  // Flag set by interrupt
boolean currentDirection = false;       // false = CW, true = CCW
double RotaryPosition = 0;              // Track virtual rotary position
double TotalStepsMoved = 0;             // Track motor steps from origin

// Reset button state
boolean buttonPressed = false;

// Button debounce handling
int buttonState = 0, lastButtonState = 0;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50; // ms

// Second button debounce handling (motor 2)
int buttonState2 = 0, lastButtonState2 = 0;
unsigned long lastDebounceTime2 = 0;
unsigned long debounceDelay2 = 50; // ms

// Motor 2 control
bool motorRotating = false;
int stepNumber = 0;
int direction = 1; // +1 CW, -1 CCW

// Stepper motor object (Arduino Stepper lib)
Stepper small_stepper(STEPS, motorPin1, motorPin3, motorPin2, motorPin4);

//
// ====================== INTERRUPTS ======================
//

// Rotary encoder interrupt (triggered on CLK falling edge)
void isr() {
  delay(4);         // Debounce
  TurnDetected = true;
}

//
// ====================== HELPER FUNCTIONS ======================
//

// Set microstepping mode for driver (A4988/DRV8825 style)
void setMicrostepMode(bool fullStep) {
  if (fullStep) {
    digitalWrite(MS1, LOW);
    digitalWrite(MS2, LOW);
  } else {
    // Example: quarter step mode (can add half/1/8 modes if needed)
    digitalWrite(MS1, HIGH);
    digitalWrite(MS2, LOW);
  }
}

// Manual stepper driver (motor 2, using IN1–IN4)
void stepMotor(int dir) {
  if (dir == 1) {
    stepNumber = (stepNumber + 1) % 8; // Increment, wrap at 7
  } else {
    stepNumber = (stepNumber - 1 + 8) % 8; // Decrement, wrap at 0
  }

  // Drive motor coils in sequence (8-step full sequence)
  switch (stepNumber) {
    case 0: digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);  digitalWrite(IN3,LOW);  digitalWrite(IN4,LOW);  break;
    case 1: digitalWrite(IN1,HIGH); digitalWrite(IN2,HIGH); digitalWrite(IN3,LOW);  digitalWrite(IN4,LOW);  break;
    case 2: digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH); digitalWrite(IN3,LOW);  digitalWrite(IN4,LOW);  break;
    case 3: digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH); digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);  break;
    case 4: digitalWrite(IN1,LOW);  digitalWrite(IN2,LOW);  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);  break;
    case 5: digitalWrite(IN1,LOW);  digitalWrite(IN2,LOW);  digitalWrite(IN3,HIGH); digitalWrite(IN4,HIGH); break;
    case 6: digitalWrite(IN1,LOW);  digitalWrite(IN2,LOW);  digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH); break;
    case 7: digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);  digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH); break;
  }
}

//
// ====================== SETUP ======================
//

void setup() {
  // Rotary encoder setup
  pinMode(PinCLK, INPUT);
  pinMode(PinDT, INPUT);  
  pinMode(PinSW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PinCLK), isr, FALLING);

  // Microstep driver pins
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  setMicrostepMode(true); // Default: full step mode

  // LEDs
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Buttons
  pinMode(BUTTON_PIN, INPUT_PULLUP); 
  pinMode(buttonPin, INPUT_PULLUP);

  // Motor 2 pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Serial debug
  Serial.begin(115200);
  Serial.println("System initialized.");
}

//
// ====================== MAIN LOOP ======================
//

void loop() {
  //
  // --- Handle Motor 2 (manual step sequence, toggle with buttonPin) ---
  //
  int reading2 = digitalRead(buttonPin);
  if (reading2 != lastButtonState2) lastDebounceTime2 = millis();

  if ((millis() - lastDebounceTime2) > debounceDelay2) {
    if (reading2 != buttonState2) {
      buttonState2 = reading2;

      if (buttonState2 == LOW) {  // Button pressed
        motorRotating = !motorRotating; // Toggle run/stop
        if (motorRotating) direction = -direction; // Reverse direction each time
      }
    }
  }

  if (motorRotating) {
    stepMotor(direction);
    delay(5); // Adjust for motor speed
  }

  lastButtonState2 = reading2;

  //
  // --- Handle BUTTON_PIN (toggles LED + motor direction) ---
  //
  int reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonState) lastDebounceTime = millis();

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == LOW) {  // Button pressed
        digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Toggle LED
        currentDirection = !currentDirection;         // Flip direction
      }
    }
  }
  lastButtonState = reading;

  //
  // --- Handle Homing Reset (PinSW button) ---
  //
  small_stepper.setSpeed(NORMAL_SPEED);

  if (!digitalRead(PinSW) && !buttonPressed) { // Button pressed
    delay(50); // Debounce
    if (!digitalRead(PinSW)) {
      buttonPressed = true;

      if (TotalStepsMoved != 0) {
        setMicrostepMode(false);                 // Quarter step for precision
        small_stepper.setSpeed(HOMING_SPEED);    // Slower speed for homing
        small_stepper.step(-TotalStepsMoved);    // Move back to origin
        setMicrostepMode(true);                  // Restore full step
        TotalStepsMoved = 0;                     // Reset tracker
      }
    }
  }
  if (digitalRead(PinSW) && buttonPressed) { // Button released
    delay(50);
    if (digitalRead(PinSW)) buttonPressed = false;
  }

  //
  // --- Handle Rotary Encoder Rotation ---
  //
  if (TurnDetected) {
    int StepsToTake = currentDirection ? -50 : 50; // Direction
    RotaryPosition += currentDirection ? -10 : 10; // Virtual position

    setMicrostepMode(true);                        // Normal full step
    small_stepper.setSpeed(NORMAL_SPEED);
    small_stepper.step(StepsToTake);

    TotalStepsMoved += StepsToTake;
    TurnDetected = false;
  }

  //
  // --- Power Save: Turn off coils when idle ---
  //
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, LOW);
}

// 3rd axis

  // // Read the state of the button
  // int reading2 = digitalRead(BUTTON_PIN2);

  // // Check if the button state has changed
  // if (reading2 != lastButtonState2) {
  //   // Reset the debounce timer
  //   lastDebounceTime2 = millis();
  // }

  // // Debounce the button
  // if ((millis() - lastDebounceTime2) > debounceDelay2) {
  //   // If the button state has changed after debouncing
  //   if (reading2 != buttonState2) {
  //     buttonState2 = reading2;

  //     // If the button is pressed (LOW because of pull-up)
  //     if (buttonState2 == LOW) {
  //       // Toggle the LED
  //       digitalWrite(LED_PIN2, !digitalRead(LED_PIN2));
  //     }
  //   }
  // }

  // // Update the last button state
  // lastButtonState2 = reading2;