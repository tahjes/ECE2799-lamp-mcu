#include <Trill.h>

Trill trillSensor;

// #define PIN_FAULT 2
// #define DIR_A 4
// #define DIR_B 5
#define PWM_OUT 26 

boolean touchActive = false;
float lastTouchPos = 0; 
float sliderValue = 0;

// Double tap detection
uint8_t tapCount = 0;
unsigned long lastTapTime = 0;
const unsigned long doubleTapThreshold = 400; // in milliseconds

// constantly drive output until value has been updated
int lastPWM = 0;

// define each state
typedef enum {
    STATE_IDLE,
    STATE_CONN,
    STATE_POL, 
    STATE_ERR,
    STATE_ON,
    STATE_DISCONNECT
} State;

State currentState = STATE_IDLE;

// // Function prototypes
// void fault_callback();
// void reverse_polarity();

// Map function to scale for PWM output
uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Globals
// volatile bool fault_status = 0; // used in ISR - volatile

// Sensor initialization
void setup() {

  Wire.begin();
  Wire.setClock(50000);  // 50 kHz I2C clock

  Serial.begin(115200);
  int ret = trillSensor.setup(Trill::TRILL_BAR);
  if(ret != 0) {
    Serial.println("Failed to initialize Trill sensor");
    Serial.print("Error code: ");
    Serial.println(ret);
  }
  else {
    Serial.println("Trill sensor initialized successfully");
  }

  // setup pins
  // pinMode(PIN_FAULT, INPUT_PULLUP); // fault pin for PSU
  // pinMode(DIR_A, OUTPUT);
  // pinMode(DIR_B, OUTPUT);
  pinMode(PWM_OUT, OUTPUT); // PWM output
  analogWrite(PWM_OUT, 0);

  // attachInterrupt(digitalPinToInterrupt(PIN_FAULT), fault_callback, FALLING);

  // // anode goes to positive
  // digitalWrite(DIR_A, LOW);
  // digitalWrite(DIR_B, LOW);

  // i2c setup
  Wire.begin();
  Wire.setClock(50000);
}

void loop() {
  delay(50); // 20 Hz read rate
  trillSensor.read();

  float currentTouchPos = -1; // value set when sensor isn't touched
  float normalizedPos = 0; // currentTouchPos scaled to read live in serial output

  // One finger is detected and used for slider
  if(trillSensor.getNumTouches() > 0) {
    currentTouchPos = trillSensor.touchLocation(0);
    normalizedPos = currentTouchPos / 320.0;

    if(!touchActive){
      lastTouchPos = normalizedPos;
      touchActive = true;

      Serial.print("Touch started at: ");
      Serial.println(normalizedPos, 4);

      handleTap(); // when a new touch starts
    }
    else {
      float diff = normalizedPos - lastTouchPos;
      if(abs(diff) > 0.005) {
        sliderValue += diff * 3200;
        sliderValue = constrain(sliderValue, 0, 3200);
        lastTouchPos = normalizedPos;

        Serial.print("Sliding - Pos: ");
        Serial.println((int)sliderValue);
      }
    }
  }
  else if(touchActive) {

    touchActive = false;
    Serial.print("Touch Ended. Slider Value: ");
    Serial.println((int)sliderValue);

  }

  // state machine
  switch (currentState) {
    case STATE_IDLE: 
      if (tapCount == 1){
        currentState = STATE_CONN;
        Serial.println("-> Transition: IDLE to CONN");
      }
      break;
    case STATE_CONN:
      currentState = STATE_ON;
      
      // if (fault_status){
      //   currentState = STATE_POL;
      //   Serial.println("-> Transition: CONN to POL");
      // }
      // else {
      //   currentState = STATE_ON;
      //   Serial.println("-> Transition: CONN to ON");
      // }
      break;
    // case STATE_POL:
    //   reverse_polarity();
    //   if (!fault_status){
    //     currentState = STATE_ON;
    //     Serial.println("-> Transition: POL to ON");
    //   }
    //   else {
    //     currentState = STATE_ERR;
    //     Serial.println("-> Transition: POL to ERR");
    //   }
    //   break;
    case STATE_ON:
    // map sensor/slider value to PWM brightness
      int pwmVal;
      if (currentTouchPos >= 0){
        pwmVal = (currentTouchPos / 320.0) * 255;
        pwmVal = constrain(pwmVal, 0, 255);
        lastPWM = pwmVal;
      }
      else {
        pwmVal = lastPWM;
      }
      analogWrite(PWM_OUT, pwmVal);
      if (tapCount > 1){
        currentState = STATE_DISCONNECT;
        tapCount = 0;
      }
      break;
    case STATE_DISCONNECT:
      Serial.println("State: DISCONNECT");
      analogWrite(PWM_OUT, 0); // LED shouldn't be outputting anything
      currentState = STATE_IDLE;
      break;
    // case STATE_ERR:
    //   Serial.println("State: ERROR");
    //   currentState = STATE_IDLE;
    //   break;
  }
}

// void fault_callback() {
//   bool fault = !digitalRead(PIN_FAULT);
//   if (fault) {
//     fault_status = 1;
//   }
//   else {
//     fault_status = 0;
//   }
// }

// void reverse_polarity() {
//   if (digitalRead(DIR_A) == LOW && digitalRead(DIR_B) == HIGH) {
//     digitalWrite(DIR_A, HIGH);
//     digitalWrite(DIR_B, LOW);
//   }
//   else if (digitalRead(DIR_A) == HIGH && digitalRead(DIR_B) == LOW) {
//     digitalWrite(DIR_A, LOW);
//     digitalWrite(DIR_B, HIGH);
//   }
// }

// System switches between on and off based on double tap
void handleTap(){
  unsigned long now = millis();
  if(now - lastTapTime < doubleTapThreshold) {
    tapCount++;
    if(tapCount == 2) {
      Serial.println("DOUBLE TAP Detected!");
      // tapCount = 0; // reset after double tap
    }
  }
  else {
    tapCount = 1;
  }
  lastTapTime = now;
}