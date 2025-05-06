#include <Trill.h>
#include <PD_UFP.h>

extern "C" {
  #include "hardware/pwm.h"
}

Trill trillSensor;
PD_UFP_c PD_UFP;

#define DIR_A 0
#define DIR_B 1
#define PWM_OUT 2
#define FUSB302_INT_PIN 3
#define TEMP_PIN A1
#define CURRENT_PIN A0

#define SERIESRESISTOR 100000 // resistor in series with the thermistor
#define THERMISTORNOMINAL 105000 // resistance at nominal temperature
#define TEMPERATURENOMINAL 25 // temp. for nominal resistance
#define BCOEFFICIENT 4190 // B coefficient of the thermistor
#define SOFT_TEMP_LIMIT 50 // soft limit for temperature
#define HARD_TEMP_LIMIT 60 // hard limit for temperature

boolean touchActive = false;
float lastTouchPos = 0; 
float sliderValue = 0;
int read_current;

// Double tap detection
uint8_t tapCount = 0;
unsigned long lastTapTime = 0;
const unsigned long doubleTapThreshold = 400; // in milliseconds

// PWM settings
int maxPWM = 13901; // maximum PWM value
int minPWM = 5100; // minimum PWM value

// define each state
typedef enum {
    STATE_IDLE,
    STATE_CONN,
    STATE_POL, 
    STATE_OVTEMP,
    STATE_ON,
    STATE_DISCONNECT
} State;

// global initial state
State currentState = STATE_IDLE;
int pwmVal = 0;

// // Function prototypes
void reverse_polarity();

// Map function to scale for PWM output
uint16_t customMap(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Sensor initialization
void setup() {

  Wire.begin();

  // Serial.begin(115200);

  // Initialize the Trill sensor
  int ret = trillSensor.setup(Trill::TRILL_BAR);
  if(ret != 0) {
    // Serial.println("Failed to initialize Trill sensor");
    // Serial.print("Error code: ");
    // Serial.println(ret);
  }
  else {
    // Serial.println("Trill sensor initialized successfully");
  }

  // Initialize the PD_UFP for 20V power delivery
  PD_UFP.init(FUSB302_INT_PIN, PD_POWER_OPTION_MAX_20V);

  // Setup pins
  pinMode(PWM_OUT, OUTPUT);
  pinMode(DIR_A, OUTPUT);
  pinMode(DIR_B, OUTPUT);
  pinMode(CURRENT_PIN, INPUT);
  pinMode(TEMP_PIN, INPUT);

  // Configure PWM_OUT for 16-bit PWM generation on RP2040 with no prescaler (~1.9 kHz frequency)
  pwm_set_wrap(pwm_gpio_to_slice_num(PWM_OUT), 65535); // Set PWM resolution to 16 bits
  pwm_set_chan_level(pwm_gpio_to_slice_num(PWM_OUT), pwm_gpio_to_channel(PWM_OUT), minPWM); // Initialize with minimum duty cycle for current regulation
  pwm_set_clkdiv(pwm_gpio_to_slice_num(PWM_OUT), 1.0f); // Set clock divider to 1 (no prescaler)
  gpio_set_function(PWM_OUT, GPIO_FUNC_PWM); // Set the GPIO function to PWM

  // anode goes to positive
  digitalWrite(DIR_A, HIGH);
  digitalWrite(DIR_B, LOW);

}

void loop() {
  // delay(50); // 20 Hz read rate
  trillSensor.read();

  PD_UFP.run();
  // if (PD_UFP.is_power_ready())
  // {
  //   Serial.write("PD supply connected\n");
  // }
  // else
  // {
  //   Serial.write("No PD supply available\n");
  // }

  float currentTouchPos = -1; // value set when sensor isn't touched
  float normalizedPos = 0; // currentTouchPos scaled to read live in serial output

  // One finger is detected and used for slider
  if(trillSensor.getNumTouches() > 0) {
    currentTouchPos = trillSensor.touchLocation(0);
    normalizedPos = currentTouchPos / 3200.0;

    if(!touchActive){
      lastTouchPos = normalizedPos;
      touchActive = true;

      // Serial.print("Touch started at: ");
      // Serial.println(normalizedPos, 4);

      handleTap(); // when a new touch starts
    }
    else {
      // Serial.print("Sliding - Pos: ");
      // Serial.println((int)currentTouchPos);
      
    }
  }
  else if(touchActive) {

    touchActive = false;
    // Serial.print("Touch Ended. Slider Value: ");
    // Serial.println((int)sliderValue);

  }

  float temperature;
  temperature = analogRead(TEMP_PIN); // read the thermistor value
  temperature = (SERIESRESISTOR / ((1023 / temperature) - 1)) / THERMISTORNOMINAL; // R/Ro
  temperature= log(temperature);                  // ln(R/Ro)
  temperature /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  temperature += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  temperature = 1.0 / temperature;                 // Invert
  temperature -= 273.15;                         // convert absolute temp to C
  // Serial.print("Temperature "); 
  // Serial.print(temperature);
  // Serial.println(" *C");

  // state machine
  switch (currentState) {
    case STATE_IDLE: 
      if (tapCount == 1){
        currentState = STATE_CONN;
        // Serial.println("-> Transition: IDLE to CONN");
        tapCount = 0;
      }
      break;
    case STATE_CONN:
      pwm_set_enabled(pwm_gpio_to_slice_num(PWM_OUT), true); // Enable PWM
      pwm_set_chan_level(pwm_gpio_to_slice_num(PWM_OUT), pwm_gpio_to_channel(PWM_OUT), 8000);
      delay(100); // wait for PWM to stabilize
      // check if current is flowing
      read_current = analogRead(CURRENT_PIN);
      if (read_current < (0.3 / 3.3) * 1023) { // 0.3V out of 3.3V, scaled to ADC range
        // Serial.println("Polarity check failed");
        // reverse polarity
        reverse_polarity();
      }
      else {
        currentState = STATE_ON;
        // Serial.println("-> Transition: CONN to ON");
      }
      break;
    case STATE_ON:
    // update pwm duty when touch is detected
      if (currentTouchPos >= 0){
        // map the touch position to a PWM value
        pwmVal = customMap(int(currentTouchPos), 0, 3200, minPWM, maxPWM);
        pwm_set_chan_level(pwm_gpio_to_slice_num(PWM_OUT), pwm_gpio_to_channel(PWM_OUT), pwmVal);
      }

      if (tapCount > 1){
        currentState = STATE_DISCONNECT;
        tapCount = 0;
      }

      else if (temperature > SOFT_TEMP_LIMIT) {
        currentState = STATE_OVTEMP;
        int lowerPWM = customMap(500, 0, 3200, minPWM, maxPWM);
        // Flash the LED to indicate over temperature
        for (int i = 0; i < 2; i++) {
          pwm_set_chan_level(pwm_gpio_to_slice_num(PWM_OUT), pwm_gpio_to_channel(PWM_OUT), lowerPWM);
          delay(500); // Delay for visibility
          pwm_set_chan_level(pwm_gpio_to_slice_num(PWM_OUT), pwm_gpio_to_channel(PWM_OUT), pwmVal);
          delay(500); // Delay for visibility
        }

        pwm_set_chan_level(pwm_gpio_to_slice_num(PWM_OUT), pwm_gpio_to_channel(PWM_OUT), lowerPWM);
          // Serial.println("-> Transition: ON to OVTEMP");
      }
      break;
    case STATE_DISCONNECT:
      // Serial.println("State: DISCONNECT");
      pwm_set_chan_level(pwm_gpio_to_slice_num(PWM_OUT), pwm_gpio_to_channel(PWM_OUT), 0); // Disableo utput
      // // consider disconnecting the LED entirely
      // digitalWrite(DIR_A, LOW);
      // digitalWrite(DIR_B, LOW);
      currentState = STATE_IDLE;

      break;
    case STATE_OVTEMP:
      // Serial.println("State: Over Temperature");
      if (temperature < SOFT_TEMP_LIMIT) {
        currentState = STATE_ON;
        // Serial.println("-> Transition: OVTEMP to ON");
      }
      else if (tapCount > 1 || temperature > HARD_TEMP_LIMIT) {
        // Serial.println("-> Transition: OVTEMP to DISCONNECT");
        currentState = STATE_DISCONNECT;
        tapCount = 0;
      }
      break;
  }
}

void reverse_polarity() {
  if (digitalRead(DIR_A) == LOW && digitalRead(DIR_B) == HIGH) {
    digitalWrite(DIR_A, HIGH);
    digitalWrite(DIR_B, LOW);
  }
  else if (digitalRead(DIR_A) == HIGH && digitalRead(DIR_B) == LOW) {
    digitalWrite(DIR_A, LOW);
    digitalWrite(DIR_B, HIGH);
  }

}

// System switches between on and off based on double tap
void handleTap(){
  unsigned long now = millis();
  if(now - lastTapTime < doubleTapThreshold) {
    tapCount++;
    if(tapCount == 2) {
      // Serial.println("DOUBLE TAP Detected!");
      // tapCount = 0; // reset after double tap
    }
  }
  else {
    tapCount = 1;
  }
  lastTapTime = now;
}