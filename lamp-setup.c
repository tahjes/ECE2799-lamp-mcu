// Raspberry Pi Pico H is used in this implementation
/* Pin Inputs */
// ADC Input
// GPIO Fault Pin

/* Pin Outputs */
// PWM Output to driver
// DIR A and B for current flow

// Transitions:
// IDLE → CONNECT_LED if sensor has been tapped/slid
// CONNECT_LED → ON if successful installation of CoB
// OR CONNECT_LED → REVERSE_POLARITY if fault detected
// POLARITY REVERSE STATE → CONNECT_LED if fixed
// REVERSE_POLARITY → ERROR if fault status after flipping polarity
// ON → DISCONNECT_LED if tapped again
// ON → ON if slide (adjust PWM)
// DISCONNECT_LED → IDLE after disconnection

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
// #include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#define ANALOG_IN 34 // GP28
#define PIN_FAULT 32 // GP27 (power supply fault pin)
#define TEMP_IN 0 // Temp sensor input for CoB holder
#define DIR_A 31 // GP26 (power supply direct)
#define DIR_B 29 // GP22 
#define PWM_OUT 7 // GP5

// define each state
typedef enum {
    STATE_IDLE,
    STATE_CONN,
    STATE_POL, 
    STATE_ERR,
    STATE_ON,
    STATE_DISCONNECT
} State;

// globals 
bool tapped = 0; // tapped status for sensor input
bool fault_status = 0; // flags status of fault

// Previous Implementation: Functions are constantly polling, especially when they don't need to
// The only function that needs to be polled is the sensor input
// Strict timing unneeded for sensor input, however it will be the case for temp sensor and fault pin
void check_sensor(); 
// void check_fault();
void fault_callback(); // callback function for fault interrupt
// void temp_callback();
void reverse_polarity();

int main()
{

    stdio_init_all();

    /* Hardware Initialization */
    // --- fault detection input ---
    gpio_init(PIN_FAULT);
    gpio_set_dir(PIN_FAULT, GPIO_IN);
    gpio_pull_up(PIN_FAULT); // set pin high unless something turns it low

    // --- Current flow direction outputs A and B ---
    gpio_init(DIR_A);
    gpio_set_dir(DIR_A, GPIO_OUT);
    gpio_init(DIR_B);
    gpio_set_dir(DIR_B, GPIO_OUT);

    // enables interrupts on pin and associates a callback function that executes when int. occurs
    gpio_set_irq_enabled_with_callback(PIN_FAULT, GPIO_IRQ_EDGE_FALL, true, fault_callback);

    // temperature sensor interrupt
    // gpio_set_irq_enabled_with_callback(TEMP_IN, GPIO_IRQ_EDGE_RISE, true, temp_callback);

    // set initial Polarity (Anode goes to positive)
    gpio_put(DIR_A, 0);
    gpio_put(DIR_B, 1);

    // --- PWM Output (Will need to adjust slice, frequency, duty cycle)
    gpio_set_function(PWM_OUT, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(PWM_OUT);
    pwm_set_wrap(slice_num, 4095);
    pwm_set_enabled(slice_num, true);

    // Setup ADC2 Pin GP28
    adc_init();
    adc_gpio_init(ANALOG_IN);
    adc_select_input(2);

    State currentState = STATE_IDLE;

    while(1){
        // constantly checking the sensor and fault pin
        check_sensor();
        // check_fault();
        // define the conditions for each state
        switch(currentState) {
            case STATE_IDLE:
                // move to connected state if tapped status is 1
                if (tapped){
                    currentState = STATE_CONN;
                }
                break;
            case STATE_CONN:
                // Reverse polarity state of fault status is active
                if (fault_status){
                    currentState = STATE_POL;
                }
                // otherwise move to ON state
                else {
                    currentState = STATE_ON;
                }
                break;
            case STATE_POL:
                // call function to switch polarity
                reverse_polarity();
                sleep_ms(50);
                // check_fault();

                // No fault, turn on LED
                if (!fault_status){
                    currentState = STATE_ON;
                }
                // move to error state 
                else {
                    currentState = STATE_ERR;
                }
                break;                
            case STATE_ERR:
                // print error message
                printf("No CoB installed or not functional");
                // return to idle
                currentState = STATE_IDLE;
                break;
            case STATE_ON:
                uint16_t brightness = adc_read(); 
                pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWM_OUT), brightness);
                // move to disconnected state if the sensor is tapped again
                if (!tapped){
                    currentState = STATE_DISCONNECT;
                }
                break;
            case STATE_DISCONNECT:
                // LED is turned off, return to idle state
                currentState = STATE_IDLE;
                break;
        }
    }
}

// function definitions
// reads raw data coming from sensor via I2C (or ADC in this case)
void check_sensor(){
    uint16_t adc_value = adc_read();

    // threshold for when a user input is considered
    if (adc_value >= 820){
        tapped = 1;
    }
    // anything less than the position above does not get registered as an input
    else{
        tapped = 0;
    }
}

// used instead of polling for fault status from check_fault
void fault_callback(uint gpio, uint32_t events){
    bool fault = !gpio_get(PIN_FAULT); // fault pin is set as active low
    // Set status as fault if pin is low
    if (fault){
        fault_status = 1;
    }
    // otherwise no fault detected
    else{
        fault_status = 0;
    }
}

// void temp_callback(uint gpio, uint32_t events){
//     // 
// };

// reverse polarity in event of a fault
void reverse_polarity(){
    
    // Anode is set to Positive
    if ((gpio_get(DIR_A) == 0) && (gpio_get(DIR_B) == 1)){
        gpio_put(DIR_A, 1);
        gpio_put(DIR_B, 0);
    }
    // Anode set to Negative
    else if ((gpio_get(DIR_A) == 1) && (gpio_get(DIR_B) == 0)){
        gpio_put(DIR_A, 0);
        gpio_put(DIR_B, 1);
    }
    
}


// // check status of the fault pin
// void check_fault(){
//     bool fault = !gpio_get(PIN_FAULT); // fault pin is set as active low
//     // Set status as fault if pin is low
//     if (fault){
//         fault_status = 1;
//     }
//     // otherwise no fault detected
//     else{
//         fault_status = 0;
//     }
// }