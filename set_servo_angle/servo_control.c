#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#include "servo_control.h"

// Why are these values set as such? 
// according to : https://docs.espressif.com/projects/esp-iot-solution/en/latest/motor/servo.html
// Generally, there is a reference signal inside the servo generating a 
// fixed period and pulse width, which is used to compare with the input 
// PWM signal to output a voltage difference so as to control the rotation direction and angle 
// of a motor. A common 180 angular rotation servo usually takes 20 ms (50 Hz) as a clock period 
// and 0.5 ~ 2.5 ms as its high level pulse, making it rotates between 0 ~ 180 degrees.
#define SERVO_MIN_PULSEWIDTH 500 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2500 //Maximum pulse width in microsecond
#define SERVO_MAX_ANGLE 180 //Maximum angle in degree upto which servo can rotate


void init_servo_gpio(uint32_t gpio_pin){
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, gpio_pin);    //Set GPIO 18 as PWM0A, to which servo is connected
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings

}

void set_servo_angle(uint32_t angle){
    
    if (angle > SERVO_MAX_ANGLE) {
        angle = SERVO_MAX_ANGLE; 
    } else if (angle < 0) {
        angle = 0;
    }
    uint32_t pulse_width = 0;
    pulse_width = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (angle)) / (SERVO_MAX_ANGLE)));
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pulse_width);
}