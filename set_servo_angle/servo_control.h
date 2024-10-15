#pragma once


#ifdef __cplusplus
extern "C" {
#endif


void init_servo_gpio(uint32_t gpio_pin);
void set_servo_angle(uint32_t angle);


#ifdef __cplusplus
}
#endif

