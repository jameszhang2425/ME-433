#include <stdio.h>
#include <string.h> 
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardwware/pwm.h"
gpio_set_function(15, GPIO_FUNC_PWM); // Set the LED Pin to be PWM
uint slice_num = pwm_gpio_to_slice_num(15); // Get PWM slice number
float div = 2; // must be between 1-255
pwm_set_clkdiv(slice_num, div); // divider
uint16_t wrap = 62500; // when to rollover, must be less than 65535
pwm_set_wrap(slice_num, wrap);
pwm_set_enabled(slice_num, true); // turn on the PWM

pwm_set_gpio_level(15, wrap / 2); // set the duty cycle to 50%