#include <stdio.h>
#include <string.h> 
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#define PWM_pin 7

int main(){
    gpio_set_function(PWM_pin, GPIO_FUNC_PWM); // Set the LED Pin to be PWM
    uint slice_num = pwm_gpio_to_slice_num(PWM_pin); // Get PWM slice number
    float div = 40; // must be between 1-255
    pwm_set_clkdiv(slice_num, div); // divider
    uint16_t wrap = 62500; // when to rollover, must be less than 65535
    pwm_set_wrap(slice_num, wrap);
    pwm_set_enabled(slice_num, true); // turn on the PWM
    while(1) {
        pwm_set_gpio_level(PWM_pin, wrap / 40); // set the duty cycle to 50%
        sleep_ms(2000);
        pwm_set_gpio_level(PWM_pin, wrap / 8);
        sleep_ms(2000);
        pwm_set_gpio_level(PWM_pin, wrap / 40);
    }
    
}