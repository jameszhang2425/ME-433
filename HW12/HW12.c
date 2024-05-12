#include <stdio.h>
#include <string.h> 
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#define PWM_pin 7
#define LED_PIN 25 

int main(){
    gpio_set_function(PWM_pin, GPIO_FUNC_PWM); // Set the LED Pin to be PWM
    uint slice_num = pwm_gpio_to_slice_num(PWM_pin); // Get PWM slice number
    float div = 40; // must be between 1-255
    pwm_set_clkdiv(slice_num, div); // divider
    uint16_t wrap = 62500; // when to rollover, must be less than 65535
    pwm_set_wrap(slice_num, wrap);
    pwm_set_enabled(slice_num, true); // turn on the PWM

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    while(1) {
        gpio_put(LED_PIN, 1);
        pwm_set_gpio_level(PWM_pin, wrap / 40); 
        sleep_ms(2000);
        gpio_put(LED_PIN, 0);
        pwm_set_gpio_level(PWM_pin, wrap / 8);
        sleep_ms(2000);
        gpio_put(LED_PIN, 1);
        pwm_set_gpio_level(PWM_pin, wrap / 40);
    }
    
}