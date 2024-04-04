#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

int main() {
    stdio_init_all();
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    printf("Start!\n");

    gpio_init(16);
    gpio_set_dir(16, GPIO_OUT);
    gpio_init(18); 
    gpio_set_dir(18, GPIO_IN);

    adc_init(); // init the adc module
    adc_gpio_init(26); // set ADC0 pin to be adc input instead of GPIO
    adc_select_input(0); // select to read from ADC0

    while (1) {
        gpio_put(16, 1);
        while (gpio_get(18) != 1) {
            sleep_ms(100);
        }
        gpio_put(16, 0);

        int sample_size;
        printf("Please enter sample size (integer): \r\n");
        scanf("%d", &sample_size);
        printf("Sample size: %d\r\n", sample_size);

        for (int ii = 0; ii < sample_size; ii++) {
            uint16_t result = adc_read();
            float voltage = (result/4095.0)*3.3;
            printf("adc: %.2fV\r\n", voltage);
            sleep_ms(100);
       }
    }
}