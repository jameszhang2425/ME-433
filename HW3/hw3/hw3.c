#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

int main() {
    stdio_init_all();
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    printf("Start!\n");

    gpio_init(16); // init pin 16 as output
    gpio_set_dir(16, GPIO_OUT);

    gpio_init(18); // init pin 18 as input
    gpio_set_dir(18, GPIO_IN);

    adc_init(); // init the adc module
    adc_gpio_init(26); // set ADC0 pin to be adc input instead of GPIO
    adc_select_input(0); // select to read from ADC0

    while (1) {
        // turn LED on
        gpio_put(16, 1);

        // wait for button input
        while (gpio_get(18) != 1) {
            sleep_ms(10);
        }

        // if gotten here, turn LED off
        gpio_put(16, 0);

        // read number of samples requested
        char message[100];
        int samples;
        printf("Please enter number of analog samples: \r\n");
        scanf("%d", &samples);
        printf("%d samples requested\r\n", samples);

        // reads ADC and prints voltages that number of times
        for (int i = 0; i < samples; i++) {
            uint16_t result = adc_read();
            float volts = (result / 4096.0) * 3.3;
            printf("adc: %.2fV\r\n", volts);
            sleep_ms(10);
        }
    }
}