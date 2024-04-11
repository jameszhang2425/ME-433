
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include <math.h>

#define READ_BIT 0x80


#ifdef PICO_DEFAULT_SPI_CSN_PIN
static inline void cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
    asm volatile("nop \n nop \n nop");
}
#endif

#if defined(spi_default) && defined(PICO_DEFAULT_SPI_CSN_PIN)
static void write_register(unsigned int initialize, unsigned int pin) {
    uint8_t buf[2];

    uint8_t adcCounts4MostSignificant = (initialize >> 6);
    uint8_t adcCounts6LeastSignificant = (initialize & 0b0000111111);
    buf[0] = pin << 7; 
    buf[0] |= 0b01110000;
    buf[0] |= adcCounts4MostSignificant;
    buf[1] = (adcCounts6LeastSignificant << 2);

    cs_select();
    spi_write_blocking(spi_default, buf, 2);
    cs_deselect();

}

#endif

int main() {
    stdio_init_all();
#if !defined(spi_default) || !defined(PICO_DEFAULT_SPI_SCK_PIN) || !defined(PICO_DEFAULT_SPI_TX_PIN) || !defined(PICO_DEFAULT_SPI_RX_PIN) || !defined(PICO_DEFAULT_SPI_CSN_PIN)
#warning spi/mcp4912_spi example requires a board with SPI pins
    puts("Default SPI pins were not defined");
#else

    printf("Hello, mcp4912! Reading raw data from registers via SPI...\n");

    // This example will use SPI0 at 0.5MHz.
    spi_init(spi_default, 500 * 1000);
    //GP18
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    //GP19
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PICO_DEFAULT_SPI_CSN_PIN);
    gpio_set_dir(PICO_DEFAULT_SPI_CSN_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);

    unsigned int sinWave[100];
    unsigned int triangleWave[100];
    for (int i = 0; i < 100; i++) {
        float sineValue = sin(2 * 3.14 * i / 50);
        sinWave[i] = (unsigned int) ((sineValue + 1) / 2 * 1023); 
    }

    const unsigned int maxVal = 1023;
    const int halfCycle = 50; 

    for (int i = 0; i < 100; i++) {
        if (i < halfCycle) {
            triangleWave[i] = (unsigned int)(((double)i / (halfCycle - 1)) * maxVal);
        } else {
            triangleWave[i] = (unsigned int)(((double)(100 - i - 1) / (halfCycle - 1)) * maxVal);
        }
    }

    while(1){
        for (unsigned int i = 0; i < 100; i++) {
            write_register(sinWave[i], 0);
            sleep_ms(5);
            write_register(triangleWave[i], 1);
            sleep_ms(5);
        }
    }
    #endif
}
