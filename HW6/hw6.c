// based on adafruit and sparkfun libraries
#include <stdio.h>
#include <string.h> // for memset
#include "hardware/adc.h"
#include "hw6.h"
#include "font.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

unsigned char SSD1306_ADDRESS = 0b0111100; // 7bit i2c address
unsigned char ssd1306_buffer[513]; // 128x32/8. Every bit is a pixel except first byte

void ssd1306_setup() {
    // first byte in ssd1306_buffer is a command
    ssd1306_buffer[0] = 0x40;
    sleep_ms(20);
    ssd1306_command(SSD1306_DISPLAYOFF);
    ssd1306_command(SSD1306_SETDISPLAYCLOCKDIV);
    ssd1306_command(0x80);
    ssd1306_command(SSD1306_SETMULTIPLEX);
    ssd1306_command(0x1F); // height-1 = 31
    ssd1306_command(SSD1306_SETDISPLAYOFFSET);
    ssd1306_command(0x0);
    ssd1306_command(SSD1306_SETSTARTLINE);
    ssd1306_command(SSD1306_CHARGEPUMP);
    ssd1306_command(0x14);
    ssd1306_command(SSD1306_MEMORYMODE);
    ssd1306_command(0x00);
    ssd1306_command(SSD1306_SEGREMAP | 0x1);
    ssd1306_command(SSD1306_COMSCANDEC);
    ssd1306_command(SSD1306_SETCOMPINS);
    ssd1306_command(0x02);
    ssd1306_command(SSD1306_SETCONTRAST);
    ssd1306_command(0x8F);
    ssd1306_command(SSD1306_SETPRECHARGE);
    ssd1306_command(0xF1);
    ssd1306_command(SSD1306_SETVCOMDETECT);
    ssd1306_command(0x40);
    ssd1306_command(SSD1306_DISPLAYON);
    ssd1306_clear();
    ssd1306_update();
}

// send a command instruction (not pixel data)
void ssd1306_command(unsigned char c) {

    uint8_t buf[2];
    buf[0] = 0x00;
    buf[1] =c;
    i2c_write_blocking(i2c_default, SSD1306_ADDRESS, buf, 2, false);
}

// update every pixel on the screen
void ssd1306_update() {
    ssd1306_command(SSD1306_PAGEADDR);
    ssd1306_command(0);
    ssd1306_command(0xFF);
    ssd1306_command(SSD1306_COLUMNADDR);
    ssd1306_command(0);
    ssd1306_command(128 - 1); // Width

    unsigned short count = 512; // WIDTH * ((HEIGHT + 7) / 8)
    unsigned char * ptr = ssd1306_buffer; // first address of the pixel buffer


    i2c_write_blocking(i2c_default, SSD1306_ADDRESS, ptr, 513, false);
}

// set a pixel value. Call update() to push to the display)
void ssd1306_drawPixel(unsigned char x, unsigned char y, unsigned char color) {
    if ((x < 0) || (x >= 128) || (y < 0) || (y >= 32)) {
        return;
    }

    if (color == 1) {
        ssd1306_buffer[1 + x + (y / 8)*128] |= (1 << (y & 7));
    } else {
        ssd1306_buffer[1 + x + (y / 8)*128] &= ~(1 << (y & 7));
    }
}

// zero every pixel value
void ssd1306_clear() {
    memset(ssd1306_buffer, 0, 512); // make every bit a 0, memset in string.h
    ssd1306_buffer[0] = 0x40; // first byte is part of command
}

void draw_char(int x, int y, char letter){
    for (int ii = 0; ii < 5; ii++){
        char c = ASCII[letter-32][ii];
        for (int jj = 0; jj < 8; jj++){
            char bit = (c>>jj)&0b1;
            if (bit == 0b1){
                ssd1306_drawPixel(x+ii, y+jj, 1);
            } else{
                ssd1306_drawPixel(x+ii, y+jj, 0);
            }
        }
    }
}

void draw_message(int x, int y, char*m){
    int kk = 0;
    while(m[kk]){
        draw_char(x+kk*5, y, m[kk]);
        kk++;
    }
    ssd1306_update();
}

#define LED_PIN 25 

int main(){
    stdio_init_all();
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);

    ssd1306_setup();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    adc_init(); 
    adc_gpio_init(26); 
    adc_select_input(0); 

    char message[50];
    char fps_message[50];
    while(1){
        gpio_put(LED_PIN, 1);
        uint16_t adc_val = adc_read();
        float adc_volts = (float)adc_val/1212.0;

        sprintf(message, "ADC(VOLTS)= %f", adc_volts);

        unsigned int start = to_us_since_boot(get_absolute_time());
        draw_message(5, 5, message);
        unsigned int stop = to_us_since_boot(get_absolute_time());
        unsigned int dt = stop - start;

        sprintf(fps_message, "FPS= %f", 1000000.0/dt);
        draw_message(5, 15, fps_message);
        sleep_ms(250);
        
        gpio_put(LED_PIN, 0);
        sleep_ms(250);
    }
}

