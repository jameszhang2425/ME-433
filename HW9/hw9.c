#include <stdio.h>
#include <string.h> 
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define LED_PIN 25

volatile int i = 0;
char rxm[100];

void on_uart_rx(){
    while (uart_is_readable(UART_ID)){
        uint8_t ch = uart_getc(UART_ID);
        if (ch == '\n') {
        rxm[i] = 0;
        i = 0;
        printf("From Zero: %s\n", rxm);
        } else {
            rxm[i] = ch;
            i++;
        }
    }
}

int main(){
    stdio_init_all();

    uart_init(UART_ID, 2400);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    int __unused actual = uart_set_baudrate(UART_ID, BAUD_RATE);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_ID, false);
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    while(1) {
        int i_comp = 0;
        scanf("%d", &i_comp);
        printf("Computer sent: %d\n", i_comp);
        char txm[100];
        sprintf(txm, "%d\n", i_comp);
        uart_puts(UART_ID, txm);
        sleep_ms(250);
    }
}