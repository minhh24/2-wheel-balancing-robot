#include "pico_uart_transports.h"
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

// --- CẤU HÌNH UART CHO MICRO-ROS ---
#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

// Hàm lấy thời gian cho Micro-ROS
int clock_gettime(clockid_t unused, struct timespec *tp) {
    uint64_t m = time_us_64();
    tp->tv_sec = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;
    return 0;
}

bool pico_serial_transport_open(struct uxrCustomTransport * transport) {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
    // Tắt Flow control
    uart_set_hw_flow(UART_ID, false, false);
    
    // Định dạng: 8 data bits, 1 stop bit, no parity
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    
    // Bật FIFO
    uart_set_fifo_enabled(UART_ID, true);
    return true;
}

bool pico_serial_transport_close(struct uxrCustomTransport * transport) {
    uart_deinit(UART_ID);
    return true;
}

// SỬA LỖI: Thêm 'const' vào buf để khớp với .h
size_t pico_serial_transport_write(struct uxrCustomTransport * transport, const uint8_t *buf, size_t len, uint8_t *errcode) {
    uart_write_blocking(UART_ID, buf, len);
    return len;
}

size_t pico_serial_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode) {
    uint64_t start_time_us = time_us_64();
    for (size_t i = 0; i < len; i++) {
        int64_t elapsed_time_us = time_us_64() - start_time_us;
        int64_t remaining_us = (timeout * 1000) - elapsed_time_us;

        if (remaining_us <= 0) {
            *errcode = 1;
            return i;
        }

        if (!uart_is_readable_within_us(UART_ID, remaining_us)) {
            *errcode = 1;
            return i;
        }
        
        buf[i] = uart_getc(UART_ID);
    }
    return len;
}
