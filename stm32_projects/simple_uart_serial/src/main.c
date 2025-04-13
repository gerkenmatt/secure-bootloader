#include "uart.h"

int main(void) {
    char buffer[100];

    usart_init();
    usart_puts("USART3 ready. Type something:\r\n");

    while (1) {
        usart_gets(buffer, sizeof(buffer));
        usart_puts("You typed: ");
        usart_puts(buffer);
        usart_puts("\r\n");
    }
}