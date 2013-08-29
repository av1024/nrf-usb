/*


 Usage:
 1. Define TX/RX pin for 485 communication in main program
 #define RS485PIN   PD0
 #define RS485PORT  PORTD
 #define RS485DDR   DDRD

 2. call uartInit(baudrate)
*/

#ifndef __UART_H__
#define __UART_H__

#include <avr/pgmspace.h>

#define UART_RX_BUF_SIZE    128
#define UART_TX_BUF_SIZE    128

//#define RS485PIN   PD0
//#define RS485PORT  PORTD
//#define RS485PORTD DDRD

// **** RS485 TX control support ****
#ifdef RS485PIN
 #if !defined(RS485PORT) || !defined(RS485PORTD)
  #error("RS485 pin defined but no RS485PORT/RS485PORTD.")
 #endif
#endif  // RS485PIN


// dual-port fix
#ifndef UDRE
    #define UDR     (UDR0)
    #define UDRE    (UDRE0)
    #define UBRRH   (UBRR0H)
    #define UBRRL   (UBRR0L)
    #define UCSRA   (UCSR0A)
    #define UCSRB   (UCSR0B)
    #define UCSRC   (UCSR0C)
    #define RXCIE   (RXCIE0)
    #define TXCIE   (TXCIE0)
    #define UDRIE   (UDRIE0)
    #define RXEN    (RXEN0)
    #define TXEN    (TXEN0)
    #define U2X     (U2X0)
    #define URSEL   (URSEL0)
    #define UCSZ0   (UCSZ00)
    #define UCSZ1   (UCSZ01)
    #define UCSZ2   (UCSZ02)
    #define TXC     (TXC0)
    #define RXC     (RXC0)

#endif

#if defined(__AVR_ATmega88__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
  #define USART0_UDRE_vect     (USART_UDRE_vect)
  #define USART0_TXC_vect     (USART_TX_vect)
  #define USART0_RXC_vect     (USART_RX_vect)
#endif

// from arduino code (works on 57600 @ 8MHz):
#define BAUD(x) ( (F_CPU/8/x-1) / 2 )

// blocking printing
// define UART_BLOCK 1 for block uart_putch() if no room for next char
//#define UART_BLOCK

#define uart_pprint(x)  uart_print_p(PSTR(x))
#define uart_rx_empty() uart_rx_count()
#define uart_tx_empty() uart_tx_count()
#define uart_init(baud) uart_init_b((((F_CPU >> 3)/baud-1) >> 1))
void uart_init_b(uint16_t ubrr);  // Use `uart_init` for baud rate initialization

// Put char into queue.
// *** MAY BLOCK *** if UART_BLOCK defined and no room in buffer
uint8_t uart_putch(uint8_t data);

// Get char from queue. Return 0 if no data
uint8_t uart_getch(void);

uint8_t uart_rx_count(void);
uint8_t uart_tx_count(void);
void uart_rx_flush(void);
void uart_tx_flush(void);   // Block until all data transmitted
uint8_t uart_print(const char *str);
uint8_t uart_print_hex(uint8_t b);
uint8_t uart_print_dec(uint8_t b);
uint8_t uart_print_bin(uint8_t b);

uint8_t uart_print_p(const char *str);
uint8_t uart_println(void);

uint8_t uart_print_mem(uint8_t *mem, uint8_t count);
uint8_t uart_print_mem_d(uint8_t *mem, uint8_t count, char delim);

#endif  // __UART_H__
