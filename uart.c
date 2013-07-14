// uart.c
// custom interrupt-driven buffered UART implementation

#include <avr/io.h>
#include <avr/interrupt.h>

#include "gcc_macro.h"
#include "uart.h"

volatile static uint8_t __tx_buf[UART_TX_BUF_SIZE];
volatile static uint8_t __rx_buf[UART_RX_BUF_SIZE];


volatile static uint8_t rx_t=0, rx_cnt=0,
        tx_h=0, tx_cnt=0,
        rx_h=0, tx_t=0;


//#define BAUD(x) ( (F_CPU/8/x-1) / 2 )
//#define void uart_init(baud) uart_init_b((((F_CPU >> 3)/baud-1) >> 1))
void uart_init_b(uint16_t ubrr) {
    // baud rate
    UBRRH = (uint8_t) (ubrr >> 8);
    UBRRL = (uint8_t) ubrr;

    // Clear Double-speed mode
    UCSRA = 0; //(1<<U2X);

    // Enable all interrupts and tx/rx lines
    //UCSRB = (1<<RXCIE) | (1<<TXCIE) | (1<<UDRIE) | (1<<RXEN) | (1<<TXEN);
    UCSRB =   (1<<RXCIE)
            #ifdef RS485PIN
            | (1<<TXCIE)
            #endif
            | (1<<RXEN)
            | (1<<TXEN)
            ;

    // Frame format 8-N-1
    #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega88__)
      UCSRC = (1<<UCSZ00) | (1<<UCSZ01);
    #else
      UCSRC = (1<<URSEL) | (1<<UCSZ00) | (1<<UCSZ01);
    #endif

    // RS485 support
    #ifdef RS485PIN
    RS485PORTD |= (1<<RS485PIN);
    RS485PORT &= ~(1<<RS485PIN);
    #endif
}

NOINLINE void uart_putch(uint8_t data) {

    #if defined(UART_BLOCK)
    #warning BLOCKING UART MODE
    uint8_t srg = SREG;
    sei();
    //FIXME: add anti-lock timer
    while(tx_cnt >= UART_TX_BUF_SIZE) { ; }
    SREG = srg;
    #endif

    #ifdef RS485PORT
        RS485PORT |= (1<<RS485PIN); // switch to TX
    #endif

    //while ( !(UCSRA & (1<<UDRE)) ); // waitfor tx empty
    if ( (UCSRA & (1<<UDRE)) && (tx_cnt==0) ) {
        UDR = data;
        UCSRB |= (1<<UDRIE);
    } else
        if (tx_cnt < UART_TX_BUF_SIZE) {
            __tx_buf[tx_t] = data;
            tx_cnt++;
            tx_t++;
            if ( tx_t==UART_TX_BUF_SIZE ) tx_t = 0;
            UCSRB |= (1<<UDRIE);
        }

}

uint8_t uart_getch(void) {
    uint8_t ch = 0;
    if (rx_cnt > 0) {
        ch = __rx_buf[rx_h];
        rx_cnt--;
        rx_h++;
        if ( rx_h == UART_RX_BUF_SIZE ) rx_h = 0;
    }
    return ch;
}

INLINE uint8_t uart_rx_empty(void) {
    return (rx_cnt == 0);
}

INLINE uint8_t uart_rx_count(void) {
    return rx_cnt;
}

INLINE uint8_t uart_tx_empty(void) {
    return (tx_cnt == 0);
}

INLINE uint8_t uart_tx_count(void) {
    return tx_cnt;
}

void uart_rx_flush(void) {
    rx_cnt = 0;
    rx_t = 0;
    rx_h = 0;
}

void uart_tx_flush(void) {
    if (tx_cnt) {
        UCSRB |= (1<<UDRIE);
        while(tx_cnt) { ; }

        UCSRB &= ~(1<<UDRIE);
    }
    tx_h = 0;
    tx_t = 0;
}

void uart_print(const char *str) {
    while (*str ) {
        uart_putch(*str++);
    }
}

inline void uart_println(void){
    uart_putch('\r');
    uart_putch('\n');
}

void uart_print_hex(uint8_t b) {
    uint8_t c = b >> 4;
    if (c > 9) c += 7;
    uart_putch(c + '0');
    c = b & 0x0F;
    if (c > 9) c += 7;
    uart_putch(c + '0');
}

void uart_print_dec(uint8_t b) {
    uint8_t c, x, z;
    c = (b / 100); // x100
    x = b % 100;
    z = x / 10;    // x10
    if (c) uart_putch(c + '0');     // >= 100
    if (b > 9) uart_putch(z + '0'); // >= 10
    uart_putch('0' + (x % 10));     // 0..9
}

void uart_print_bin(uint8_t b) {
    for(uint8_t i=0;i<8;i++) {
        if ( b & (0x80 >> i) ) uart_putch('1');
        else uart_putch('0');
    }
}

void uart_print_mem(uint8_t *mem, uint8_t count) {
    while(count--)
        uart_print_hex(*mem++);
}

void uart_print_mem_d(uint8_t *mem, uint8_t count, char delim) {
    for(register uint8_t i=0;i<count;i++) {
        if (i) uart_putch(delim);
        uart_print_hex(*mem++);
    }
}

void uart_print_p(const char *str) {
    uint8_t c;
    do {
        c = pgm_read_byte(str++);
        if (c != 0) uart_putch(c);
    } while (c);
}


#ifdef RS485PIN
ISR(USART0_TXC_vect) {
    RS485PORT &= ~(1<<RS485PIN);
}
#endif

ISR(USART0_UDRE_vect) {
    cli();
    if (tx_cnt > 0) {
        UDR = __tx_buf[tx_h];
        tx_cnt--;
        tx_h++;
        if ( tx_h == UART_TX_BUF_SIZE) tx_h = 0;
    }
    else {
        UCSRB &= ~(1<<UDRIE);
        tx_h = 0;
        tx_t = 0;
    }
    sei();
}

ISR(USART0_RXC_vect) {
    cli();
    if (rx_cnt < UART_RX_BUF_SIZE) {
        __rx_buf[rx_t] = UDR;
        rx_t++;
        if ( rx_t == UART_RX_BUF_SIZE ) rx_t = 0;
        rx_cnt++;
    }
    sei();
}
