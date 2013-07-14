/*

TEST BLINK APP

*/

#define F_CPU 12000000L
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "gcc_macro.h"
#include "nrf.h"
#include "nRF24L01.h"
#include "uart.h"

#define LED_R 5
#define LED_B 6

#define LED_R_ON    PORTD |= (1<<LED_R)
#define LED_R_OFF   PORTD &= ~(1<<LED_R)
#define LED_B_ON    PORTD |= (1<<LED_B)
#define LED_B_OFF   PORTD &= ~(1<<LED_B)
#define LED_OFF     PORTD &= ~(1<<LED_B | 1<<LED_R)

#define RF_CHANNEL 1

uint8_t nrf_rx_addr[ADDRESS_WIDTH] = {0xe7, 0xe7, 0xe7, 0xe7, 0xe7};
uint8_t nrf_cfg[] = {
    (1<<ENAA_P0), // | (1<<ENAA_P1),
    //0,  //TEST: disable auto-ack for RF monitoring
    //(1<<ERX_P0) | (1<<ERX_P1),
    (1<<ERX_P0),
    AW_3, // 3 bytes address
    ARC_5 | ARD_1250, // 1250ms delay, 5 retransmits
    RF_CHANNEL,
    RF_1MBPS | RF_0DBM
};

uint8_t nrf_fail = 0;

void setup(void) {

    // LEDS
    DDRD |= (1<<LED_R) | (1<<LED_B);
    LED_B_ON;
    // RS232
    uart_init(57600);
    // NRF
    nrf_init();
    nrf_rx_config(nrf_rx_addr);

    nrf_config_register(RX_PW_P0, 3); // test!!!
    nrf_config_register(RX_PW_P1, 3); // test!!!

    nrf_config_register(FEATURE, (1<<EN_DPL));
    nrf_config_register(DYNPD, (1<<DPL_P0));

    RX_POWERUP;
    _delay_us(150);
    CE_HIGH;
    LED_B_OFF;
}


void prt_cfg(void) {
    uint8_t x;
    uint8_t addr[5];
    uart_pprint("CONFIG");
    for(uint8_t i=0;i<7;i++) {
        x = nrf_read(i);
        uart_pprint(":");
        uart_print_hex(x);
    }
    uart_println();


    uart_pprint("CD: ");
    x = nrf_read(RPD);
    uart_print_dec(x);

    x = nrf_read(OBSERVE_TX);
    uart_pprint(" PLOS: ");
    uart_print_dec(x >> 4);
    uart_pprint(" ARC: ");
    uart_print_dec(x & 0x0F);

    x = nrf_read(FEATURE);
    uart_print(" FEATURE: 0b");
    uart_print_bin(x);
    x = nrf_read(DYNPD);
    uart_print(" DYNPD: 0b");
    uart_print_bin(x);
    uart_println();

    x = nrf_command(NOP);
    uart_pprint("STATUS: 0b");
    uart_print_bin(x);
    x = nrf_read(FIFO_STATUS);
    uart_pprint(", FIFO: 0b");
    uart_print_bin(x);
    uart_println();

    nrf_read_register(RX_ADDR_P0, addr, 5);
    uart_pprint("RX_P0: ");
    for(x=0;x<5;x++) uart_print_hex(addr[x]);
    nrf_read_register(RX_ADDR_P1, addr, 5);
    uart_pprint(" RX_P1: ");
    for(x=0;x<5;x++) uart_print_hex(addr[x]);

    nrf_read_register(TX_ADDR, addr, 5);
    uart_pprint(" TX: ");
    for(x=0;x<5;x++) uart_print_hex(addr[x]);
    uart_println();

    /*
    uart_pprint("SPI REGS/SPCR,SPSR,DDRB,PORTB,PINB,PORTD,PIND/: ");
        uart_print_hex(SPCR);
        uart_putch(' ');
        uart_print_hex(SPSR);
        uart_putch(' ');
        uart_print_hex(DDRB);
        uart_putch(' ');
        uart_print_hex(PORTB);
        uart_putch(' ');
        uart_print_hex(PINB);
        uart_putch(' ');
        uart_print_hex(PORTD);
        uart_putch(' ');
        uart_print_hex(PIND);
        uart_println();
    */

}

int main(void) {
    register uint8_t i;
    register uint8_t stat;
    uint8_t buf[32];
    uint8_t prev_rpd = 0;
    uint8_t chan = 1;
    uint16_t cnt = 0;

    setup();
    uart_pprint("NRF-USB\n");
    /*
        LED_R_ON;
        _delay_ms(500);
        LED_R_OFF;
        _delay_ms(450);
        LED_B_ON;
        _delay_ms(50);
        LED_B_OFF;
     */

     prt_cfg();

    while (1) {
        stat = nrf_command(NOP);
        // check if NRF module unplugged
        if (stat == 0xFF) {
            LED_R_ON;
            nrf_fail = 1;
            _delay_ms(500);
            LED_R_OFF;
            _delay_ms(500);
            setup();
            continue;
        }

        nrf_fail = 0;
        //nrf_command(NOP);
        i = nrf_read(FIFO_STATUS);
        if (i & ((1<<FIFO_TX_FULL) | (1<<FIFO_RX_FULL))) {
            LED_R_ON;
            if (i & (1<<FIFO_TX_FULL)) nrf_command(FLUSH_TX);
            if (i & (1<<FIFO_RX_FULL)) nrf_command(FLUSH_RX);
            LED_R_OFF;
        }

        i = nrf_read(RPD);
        if (i & (0x1) && !prev_rpd) {
            LED_B_ON;
            prev_rpd = 1;
            uart_print("CH ");
            uart_print_dec(chan);
            uart_print(": CD\n");
        } else {
            prev_rpd = 0;
            if (!(i & 1)) LED_B_OFF;
        }

        if (nrf_data_available()) {

            for(i=0;i<32;i++) buf[i] = 0;
            stat = nrf_read(R_RX_PAYLOAD);
            i = nrf_get_payload(buf, 32);
            nrf_config_register(STATUS, (1<<RX_DR));
            //uart_send...
            if (i) {
                uart_print("CH ");
                uart_print_dec(chan);
                uart_print("[");
                uart_print_dec(stat);
                uart_print("]: ");
                uart_print_dec(i);

                uart_print(": ");
                for(i=31;i>31-3;i--) {
                    uart_print_hex(buf[i]);
                    uart_print(" ");
                    if (buf[1] == 0) break;
                }
                uart_println();
            }
            _delay_ms(1);

        }
        _delay_ms(1);
        cnt++;
        if (cnt > 3000) {
            LED_R_ON;
            i = nrf_read(FIFO_STATUS);
            chan++;
            chan &= 0x7F;
            cnt = 0;
            CE_LOW;
            nrf_config_register(RF_CH, chan);
            RX_POWERUP;
            _delay_us(150);
            CE_HIGH;
            //prt_cfg();
            LED_R_OFF;
        }
    }
    return 0;
}
