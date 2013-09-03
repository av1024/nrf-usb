/*

TEST BLINK APP

*/

//#define F_CPU 12000000L
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <stdlib.h>
//#define UART_BLOCK

#include "gcc_macro.h"
#include "nrf.h"
#include "nrf/nRF24L01.h"
#include "uart.h"

#define LED_R 5
#define LED_B 6

#define LED_R_ON    PORTD |= (1<<LED_R)
#define LED_R_OFF   PORTD &= ~(1<<LED_R)
#define LED_B_ON    PORTD |= (1<<LED_B)
#define LED_B_OFF   PORTD &= ~(1<<LED_B)
#define LED_OFF     PORTD &= ~(1<<LED_B | 1<<LED_R)

#define RF_CHANNEL 110

uint8_t nrf_rx0_addr[ADDRESS_WIDTH] = {0xe7, 0xe7, 0xe7, 0xe7, 0xe7};
uint8_t nrf_tx_addr[ADDRESS_WIDTH] = {0xe7, 0xe7, 0xe7, 0xe7, 0xe7};
uint8_t nrf_rx1_addr[ADDRESS_WIDTH] = {0xc2, 0xc2, 0xc2, 0xc2, 0xc2};
uint8_t nrf_cfg[] = {
    (1<<ENAA_P0) | (1<<ENAA_P1),
    //0,  //TEST: disable auto-ack for RF monitoring
    (1<<ERX_P0) | (1<<ERX_P1),
    AW_5, // 5 bytes address
    ARC_5 | ARD_1000, // 1250ms delay, 5 retransmits
    RF_CHANNEL,
    RF_1MBPS | RF_0DBM
};

uint8_t nrf_fail = 0;

void setup(void) {

    // LEDS
    DDRD |= (1<<LED_R) | (1<<LED_B);
    LED_B_ON;
    LED_R_ON;
    // RS232
    uart_init(57600);

    // NRF
    nrf_init();

    nrf_rx_config(nrf_cfg,
                  nrf_rx0_addr,
                  nrf_rx1_addr,
                  nrf_tx_addr);

    nrf_config_register(FEATURE, (1<<EN_DPL));
    nrf_config_register(DYNPD, (1<<DPL_P0)|(1<<DPL_P1)|(1<<DPL_P2)|(1<<DPL_P3)|(1<<DPL_P4)|(1<<DPL_P5));
    //nrf_tx_config(nrf_rx_addr);

    //nrf_write_register(RX_ADDR_P1, nrf_tx_addr, 5);


    //nrf_config_register(RX_PW_P0, 32); // test!!!
    //nrf_config_register(RX_PW_P1, 32); // test!!!

    //nrf_config_register(FEATURE, 0);
    //nrf_config_register(DYNPD, 0);

    RX_POWERUP;
    _delay_us(150);
    CE_HIGH;
    LED_B_OFF;
    LED_R_OFF;
    sei();
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

void processNRF(uint8_t *buf, uint8_t sz, uint8_t pipe) {
    uint32_t cntr;
    uint8_t *c = (uint8_t *)&cntr;
    uint8_t i;
    uint8_t x;
    char tmp[12];
    sz &= 0x1F;
    if (sz < 2) {
        uart_print("RF size\n");
    } else {
        uart_print("From[");
        uart_print_hex(pipe);
        uart_print("] ");
        uart_print_hex(*buf++);
        uart_print(": ");
        sz--;
        switch(*buf) {
            case 'o':
                uart_print("ok\n");
                break;
            case 'a':
                uart_print("VCC: ");
                uart_print_dec(buf[1]/10);
                uart_putch('.');
                uart_print_dec(buf[1]%10);
                uart_print("V, Light: ");
                uart_print_dec(buf[2]);
                uart_print(", Counter #1: ");
                *c++ = buf[3];
                *c++ = buf[4];
                *c++ = buf[5];
                *c = buf[6];
                utoa(cntr, tmp, 10);
                uart_print(tmp);
                uart_print(", Counter #2: ");
                c = (uint8_t*)&cntr;
                *c++ = buf[7];
                *c++ = buf[8];
                *c++ = buf[9];
                *c = buf[10];
                utoa(cntr, tmp, 10);
                uart_print(tmp);
                uart_println();
                break;
            case 't':
                uart_print("T [");
                buf++;
                for(i=0;i<8;i++) uart_print_hex(*buf++);
                uart_print("]: ");
                if (((signed char)*buf) < 0) {
                    uart_putch('-');
                    uart_print_dec((uint8_t)(-(signed char)(*buf)));
                } else {
                    if (!(*buf)) uart_putch(' ');
                    else uart_putch('+');
                    uart_print_dec(*buf);
                }
                uart_putch('.');
                buf++;
                uart_print_dec(*buf);
                uart_print("C\n");
                //uart_tx_flush();
                break;
            default:
                buf--;
                uart_print("DATA ");

                for(register uint8_t i=0;i<32;i++) {
                    uart_print_hex(buf[i]);
                    uart_print(" ");
                    //if (buf[1] == 0) break;
                }
                uart_println();
        }
    }
    uart_tx_flush();

}

int main(void) {
    register uint8_t i;
    register uint8_t stat;
    uint8_t buf[32];
    uint8_t prev_rpd = 0;
    uint8_t chan = 1;
    uint16_t cnt = 0;

    setup();
    /*for(;;) {
        LED_R_ON;
        _delay_ms(100);
        uart_putch('.');
        LED_R_OFF;
        _delay_ms(900);
    }*/
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
        //nrf_rx_config(nrf_rx_addr);
        RX_POWERUP;
        _delay_ms(900);
        //LED_B_ON;
        _delay_ms(100);
        //LED_B_OFF;


        stat = nrf_command(NOP);
        // check if NRF module unplugged
        if (stat == 0xFF) {
            LED_R_ON;
            nrf_fail = 1;
            _delay_ms(500);
            LED_R_OFF;
            _delay_ms(500);
            uart_print("NRF down\n");
            setup();
            continue;
        }
        //uart_print_hex(stat);
        //uart_putch(' ');
        stat = nrf_command(NOP);
        if (stat & ((1<<MAX_RT) | (1<<TX_FULL))) {
            nrf_config_register(STATUS, (1<<MAX_RT) | (1<<TX_FULL));
            nrf_command(FLUSH_TX);
        }

        while (nrf_data_available())
        {
	    LED_R_ON;
            for(i=31;i>0;i--) buf[i] = 0;
            i = nrf_command(NOP);
            stat = nrf_read(R_RX_PL_WID);
            //uart_print("cz:");
            //uart_print_dec(stat);
            //uart_println();
            if (stat > 32) {
                nrf_command(FLUSH_RX);
                nrf_config_register(STATUS, (1<<RX_DR));
                continue;
            }

            i = nrf_command(NOP);
            nrf_get_payload(buf, stat); // pipe + 1 or 0
            nrf_config_register(STATUS, (1<<RX_DR));
            //uart_send...
            //if (i) {

                processNRF(buf, stat, i);

            //}
            //else {

            //}
	    LED_R_OFF;

        }

    }

    return 0;
}
