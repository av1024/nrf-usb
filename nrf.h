#ifndef __NRF_H__
#define __NRF_H__


// Pin definitions for chip select chip enable and SPI
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168P__)
    #define CSN      PB2
    #define CE       PB1
    #define CLK      PB5
    #define MOSI     PB3
    #define MISO     PB4
    //IRQ1
    #define IRQ      PD3
    #define PORT_IRQ PORTD
    #define DDR_IRQ  DDRD
    #define DDR_ISP  DDRB
    #define PORT_ISP PORTB
#elif defined(__AVR_ATmega162__)
    #define CSN      PB4
    #define CE       PB3
    #define CLK      PB7
    #define MOSI     PB5
    #define MISO     PB6
    //IRQ1
    #define IRQ      PD3
    #define PORT_IRQ PORTD
    #define DDR_IRQ  DDRD
    #define DDR_ISP  DDRB
    #define PORT_ISP PORTB
#else
#error Undefined MCU
#endif


// Definitions for selecting and enabling the radio
#define CSN_HIGH      PORT_ISP |=  (1<<CSN);
#define CSN_LOW       PORT_ISP &= ~(1<<CSN);
#define CE_HIGH       PORT_ISP |=  (1<<CE);
#define CE_LOW        PORT_ISP &= ~(1<<CE);

#define PAYLOAD_WIDTH   32
#define ADDRESS_WIDTH   5

// define as 0 or 1 for corresponding INT0/INT1, other value for custom IRQ pin
// The only INT0/INT1 with falling edge initialization performed
// Comment out to disable IRQ
//#define IRQINT  1

#ifndef IRQINT
  // No IRQ handler
  #define MASK_IRQ_RX 1
  #define MASK_IRQ_TX 1
  #define MASK_IRQ_MAX 1
#else
  #define MASK_IRQ_RX 1
  #define MASK_IRQ_TX 0
  #define MASK_IRQ_MAX 0
#endif

// Since individual bits cannot be modified, all CONFIG settings must be set up at start up.
// EN_CRC enable CRC, CRCO set up 2 bytes CRC. As precaution, also MASK TX_DS and MAX_RT interrupt for RX
#define TX_POWERDWN       nrf_config_register(CONFIG, (1<<EN_CRC) | (1<<CRCO));
#define TX_POWERUP        nrf_config_register(CONFIG, \
    ((MASK_IRQ_TX)<<MASK_TX_DS) \
    | ((MASK_IRQ_MAX)<<MASK_MAX_RT) \
    | ((MASK_IRQ_RX)<<PWR_UP) \
    /*| (1<<EN_CRC) | (1<<CRCO) */);
#define RX_POWERUP        nrf_config_register(CONFIG, \
    ((MASK_IRQ_TX)<<MASK_TX_DS) \
    | ((MASK_IRQ_MAX)<<MASK_MAX_RT) \
    | ((MASK_IRQ_RX)<<MASK_RX_DR) \
    | (1<<PWR_UP) /*| (1<<EN_CRC) | (1<<CRCO) */ | (1<<PRIM_RX) );

#endif  //__NRF_H__
