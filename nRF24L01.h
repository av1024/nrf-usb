
#ifndef __NRF24L01_H__
#define __NRF24L01_H__

 /* Memory Map */
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define RPD         0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD       0x1C
#define FEATURE     0x1D

/* Bit Mnemonics */
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0
#define AW          0
#define ARD         4
#define ARC         0
#define PLL_LOCK    4
#define RF_DR_LOW   5
#define RF_DR_HIGH  3
#define RF_PWR      1
#define LNA_HCURR   0
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_TX_FULL   5
#define FIFO_TX_EMPTY    4
#define FIFO_RX_FULL     1
#define FIFO_RX_EMPTY    0

//DYNPD reg (ENAA_xx+EN_DPL required)
#define DPL_P0  0
#define DPL_P1  1
#define DPL_P2  2
#define DPL_P3  3
#define DPL_P4  4
#define DPL_P5  5

// FEATURE reg
#define EN_DPL      2
#define EN_ACK_PAY  1
#define EN_DYN_ACK  0

/* bitmask mnemonics */
// RF Power
#define RF_18DBM    0x00    /* -18dBm, 7.0mA */
#define RF_12DBM    0x02    /* -12dBm, 7.5mA */
#define RF_6DBM     0x04    /*  -6dBm, 9.0mA */
#define RF_0DBM     0x06    /*   0dBm, 11.3mA */
// RF Speed
#define RF_1MBPS    0x00
#define RF_2MBPS    0x08
#define RF_250KBPS  0x20

// adderss width:
#define AW_3        0x01
#define AW_4        0x10
#define AW_5        0x11

// retransmit delay
#define ARD_250    0x00    /* 250uS retransmit delay */
#define ARD_500    0x10
#define ARD_750    0x20
#define ARD_1000   0x30
#define ARD_1250   0x40
#define ARD_1500   0x50
#define ARD_1750   0x60
#define ARD_2000   0x70
#define ARD_2250   0x80
#define ARD_2500   0x90
#define ARD_2750   0xA0
#define ARD_3000   0xB0
#define ARD_3250   0xC0
#define ARD_3500   0xD0
#define ARD_3750   0xE0
#define ARD_4000   0xF0

#define ARC_NONE    0x00    /* retransmit disabled */
#define ARC_1       0x01    /* 1-times retransmit */
#define ARC_2       0x02
#define ARC_3       0x03
#define ARC_4       0x04
#define ARC_5       0x05
#define ARC_6       0x06
#define ARC_7       0x07
#define ARC_8       0x08
#define ARC_9       0x09
#define ARC_10      0x0A
#define ARC_11      0x0B
#define ARC_12      0x0C
#define ARC_13      0x0D
#define ARC_14      0x0E
#define ARC_15      0x0F

/* Instruction Mnemonics */
#define R_REGISTER          0x00
#define W_REGISTER          0x20
#define REGISTER_MASK       0x1F
#define R_RX_PAYLOAD        0x61
#define W_TX_PAYLOAD        0xA0
#define FLUSH_TX            0xE1
#define FLUSH_RX            0xE2
#define REUSE_TX_PL         0xE3
// Note(from datasheet): Flush RX FIFO if result value greater then 32 bytes
#define R_RX_PL_WID         0x60
//NB: set first 3 bits as pipe # (0b000...0b101)
#define W_ACK_PAYLOAD       0xA8
#define W_TX_PAYLOAD_NOACK  0xB0
#define NOP                 0xFF


/*
static void spi_init(void);
static void spi_transfer_sync (uint8_t *, uint8_t *, uint8_t);
static void spi_transmit_sync (uint8_t *, uint8_t);
static uint8_t spi_fast_shift (uint8_t);
*/

void nrf_init(void);


//void NRFConfig(NRFSetup *cfg);

void nrf_rx_config(uint8_t * config_buf,
                   uint8_t *rx0_addr,
                   uint8_t *rx1_addr,
                   uint8_t *tx_addr); // setup RX mode. !!!Should be called before TX setup!!!
void nrf_tx_config(uint8_t * addr); // Switch ***FROM RX*** to TX mode NB! call nrf_rx_config() first


uint8_t nrf_command(uint8_t);   // write 1 byte
uint8_t nrf_read(uint8_t reg);  // read 1 byte from register
void nrf_config_register(uint8_t reg, uint8_t value); // write 1 byte to register
void nrf_read_register(uint8_t reg, uint8_t * value, uint8_t len);    // read (len) bytes from register
void nrf_write_register(uint8_t reg, uint8_t * value, uint8_t len);

uint8_t nrf_send_completed(void);

// send (PAYLOAD_WIDTH) bytes from buffer
// return 1 on success, 0 if MAX_RT reached
uint8_t nrf_send(uint8_t *value, uint8_t len);
uint8_t nrf_data_available(void); // returns (pipe#+1) if data found, 0 if no data in pipe(s)
uint8_t nrf_get_payload(uint8_t *buf, uint8_t len); // fill buffer[PAYLOAD_WIDTH] with payload


#endif // __NRF24L01_H__
