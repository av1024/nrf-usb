/*
 * nRF24L01.c
 *
 * Created: 3/23/2012 6:48:23 PM
 *  Author: John
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "nrf.h" // pin defines
#include "nRF24L01.h"

/*
uint8_t EEMEM CH;
uint8_t EEMEM eeNODE_NUMBER;
uint8_t EEMEM eeNRF_ADDRESS[5];
uint8_t NODE_NUMBER;
*/


/* DEBUG: wait with lock-up protection
#define SPI_WAIT PORTC &= ~(1<<PC4); for(uint8_t __i=0;__i<255;__i++) { \
                    if (SPSR & (1<<SPIF)) { PORTC |= (1<<PC4); break; } }
*/

#define SPI_WAIT while(!(SPSR & (1<<SPIF)));

// ************************************************************************************************ //
//ISP Functions
// For ISP for ATMEGA168PA
// Initialize pins for spi communication
static void spi_init(void)

{
    DDR_ISP |= (1<<CSN) | (1<<CLK) | (1<<MOSI) | (1<<CE);
    // Enable MISO Pull-UP
    PORT_ISP |= (1<<MISO);

    // Do not enable Pull-UP for IRQ, this increase power consumption for 80uA.
    // NRF24L01 already has PULL-UP.
    //PORT_IRQ |= (1<<IRQ);

    // Since CSN is CS "NON", Disable Slave
    PORT_ISP |= (1<<CSN);

    /* Set the ISP Control Register
        Enable SPI, Master, and set speed to fclock/2
    */
    SPCR = (1<<SPE) | (1<<MSTR) | (0<<SPR0) | (0<<SPR1) | (1<<SPI2X);
}

static void spi_transfer_sync (uint8_t *dataout, uint8_t *datain, uint8_t len)
// Shift full array through target device
// This send the LSByte first, as required by NRF24L01
{
    uint8_t i;
    for (i = len; i > 0; i--) {
        // Load data in SPDR
        SPDR = dataout[i-1];

        SPI_WAIT;

        datain[i-1] = SPDR;
    }

}


static void spi_transmit_sync (uint8_t * dataout, uint8_t len)
// Shift full array to target device without receiving any byte
// This send the LSByte first, as required by NRF24L01
{
    uint8_t i;

    for (i = len; i > 0; i--) {
        SPDR = dataout[i-1];
        SPI_WAIT;
    }

}

static uint8_t spi_fast_shift (uint8_t data)
// Clocks only one byte to target device and returns the received one
{
    SPDR = data;
    SPI_WAIT;

    return SPDR;
}



// ***********************************      END of SPI Functions     ***************************************** //

// ***********************************      NRF24L01 Basic Functions ***************************************** //

uint8_t nrf_command(uint8_t reg)
// Send command to radio without argument; Used for FLUSH_TX, FLUSH_RX, NOP. Return status register.
{
    uint8_t status_reg;

    CSN_LOW  // ISP Slave on
    status_reg = spi_fast_shift(reg);
    CSN_HIGH // ISP Slave off

    return status_reg;
}

uint8_t nrf_read(uint8_t reg)
// Clocks only 1 byte from specfied register
{
    uint8_t r;
    CSN_LOW;  // ISP Slave on
    spi_fast_shift(reg); //R_REGISTER | (REGISTER_MASK & reg));
    r = spi_fast_shift(NOP);
    CSN_HIGH; // ISP Slave off
    return r;
}

void nrf_config_register(uint8_t reg, uint8_t value)
// Clocks only one byte into the given NRF24L01+ register
{
    CSN_LOW  // ISP Slave on
    spi_fast_shift(W_REGISTER | (REGISTER_MASK & reg));
    spi_fast_shift(value);
    CSN_HIGH // ISP Slave off
}

void nrf_read_register(uint8_t reg, uint8_t * value, uint8_t len)
// Reads an array of bytes from the given start position in the NRF24L01+ registers.
{
    CSN_LOW     // ISP Slave on
    spi_fast_shift(R_REGISTER | (REGISTER_MASK & reg));
    spi_transfer_sync(value,value,len);
    CSN_HIGH    // ISP Slave off
}

void nrf_write_register(uint8_t reg, uint8_t * value, uint8_t len)
// Writes an array of bytes into into NRF24L01+ registers.
{
    CSN_LOW
    spi_fast_shift(W_REGISTER | (REGISTER_MASK & reg));
    spi_transmit_sync(value,len);
    CSN_HIGH
}

uint8_t nrf_send_completed()
// Return 1 if MAX_RT or TX_DS flag triggers, otherwise return 0.
{
    uint8_t status;

    status = nrf_command(NOP);

    if ((status & (1<<MAX_RT)) || (status & (1<<TX_DS)))
    {
        nrf_config_register(STATUS, (1<<TX_DS));
        return 1;
    }
    else
    {
        return 0;
    }

}

uint8_t nrf_send(uint8_t * value, uint8_t len)
// Sends a data package to the default address. Be sure to send the correct
// amount of bytes as configured as payload on the receiver.
{

    TX_POWERUP                     // Power up NRF
    _delay_us(150);


    nrf_config_register(STATUS,(1<<TX_DS)|(1<<MAX_RT)|(1<<TX_FULL)); // clear status register, write 1 to clear bit.

    CSN_LOW                    // Pull down chip select
    spi_fast_shift( FLUSH_TX );     // Write cmd to flush tx fifo
    CSN_HIGH                    // Pull up chip select

    CSN_LOW                    // Pull down chip select
    spi_fast_shift( W_TX_PAYLOAD ); // Write cmd to write payload
    spi_transmit_sync(value, len);   // Write payload
    CSN_HIGH                    // Pull up chip select

    CE_HIGH                     // Start transmission
    _delay_us(15);

                                    // Wait until data is sent or MAX_RT flag
    while(!(nrf_send_completed())); // This function return 1 if data was transmitted or after MAX_RT.
    CE_LOW
    if (nrf_command(NOP) & (1<<MAX_RT)) return 0;
    return 1;
}
// ***********************************      END NRF24L01 Basic Functions     ***************************************** //


void nrf_init(void) {

    //cli();
    // IRQ support setup
    DDR_ISP |= (1<<CE);

    // setup INTx
    #ifdef IRQINT
      DDR_IRQ &= ~(1<<IRQ);
      //PORTD |= (1<<NRF_IRQ); // don't enable pull-up: internally enabled

      #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega88__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega48__) || defined(__AVR_ATmega328P__)
        #if (IRQINT==0)
          EICRA &= ~(1<<ISC00) ; // INT0, activated by falling edge
          EICRA |= (1<<ISC01);
          EIMSK |= (1<<INT0);   // Enable interrupt
        #elif (IRQINT==1)
          EICRA &= ~(1<<ISC10) ; // INT1, activated by falling edge
          EICRA |= (1<<ISC11);
          EIMSK |= (1<<INT1);   // Enable interrupt
        #else
          #warning Custom IRQ pin selected. No INT0/1 setup performed.
        #endif
      #elif defined(__AVR_ATmega162__)
        #if (IRQINT==0)
          MCUCR &= ~(1<<ISC00);
          MCUCR |= (1<<ISC01);
          GICR |= (1<<INT0);
        #elif (IRQINT==1)
          MCUCR &= ~(1<<ISC10);
          MCUCR |= (1<<ISC11);
          GICR |= (1<<INT1);
        #else
          #warning Custom IRQ pin selected. No INT0/1 setup performed.
        #endif
      #else
      #warning No supported IRQ port defined
      #endif
    #else
      #warning No hardware IRQ PIN defined
    #endif // IRQINT

    spi_init();
    //sei();

    // power-on sequence
    CE_HIGH;
    _delay_ms(10);

    CE_LOW;
    CSN_HIGH;
    _delay_ms(10);
}


void nrf_rx_config(uint8_t * config_buf,
                   uint8_t *rx0_addr,
                   uint8_t *rx1_addr,
                   uint8_t *tx_addr) {

    register uint8_t i;
    CE_LOW;

    //TODO: read addresses from EEPROM
    //eeprom_read_block()...

    // basic config (data size, auto-ack, rf)
    for(uint8_t i=0;i<6;i++) {
        nrf_config_register(i+1, config_buf[i]);
    }

    // rx/tx address
    nrf_write_register(RX_ADDR_P0, rx0_addr, ADDRESS_WIDTH);
    nrf_write_register(RX_ADDR_P1, rx1_addr, ADDRESS_WIDTH);
    for(i=0;i<4;i++) nrf_config_register(RX_ADDR_P2+i, rx1_addr[ADDRESS_WIDTH-1]+1+i);

    nrf_write_register(TX_ADDR, tx_addr, ADDRESS_WIDTH);

    // payload width (fixed)
    for (i=0;i<6;i++) {
        nrf_config_register(RX_PW_P0+i, PAYLOAD_WIDTH);
    }

    // disable FEATURE and DYNPD by default
    nrf_config_register(FEATURE, 0);
    nrf_config_register(DYNPD, 0);

    // reset IRQs
    nrf_command(FLUSH_TX);
    nrf_command(FLUSH_RX);
    nrf_config_register(STATUS, (1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT));

    // activate RX mode
    //RX_POWERUP;
    //_delay_us(150);
    CE_HIGH;
}

/*
void nrf_tx_config(uint8_t * addr) {

    CE_LOW;
    TX_POWERUP;

    // setup config in rx_config...

    // setup TX addr
    nrf_write_register(TX_ADDR, addr, ADDRESS_WIDTH);

    // reset IRQs
    nrf_command(FLUSH_TX);
    nrf_config_register(STATUS, (1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT));

    // activate TX mode
    CE_HIGH;
}
*/

uint8_t nrf_data_available(void)
// Returns payload pipe#+1 if data aveilable, 0 if no data in pipe
{
    uint8_t st, pl;

    CSN_LOW;
    st = spi_fast_shift(NOP);
    CSN_HIGH;

    pl = (st>>1) & 0x07; // pipe #
    // INT flag set or pipe not 'empty'
    if ((st & (1<<RX_DR)) || (pl != 0x07)) return (pl+1) & 0x07;
    else return 0;
}

uint8_t nrf_get_payload(uint8_t *buf, uint8_t len) {
    // returns 0 if no data for RX
    // returns 1..7 as (payload pipe+1) and filled buf on success
    uint8_t stat;
    uint8_t reg;
/*
    stat = nrf_command(NOP);
    if (!(stat & (1<<RX_DR))) return 0;

    reg = ((stat >> 1) & 0x07) + 1;
    if (reg >= 0x80) return 0; // fifo empty

    CE_LOW;
    */
    reg = ((nrf_command(NOP) >> 1) & 0x07) + 1;
    CSN_LOW                    // Pull down chip select
    spi_fast_shift( R_RX_PAYLOAD ); // Send Read Payload Command
    spi_transfer_sync(buf, buf, len);   // Read payload
    CSN_HIGH                    // Pull up chip select

    return reg;
}


/*

// ***********************************  Functions for Remote Configuration   ****************************************** //

// Configure the NRF as a receiver. In order to get configuration data from master

void remote_nrf_config(void)

{
	// NRF Default Configuration
	uint8_t default_NRF_ADDR[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
	uint8_t default_NRF_CH = 2;


	// Deactivate radio before changing configuration
	 CE_LOW

	 // Set radio to RX Mode
	RX_POWERUP

	// This reset the NRF IRQ , RX_DR, TX_DS, MAX_RT
	nrf_config_register(STATUS,(1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT));


	// Configure default radio channel
	nrf_config_register(RF_CH,default_NRF_CH);

	// Configure P0 to default values
		nrf_write_register(RX_ADDR_P0, default_NRF_ADDR, 5);	// Load address in NRF

		nrf_config_register(RX_PW_P0, CFG_PAYLOAD_WIDTH);		// Configure Payload Width for this Pipe


	// Activate PRX Mode
	CE_HIGH

	nrf_command(FLUSH_RX);		// Flush any data in RX register.

	// Enable Radio Interrupt
	EIMSK |= (1<<INT0);


}

void nrf_read_payload(void)
// Sends a data package to the default address. Be sure to send the correct
// amount of bytes as configured as payload on the receiver.
{

    CSN_LOW                    // Pull down chip select
    spi_fast_shift( R_RX_PAYLOAD ); // Send Read Payload Command
    spi_transfer_sync(buffer,buffer, PAYLOAD_WIDTH);   // Read payload
    CSN_HIGH                    // Pull up chip select

	// clear status register flag IRQ, write 1 to clear bit.
	nrf_config_register(STATUS,(1<<RX_DR) | (1<<TX_DS) | (1<<MAX_RT));


	// Verify the the Received Node Number before accepting the data.

	if (NODE_NUMBER == buffer[5]){
		DATA0 = buffer[0];
		DATA1 = buffer[1];
		DATA2 = buffer[2];
		DATA3 = buffer[3];
		RECV_MSGID = buffer[4];
	}
	// Else forward the message
	else {
		// short delay to avoid collitions
		_delay_loop_2(NODE_NUMBER * 100);

		// configure radio in TX
		nrf_tx_config();

		// send package
		nrf_send(buffer);

		// configure radio as rx again.
		nrf_rx_config();

	}

}



void nrf_read_cfg_payload(void)
// Sends a data package to the default address. Be sure to send the correct
// amount of bytes as configured as payload on the receiver.
{
		uint8_t data_input[CFG_PAYLOAD_WIDTH];
		uint8_t trash_out[CFG_PAYLOAD_WIDTH];


    CSN_LOW                    // Pull down chip select
    spi_fast_shift( R_RX_PAYLOAD ); // Send Read Payload Command
    spi_transfer_sync(trash_out,data_input, CFG_PAYLOAD_WIDTH);   // Read payload
    CSN_HIGH                    // Pull up chip select

	// save received data in eeprom
	remote_cfg_toEEPROM(data_input);

	// clear status register flag IRQ, write 1 to clear bit.
	nrf_config_register(STATUS,(1<<RX_DR) | (1<<TX_DS) | (1<<MAX_RT));
}


// Save the received configuration data in eeprom
void remote_cfg_toEEPROM(uint8_t * data_to_EE){


	// store received Configuration CH to EEPROM
	eeprom_update_byte(&CH,data_to_EE[6]);

	// store received Configuration Node Number to EEPROM
	eeprom_update_byte(&eeNODE_NUMBER,data_to_EE[5]);

	// store received configuration address to eeprom
	eeprom_update_byte(&eeNRF_ADDRESS[4],data_to_EE[4]);
	eeprom_update_byte(&eeNRF_ADDRESS[3],data_to_EE[3]);
	eeprom_update_byte(&eeNRF_ADDRESS[2],data_to_EE[2]);
	eeprom_update_byte(&eeNRF_ADDRESS[1],data_to_EE[1]);
	eeprom_update_byte(&eeNRF_ADDRESS[0],data_to_EE[0]);

}
 */
