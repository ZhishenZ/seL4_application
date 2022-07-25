
//TODO Work in Progress!!

//HElPER stuff

#include "bcm2837_i2c.h"
#include <stddef.h>


/* This variable allows us to test on hardware other than RPi.
// It prevents access to the kernel memory, and does not do any peripheral access
// Instead it prints out what it _would_ do if debug were 0
 */


/* I2C The time needed to transmit one byte. In microseconds.
 */
static int i2c_byte_wait_us = 0;

static volatile uint32_t* bcm2837_bsc = NULL;


void bcm2837_delayMicroseconds(uint64_t dummyseconds);

int bcm2837_i2c_begin(void* vaddr, void* gpio_vaddr)
{
//TODO Allow for both I2C to be used with outside Define!
    uint16_t cdiv = 0;
    volatile uint32_t* paddr = NULL;

    bcm2837_gpio_init(gpio_vaddr);
    bcm2837_bsc = (uint32_t*)(vaddr);
    
    /* Set the I2C/BSC pins to enable I2C access on them */
    bcm2837_gpio_fsel(SDAPIN, SDAMODE); /* SDA */
    bcm2837_gpio_fsel(SCLPIN, SCLMODE); /* SCL */ 
    paddr = bcm2837_bsc + BCM2837_BSC_DIV/4;
    /* Read the clock divider register */
    cdiv = bcm2837_peri_read(paddr);
    /* Calculate time for transmitting one byte
    // 1000000 = micros seconds in a second
    // 9 = Clocks per byte : 8 bits + ACK
    */
    i2c_byte_wait_us = ((float)cdiv / BCM2837_CORE_CLK_HZ) * 1000000 * 9;

    return 1;
}

void bcm2837_i2c_end(void)
{

//TODO allow generic define for GPIO with outside defines!
    bcm2837_gpio_fsel(SDAPIN, BCM2837_GPIO_FSEL_INPT); /* SDA */
    bcm2837_gpio_fsel(SCLPIN, BCM2837_GPIO_FSEL_INPT); /* SCL */

}


void bcm2837_i2c_setSlaveAddress(uint8_t addr)
{
    /* Set I2C Device Address */
    volatile uint32_t* paddr = bcm2837_bsc + BCM2837_BSC_A/4;
    bcm2837_peri_write(paddr, addr);
}

/* defaults to 0x5dc, should result in a 166.666 kHz I2C clock frequency.
// The divisor must be a power of 2. Odd numbers
// rounded down.
*/
void bcm2837_i2c_setClockDivider(uint16_t divider)
{
    volatile uint32_t* paddr = bcm2837_bsc + BCM2837_BSC_DIV/4;

    bcm2837_peri_write(paddr, divider);
    /* Calculate time for transmitting one byte
    // 1000000 = micros seconds in a second
    // 9 = Clocks per byte : 8 bits + ACK
    */
    i2c_byte_wait_us = ((float)divider / BCM2837_CORE_CLK_HZ) * 1000000 * 9;
}

/* set I2C clock divider by means of a baudrate number */
void bcm2837_i2c_set_baudrate(uint32_t baudrate)
{
	uint32_t divider;
	/* use 0xFFFE mask to limit a max value and round down any odd number */
	divider = (BCM2837_CORE_CLK_HZ / baudrate) & 0xFFFE;
	bcm2837_i2c_setClockDivider( (uint16_t)divider );
}

/* Writes an number of bytes to I2C */
int bcm2837_i2c_write(const char * buf, uint32_t len)
{
    volatile uint32_t* dlen    = bcm2837_bsc + BCM2837_BSC_DLEN/4;
    volatile uint32_t* fifo    = bcm2837_bsc + BCM2837_BSC_FIFO/4;
    volatile uint32_t* status  = bcm2837_bsc + BCM2837_BSC_S/4;
    volatile uint32_t* control = bcm2837_bsc + BCM2837_BSC_C/4;  

    uint32_t remaining = len;
    uint32_t i = 0;
//    int reason = BCM2837_I2C_REASON_OK;

    /* Clear FIFO */
    bcm2837_peri_set_bits(control, BCM2837_BSC_C_CLEAR_1 , BCM2837_BSC_C_CLEAR_1 );
    /* Clear Status */
    bcm2837_peri_write(status, BCM2837_BSC_S_CLKT | BCM2837_BSC_S_ERR | BCM2837_BSC_S_DONE);
    /* Set Data Length */
    bcm2837_peri_write(dlen, len);
    /* pre populate FIFO with max buffer */
    while( remaining && ( i < BCM2837_BSC_FIFO_SIZE ) )
    {
        bcm2837_peri_write_nb(fifo, buf[i]);
        i++;
        remaining--;
    }
    /* Enable device and start transfer */
    bcm2837_peri_write(control, BCM2837_BSC_C_I2CEN | BCM2837_BSC_C_ST);
    
    /* Transfer is over when BCM2837_BSC_S_DONE */
    while(!(bcm2837_peri_read(status) & BCM2837_BSC_S_DONE ))
    {
        while ( remaining && (bcm2837_peri_read(status) & BCM2837_BSC_S_TXD ))
    	{
	    /* Write to FIFO */
	    bcm2837_peri_write(fifo, buf[i]);
	    i++;
	    remaining--;
    	}
    }

   bcm2837_peri_set_bits(control, BCM2837_BSC_S_DONE , BCM2837_BSC_S_DONE);

    /* Received a NACK */
    if (bcm2837_peri_read(status) & BCM2837_BSC_S_ERR)
    {
	    return BCM2837_I2C_REASON_ERROR_NACK;
    }

    /* Received Clock Stretch Timeout */
    if (bcm2837_peri_read(status) & BCM2837_BSC_S_CLKT)
    {
	    return BCM2837_I2C_REASON_ERROR_CLKT;
    } 

    return (int) (len - remaining);
}

/* Read an number of bytes from I2C */
int bcm2837_i2c_read(char* buf, uint32_t len)
{
    volatile uint32_t* dlen    = bcm2837_bsc + BCM2837_BSC_DLEN/4;
    volatile uint32_t* fifo    = bcm2837_bsc + BCM2837_BSC_FIFO/4;
    volatile uint32_t* status  = bcm2837_bsc + BCM2837_BSC_S/4;
    volatile uint32_t* control = bcm2837_bsc + BCM2837_BSC_C/4;

    uint32_t remaining = len;
    uint32_t i = 0;
 //   uint8_t reason = BCM2837_I2C_REASON_OK;

    /* Clear FIFO */
    bcm2837_peri_set_bits(control, BCM2837_BSC_C_CLEAR_1 , BCM2837_BSC_C_CLEAR_1 );
    /* Clear Status */
    bcm2837_peri_write_nb(status, BCM2837_BSC_S_CLKT | BCM2837_BSC_S_ERR | BCM2837_BSC_S_DONE);
    /* Set Data Length */
    bcm2837_peri_write_nb(dlen, len);
    /* Start read */
    bcm2837_peri_write_nb(control, BCM2837_BSC_C_I2CEN | BCM2837_BSC_C_ST | BCM2837_BSC_C_READ);
    
    /* wait for transfer to complete */
    while (!(bcm2837_peri_read_nb(status) & BCM2837_BSC_S_DONE))
    {
        /* we must empty the FIFO as it is populated and not use any delay */
        while (remaining && bcm2837_peri_read_nb(status) & BCM2837_BSC_S_RXD)
    	{
	    /* Read from FIFO, no barrier */
	    buf[i] = bcm2837_peri_read_nb(fifo);
	    i++;
	    remaining--;
    	}
    }
    
    /* transfer has finished - grab any remaining stuff in FIFO */
    while (remaining && (bcm2837_peri_read_nb(status) & BCM2837_BSC_S_RXD))
    {
        /* Read from FIFO, no barrier */
        buf[i] = bcm2837_peri_read_nb(fifo);
        i++;
        remaining--;
    }
    
    bcm2837_peri_set_bits(status, BCM2837_BSC_S_DONE , BCM2837_BSC_S_DONE);
    /* Received a NACK */
    if (bcm2837_peri_read(status) & BCM2837_BSC_S_ERR)
    {
	    return BCM2837_I2C_REASON_ERROR_NACK;
    }

    /* Received Clock Stretch Timeout */
    if (bcm2837_peri_read(status) & BCM2837_BSC_S_CLKT)
    {
	    return BCM2837_I2C_REASON_ERROR_CLKT;
    }

    return (int) len - remaining;
}

/* Read an number of bytes from I2C sending a repeated start after writing
// the required register. Only works if your device supports this mode
*/
int bcm2837_i2c_read_register_rs(char* regaddr, char* buf, uint32_t len)
{   
    volatile uint32_t* dlen    = bcm2837_bsc + BCM2837_BSC_DLEN/4;
    volatile uint32_t* fifo    = bcm2837_bsc + BCM2837_BSC_FIFO/4;
    volatile uint32_t* status  = bcm2837_bsc + BCM2837_BSC_S/4;
    volatile uint32_t* control = bcm2837_bsc + BCM2837_BSC_C/4;
	uint32_t remaining = len;
    uint32_t i = 0;
    
    /* Clear FIFO */
    bcm2837_peri_set_bits(control, BCM2837_BSC_C_CLEAR_1 , BCM2837_BSC_C_CLEAR_1 );
    /* Clear Status */
    bcm2837_peri_write(status, BCM2837_BSC_S_CLKT | BCM2837_BSC_S_ERR | BCM2837_BSC_S_DONE);
    /* Set Data Length */
    bcm2837_peri_write(dlen, 1);
    /* Enable device and start transfer */
    bcm2837_peri_write(control, BCM2837_BSC_C_I2CEN);
    bcm2837_peri_write(fifo, regaddr[0]);
    bcm2837_peri_write(control, BCM2837_BSC_C_I2CEN | BCM2837_BSC_C_ST);
    
    /* poll for transfer has started */
    while ( !( bcm2837_peri_read(status) & BCM2837_BSC_S_TA ) )
    {
        /* Linux may cause us to miss entire transfer stage */
        if(bcm2837_peri_read(status) & BCM2837_BSC_S_DONE)
            break;
    }
    
    /* Send a repeated start with read bit set in address */
    bcm2837_peri_write(dlen, len);
    bcm2837_peri_write(control, BCM2837_BSC_C_I2CEN | BCM2837_BSC_C_ST  | BCM2837_BSC_C_READ );
    
    /* Wait for write to complete and first byte back. */
    bcm2837_delayMicroseconds(i2c_byte_wait_us * 3);
    
    /* wait for transfer to complete */
    while (!(bcm2837_peri_read(status) & BCM2837_BSC_S_DONE))
    {
        /* we must empty the FIFO as it is populated and not use any delay */
        while (remaining && bcm2837_peri_read(status) & BCM2837_BSC_S_RXD)
    	{
	    /* Read from FIFO */
	    buf[i] = bcm2837_peri_read(fifo);
	    i++;
	    remaining--;
    	}
    }

    /* transfer has finished - grab any remaining stuff in FIFO */
    while (remaining && (bcm2837_peri_read(status) & BCM2837_BSC_S_RXD))
    {
        /* Read from FIFO */
        buf[i] = bcm2837_peri_read(fifo);
        i++;
        remaining--;
    }
    bcm2837_peri_set_bits(control, BCM2837_BSC_S_DONE , BCM2837_BSC_S_DONE);
    
    /* Received a NACK */
    if (bcm2837_peri_read(status) & BCM2837_BSC_S_ERR)
    {
		return BCM2837_I2C_REASON_ERROR_NACK;
    }

    /* Received Clock Stretch Timeout */
    if (bcm2837_peri_read(status) & BCM2837_BSC_S_CLKT)
    {
	    return BCM2837_I2C_REASON_ERROR_CLKT;
    }


    return (int) len-remaining;
}

/* Sending an arbitrary number of bytes before issuing a repeated start 
// (with no prior stop) and reading a response. Some devices require this behavior.
*/
int bcm2837_i2c_write_read_rs(char* cmds, uint32_t cmds_len, char* buf, uint32_t buf_len)
{   
    volatile uint32_t* dlen    = bcm2837_bsc + BCM2837_BSC_DLEN/4;
    volatile uint32_t* fifo    = bcm2837_bsc + BCM2837_BSC_FIFO/4;
    volatile uint32_t* status  = bcm2837_bsc + BCM2837_BSC_S/4;
    volatile uint32_t* control = bcm2837_bsc + BCM2837_BSC_C/4;
    uint32_t remaining = cmds_len;
    uint32_t i = 0;
   
    /* Clear FIFO */
    bcm2837_peri_set_bits(control, BCM2837_BSC_C_CLEAR_1 , BCM2837_BSC_C_CLEAR_1 );

    /* Clear Status */
    bcm2837_peri_write(status, BCM2837_BSC_S_CLKT | BCM2837_BSC_S_ERR | BCM2837_BSC_S_DONE);

    /* Set Data Length */
    bcm2837_peri_write(dlen, cmds_len);
 
    /* pre populate FIFO with max buffer */
    while( remaining && ( i < BCM2837_BSC_FIFO_SIZE ) )
    {
        bcm2837_peri_write_nb(fifo, cmds[i]);
        i++;
        remaining--;
    }

    /* Enable device and start transfer */
    bcm2837_peri_write(control, BCM2837_BSC_C_I2CEN | BCM2837_BSC_C_ST);
    
    /* poll for transfer has started (way to do repeated start, from BCM2837 datasheet) */
    while ( !( bcm2837_peri_read(status) & BCM2837_BSC_S_TA ) )
    {
        /* Linux may cause us to miss entire transfer stage */
        if(bcm2837_peri_read_nb(status) & BCM2837_BSC_S_DONE)
            break;
    }
    
    remaining = buf_len;
    i = 0;

    /* Send a repeated start with read bit set in address */
    bcm2837_peri_write(dlen, buf_len);
    bcm2837_peri_write(control, BCM2837_BSC_C_I2CEN | BCM2837_BSC_C_ST  | BCM2837_BSC_C_READ );
    
    /* Wait for write to complete and first byte back. */
    bcm2837_delayMicroseconds(i2c_byte_wait_us * (cmds_len + 1));
    
    /* wait for transfer to complete */
    while (!(bcm2837_peri_read_nb(status) & BCM2837_BSC_S_DONE))
    {
        /* we must empty the FIFO as it is populated and not use any delay */
        while (remaining && bcm2837_peri_read(status) & BCM2837_BSC_S_RXD)
    	{
	    /* Read from FIFO, no barrier */
	    buf[i] = bcm2837_peri_read_nb(fifo);
	    i++;
	    remaining--;
    	}
    }
    
    /* transfer has finished - grab any remaining stuff in FIFO */
    while (remaining && (bcm2837_peri_read(status) & BCM2837_BSC_S_RXD))
    {
        /* Read from FIFO */
        buf[i] = bcm2837_peri_read(fifo);
        i++;
        remaining--;
    }
    bcm2837_peri_set_bits(control, BCM2837_BSC_S_DONE , BCM2837_BSC_S_DONE);
    /* Received a NACK */
    if (bcm2837_peri_read(status) & BCM2837_BSC_S_ERR)
    {
	    return BCM2837_I2C_REASON_ERROR_NACK;
    }

    /* Received Clock Stretch Timeout */
    if (bcm2837_peri_read(status) & BCM2837_BSC_S_CLKT)
    {
	    return BCM2837_I2C_REASON_ERROR_CLKT;
    }
 

    return (int) buf_len - remaining;
}

 void bcm2837_delayMicroseconds(uint64_t dummyseconds)
  {
      return;
      for(int i = 0; i < 10000; i++)
      {
        __asm__("nop");
      }

  }