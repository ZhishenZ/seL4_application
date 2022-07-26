/*
* HENDSOLDT Cyber GmbH
*/
#pragma once

#include <stdint.h>
#include "bcm2837_gpio.h"


#define RPi3
// I2C ID used by lib-platsupport must be set
#if !defined(I2C_CONFIG_ID)
#error "I2C_CONFIG_ID missing"
#endif

#if I2C_CONFIG_ID == 1
#define I2C0
#elif I2C_CONFIG_ID == 2
#define I2C1
#else
#error "I2C_CONFIG_ID not supported"
#endif 
/*! Base Address of the BSC0 registers */
#define BCM2837_BSC0_BASE       0x205000
/*! Base Address of the BSC1 registers */
#define BCM2837_BSC1_BASE       0x804000

/*! Speed of the core clock core_clk */
#define BCM2837_CORE_CLK_HZ		250000000	/*!< 250 MHz */

#ifdef RPi3
#ifdef I2C0
/*SDA*/
#define SDAPIN RPI_GPIO_P1_03
#define SDAMODE BCM2837_GPIO_FSEL_ALT0
#define SCLPIN RPI_GPIO_P1_05
#define SCLMODE BCM2837_GPIO_FSEL_ALT0
#define I2C_BASE_ADDRESS BCM2837_BSC0_BASE  
/*SDL*/
#else
#ifdef I2C1
#define SDAPIN RPI_V2_GPIO_P1_03
#define SDAMODE BCM2837_GPIO_FSEL_ALT0
#define SCLPIN RPI_V2_GPIO_P1_05
#define SCLMODE BCM2837_GPIO_FSEL_ALT0
#define I2C_BASE_ADDRESS BCM2837_BSC1_BASE
#endif
#endif
#endif

/* Defines for I2C
   GPIO register offsets from BCM2837_BSC*_BASE.
   Offsets into the BSC Peripheral block in bytes per 3.1 BSC Register Map
*/
#define BCM2837_BSC_C       0x0000 /*!< BSC Master Control */
#define BCM2837_BSC_S       0x0004 /*!< BSC Master Status */
#define BCM2837_BSC_DLEN    0x0008 /*!< BSC Master Data Length */
#define BCM2837_BSC_A       0x000c /*!< BSC Master Slave Address */
#define BCM2837_BSC_FIFO    0x0010 /*!< BSC Master Data FIFO */
#define BCM2837_BSC_DIV     0x0014 /*!< BSC Master Clock Divider */
#define BCM2837_BSC_DEL     0x0018 /*!< BSC Master Data Delay */
#define BCM2837_BSC_CLKT    0x001c /*!< BSC Master Clock Stretch Timeout */

/* Register masks for BSC_C */
#define BCM2837_BSC_C_I2CEN   0x00008000 /*!< I2C Enable, 0 = disabled, 1 = enabled */
#define BCM2837_BSC_C_INTR    0x00000400 /*!< Interrupt on RX */
#define BCM2837_BSC_C_INTT    0x00000200 /*!< Interrupt on TX */
#define BCM2837_BSC_C_INTD    0x00000100 /*!< Interrupt on DONE */
#define BCM2837_BSC_C_ST      0x00000080 /*!< Start transfer, 1 = Start a new transfer */
#define BCM2837_BSC_C_CLEAR_1 0x00000020 /*!< Clear FIFO Clear */
#define BCM2837_BSC_C_CLEAR_2 0x00000010 /*!< Clear FIFO Clear */
#define BCM2837_BSC_C_READ    0x00000001 /*!< Read transfer */

/* Register masks for BSC_S */
#define BCM2837_BSC_S_CLKT    0x00000200 /*!< Clock stretch timeout */
#define BCM2837_BSC_S_ERR     0x00000100 /*!< ACK error */
#define BCM2837_BSC_S_RXF     0x00000080 /*!< RXF FIFO full, 0 = FIFO is not full, 1 = FIFO is full */
#define BCM2837_BSC_S_TXE     0x00000040 /*!< TXE FIFO full, 0 = FIFO is not full, 1 = FIFO is full */
#define BCM2837_BSC_S_RXD     0x00000020 /*!< RXD FIFO contains data */
#define BCM2837_BSC_S_TXD     0x00000010 /*!< TXD FIFO can accept data */
#define BCM2837_BSC_S_RXR     0x00000008 /*!< RXR FIFO needs reading (full) */
#define BCM2837_BSC_S_TXW     0x00000004 /*!< TXW FIFO needs writing (full) */
#define BCM2837_BSC_S_DONE    0x00000002 /*!< Transfer DONE */
#define BCM2837_BSC_S_TA      0x00000001 /*!< Transfer Active */

#define BCM2837_BSC_FIFO_SIZE    16 /*!< BSC FIFO size */

/*! \brief bcm2837I2CClockDivider
  Specifies the divider used to generate the I2C clock from the system clock.
  Clock divided is based on nominal base clock rate of 250MHz
*/
typedef enum
{
    BCM2837_I2C_CLOCK_DIVIDER_2500   = 2500,      /*!< 2500 = 10us = 100 kHz */
    BCM2837_I2C_CLOCK_DIVIDER_626    = 626,       /*!< 622 = 2.504us = 399.3610 kHz */
    BCM2837_I2C_CLOCK_DIVIDER_150    = 150,       /*!< 150 = 60ns = 1.666 MHz (default at reset) */
    BCM2837_I2C_CLOCK_DIVIDER_148    = 148        /*!< 148 = 59ns = 1.689 MHz */
} bcm2837I2CClockDivider;

/*! \brief bcm2837I2CReasonCodes
  Specifies the reason codes for the bcm2837_i2c_write and bcm2837_i2c_read functions.
*/
typedef enum
{
    BCM2837_I2C_REASON_OK           = 0,      /*!< Success */
    BCM2837_I2C_REASON_ERROR_NACK    = -1,      /*!< Received a NACK */
    BCM2837_I2C_REASON_ERROR_CLKT    = -2      /*!< Received Clock Stretch Timeout */
} bcm2837I2CReasonCodes;

    /*! \defgroup i2c I2C access
      These functions let you use I2C (The Broadcom Serial Control bus with the Philips
      I2C bus/interface version 2.1 January 2000.) to interface with an external I2C device.
      @{
    */

    /*! Start I2C operations.
      Forces RPi I2C pins P1-03 (SDA) and P1-05 (SCL)
      to alternate function ALT0, which enables those pins for I2C interface.
      You should call bcm2837_i2c_end() when all I2C functions are complete to return the pins to
      their default functions
      \return 1 if successful, 0 otherwise (perhaps because you are not running as root)
      \sa  bcm2837_i2c_end()
    */
    extern int bcm2837_i2c_begin(void* vaddr, void* gpio_vaddr);

    /*! End I2C operations.
      I2C pins P1-03 (SDA) and P1-05 (SCL)
      are returned to their default INPUT behaviour.
    */
    extern void bcm2837_i2c_end(void);

    /*! Sets the I2C slave address.
      \param[in] addr The I2C slave address.
    */
    extern void bcm2837_i2c_setSlaveAddress(uint8_t addr);

    /*! Sets the I2C clock divider and therefore the I2C clock speed.
      \param[in] divider The desired I2C clock divider, one of BCM2837_I2C_CLOCK_DIVIDER_*,
      see \ref bcm2837I2CClockDivider
    */
    extern void bcm2837_i2c_setClockDivider(uint16_t divider);

    /*! Sets the I2C clock divider by converting the baudrate parameter to
      the equivalent I2C clock divider. ( see \sa bcm2837_i2c_setClockDivider)
      For the I2C standard 100khz you would set baudrate to 100000
      The use of baudrate corresponds to its use in the I2C kernel device
      driver. (Of course, bcm2837 has nothing to do with the kernel driver)
    */
    extern void bcm2837_i2c_set_baudrate(uint32_t baudrate);

    /*! Transfers any number of bytes to the currently selected I2C slave.
      (as previously set by \sa bcm2837_i2c_setSlaveAddress)
      \param[in] buf Buffer of bytes to send.
      \param[in] len Number of bytes in the buf buffer, and the number of bytes to send.
      \return reason see \ref bcm2837I2CReasonCodes
    */
    extern int bcm2837_i2c_write(const char * buf, uint32_t len);

    /*! Transfers any number of bytes from the currently selected I2C slave.
      (as previously set by \sa bcm2837_i2c_setSlaveAddress)
      \param[in] buf Buffer of bytes to receive.
      \param[in] len Number of bytes in the buf buffer, and the number of bytes to received.
      \return reason see \ref bcm2837I2CReasonCodes
    */
    extern int bcm2837_i2c_read(char* buf, uint32_t len);

    /*! Allows reading from I2C slaves that require a repeated start (without any prior stop)
      to read after the required slave register has been set. For example, the popular
      MPL3115A2 pressure and temperature sensor. Note that your device must support or
      require this mode. If your device does not require this mode then the standard
      combined:
      \sa bcm2837_i2c_write
      \sa bcm2837_i2c_read
      are a better choice.
      Will read from the slave previously set by \sa bcm2837_i2c_setSlaveAddress
      \param[in] regaddr Buffer containing the slave register you wish to read from.
      \param[in] buf Buffer of bytes to receive.
      \param[in] len Number of bytes in the buf buffer, and the number of bytes to received.
      \return reason see \ref bcm2837I2CReasonCodes
    */
    extern int bcm2837_i2c_read_register_rs(char* regaddr, char* buf,
                                                uint32_t len);

    /*! Allows sending an arbitrary number of bytes to I2C slaves before issuing a repeated
      start (with no prior stop) and reading a response.
      Necessary for devices that require such behavior, such as the MLX90620.
      Will write to and read from the slave previously set by \sa bcm2837_i2c_setSlaveAddress
      \param[in] cmds Buffer containing the bytes to send before the repeated start condition.
      \param[in] cmds_len Number of bytes to send from cmds buffer
      \param[in] buf Buffer of bytes to receive.
      \param[in] buf_len Number of bytes to receive in the buf buffer.
      \return reason see \ref bcm2837I2CReasonCodes
    */
    extern int bcm2837_i2c_write_read_rs(char* cmds, uint32_t cmds_len,
                                             char* buf, uint32_t buf_len);

    /*! @} */
