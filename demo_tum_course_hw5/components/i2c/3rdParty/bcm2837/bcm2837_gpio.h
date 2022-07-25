/*
 * Copyright (C) 2011-2013 Mike McCauley
 * Copyright (C) 2020-2021, HENSOLDT Cyber GmbH
 * SPDX-License-Identifier: GPL-3.0-only
 */

#pragma once

#include <stdint.h>

/*! Base Address of the System Timer registers */
#define BCM2837_ST_BASE                 0x3000
/*! Base Address of the Pads registers */
#define BCM2837_GPIO_PADS               0x100000
/*! Base Address of the Clock/timer registers */
#define BCM2837_CLOCK_BASE              0x101000
/*! Base Address of the GPIO registers */
#define BCM2837_GPIO_BASE               0x200000
/*! Base Address of the SPI0 registers */
#define BCM2837_SPI0_BASE               0x204000
/*! Base Address of the BSC0 registers */
#define BCM2837_BSC0_BASE               0x205000
/*! Base Address of the PWM registers */
#define BCM2837_GPIO_PWM                0x20C000
/*! Base Address of the AUX registers */
#define BCM2837_AUX_BASE                0x215000
/*! Base Address of the AUX_SPI1 registers */
#define BCM2837_SPI1_BASE               0x215080
/*! Base Address of the AUX_SPI2 registers */
#define BCM2837_SPI2_BASE               0x2150C0
/*! Base Address of the BSC1 registers */
#define BCM2837_BSC1_BASE               0x804000

enum gpio_port
{
    GPIO_NBANKS
};

/*! \brief bcm2837RegisterBase
  Register bases for bcm2837_regbase()
*/
typedef enum
{
    BCM2837_REGBASE_ST   = 1, /*!< Base of the ST (System Timer) registers. */
    BCM2837_REGBASE_GPIO = 2, /*!< Base of the GPIO registers. */
    BCM2837_REGBASE_PWM  = 3, /*!< Base of the PWM registers. */
    BCM2837_REGBASE_CLK  = 4, /*!< Base of the CLK registers. */
    BCM2837_REGBASE_PADS = 5, /*!< Base of the PADS registers. */
    BCM2837_REGBASE_SPI0 = 6, /*!< Base of the SPI0 registers. */
    BCM2837_REGBASE_BSC0 = 7, /*!< Base of the BSC0 registers. */
    BCM2837_REGBASE_BSC1 = 8,  /*!< Base of the BSC1 registers. */
    BCM2837_REGBASE_AUX  = 9,  /*!< Base of the AUX registers. */
    BCM2837_REGBASE_SPI1 = 10  /*!< Base of the SPI1 registers. */
} bcm2837RegisterBase;

/* Defines for GPIO
   The BCM2837 has 54 GPIO pins.
   BCM2837 data sheet, Page 90 onwards.
*/
/*! GPIO register offsets from BCM2837_GPIO_BASE.
  Offsets into the GPIO Peripheral block in bytes per 6.1 Register View
*/
#define BCM2837_GPFSEL0                      0x0000 /*!< GPIO Function Select 0 */
#define BCM2837_GPFSEL1                      0x0004 /*!< GPIO Function Select 1 */
#define BCM2837_GPFSEL2                      0x0008 /*!< GPIO Function Select 2 */
#define BCM2837_GPFSEL3                      0x000c /*!< GPIO Function Select 3 */
#define BCM2837_GPFSEL4                      0x0010 /*!< GPIO Function Select 4 */
#define BCM2837_GPFSEL5                      0x0014 /*!< GPIO Function Select 5 */
#define BCM2837_GPSET0                       0x001c /*!< GPIO Pin Output Set 0 */
#define BCM2837_GPSET1                       0x0020 /*!< GPIO Pin Output Set 1 */
#define BCM2837_GPCLR0                       0x0028 /*!< GPIO Pin Output Clear 0 */
#define BCM2837_GPCLR1                       0x002c /*!< GPIO Pin Output Clear 1 */
#define BCM2837_GPLEV0                       0x0034 /*!< GPIO Pin Level 0 */
#define BCM2837_GPLEV1                       0x0038 /*!< GPIO Pin Level 1 */
#define BCM2837_GPEDS0                       0x0040 /*!< GPIO Pin Event Detect Status 0 */
#define BCM2837_GPEDS1                       0x0044 /*!< GPIO Pin Event Detect Status 1 */
#define BCM2837_GPREN0                       0x004c /*!< GPIO Pin Rising Edge Detect Enable 0 */
#define BCM2837_GPREN1                       0x0050 /*!< GPIO Pin Rising Edge Detect Enable 1 */
#define BCM2837_GPFEN0                       0x0058 /*!< GPIO Pin Falling Edge Detect Enable 0 */
#define BCM2837_GPFEN1                       0x005c /*!< GPIO Pin Falling Edge Detect Enable 1 */
#define BCM2837_GPHEN0                       0x0064 /*!< GPIO Pin High Detect Enable 0 */
#define BCM2837_GPHEN1                       0x0068 /*!< GPIO Pin High Detect Enable 1 */
#define BCM2837_GPLEN0                       0x0070 /*!< GPIO Pin Low Detect Enable 0 */
#define BCM2837_GPLEN1                       0x0074 /*!< GPIO Pin Low Detect Enable 1 */
#define BCM2837_GPAREN0                      0x007c /*!< GPIO Pin Async. Rising Edge Detect 0 */
#define BCM2837_GPAREN1                      0x0080 /*!< GPIO Pin Async. Rising Edge Detect 1 */
#define BCM2837_GPAFEN0                      0x0088 /*!< GPIO Pin Async. Falling Edge Detect 0 */
#define BCM2837_GPAFEN1                      0x008c /*!< GPIO Pin Async. Falling Edge Detect 1 */
#define BCM2837_GPPUD                        0x0094 /*!< GPIO Pin Pull-up/down Enable */
#define BCM2837_GPPUDCLK0                    0x0098 /*!< GPIO Pin Pull-up/down Enable Clock 0 */
#define BCM2837_GPPUDCLK1                    0x009c /*!< GPIO Pin Pull-up/down Enable Clock 1 */

/*!   \brief bcm2837PortFunction
  Port function select modes for bcm2837_gpio_fsel()
*/
typedef enum
{
    BCM2837_GPIO_FSEL_INPT  = 0x00,   /*!< Input 0b000 */
    BCM2837_GPIO_FSEL_OUTP  = 0x01,   /*!< Output 0b001 */
    BCM2837_GPIO_FSEL_ALT0  = 0x04,   /*!< Alternate function 0 0b100 */
    BCM2837_GPIO_FSEL_ALT1  = 0x05,   /*!< Alternate function 1 0b101 */
    BCM2837_GPIO_FSEL_ALT2  = 0x06,   /*!< Alternate function 2 0b110, */
    BCM2837_GPIO_FSEL_ALT3  = 0x07,   /*!< Alternate function 3 0b111 */
    BCM2837_GPIO_FSEL_ALT4  = 0x03,   /*!< Alternate function 4 0b011 */
    BCM2837_GPIO_FSEL_ALT5  = 0x02,   /*!< Alternate function 5 0b010 */
    BCM2837_GPIO_FSEL_MASK  = 0x07    /*!< Function select bits mask 0b111 */
} bcm2837FunctionSelect;

/*! \brief bcm2837PUDControl
  Pullup/Pulldown defines for bcm2837_gpio_pud()
*/
typedef enum
{
    BCM2837_GPIO_PUD_OFF     = 0x00,   /*!< Off ? disable pull-up/down 0b00 */
    BCM2837_GPIO_PUD_DOWN    = 0x01,   /*!< Enable Pull Down control 0b01 */
    BCM2837_GPIO_PUD_UP      = 0x02    /*!< Enable Pull Up control 0b10  */
} bcm2837PUDControl;

/*! Pad control register offsets from BCM2837_GPIO_PADS */
#define BCM2837_PADS_GPIO_0_27               0x002c /*!< Pad control register for pads 0 to 27 */
#define BCM2837_PADS_GPIO_28_45              0x0030 /*!< Pad control register for pads 28 to 45 */
#define BCM2837_PADS_GPIO_46_53              0x0034 /*!< Pad control register for pads 46 to 53 */

/*! Pad Control masks */
#define BCM2837_PAD_PASSWRD                  (0x5A << 24)  /*!< Password to enable setting pad mask */
#define BCM2837_PAD_SLEW_RATE_UNLIMITED      0x10 /*!< Slew rate unlimited */
#define BCM2837_PAD_HYSTERESIS_ENABLED       0x08 /*!< Hysteresis enabled */
#define BCM2837_PAD_DRIVE_2mA                0x00 /*!< 2mA drive current */
#define BCM2837_PAD_DRIVE_4mA                0x01 /*!< 4mA drive current */
#define BCM2837_PAD_DRIVE_6mA                0x02 /*!< 6mA drive current */
#define BCM2837_PAD_DRIVE_8mA                0x03 /*!< 8mA drive current */
#define BCM2837_PAD_DRIVE_10mA               0x04 /*!< 10mA drive current */
#define BCM2837_PAD_DRIVE_12mA               0x05 /*!< 12mA drive current */
#define BCM2837_PAD_DRIVE_14mA               0x06 /*!< 14mA drive current */
#define BCM2837_PAD_DRIVE_16mA               0x07 /*!< 16mA drive current */

/*! \brief bcm2837PadGroup
  Pad group specification for bcm2837_gpio_pad()
*/
typedef enum
{
    BCM2837_PAD_GROUP_GPIO_0_27         = 0, /*!< Pad group for GPIO pads 0 to 27 */
    BCM2837_PAD_GROUP_GPIO_28_45        = 1, /*!< Pad group for GPIO pads 28 to 45 */
    BCM2837_PAD_GROUP_GPIO_46_53        = 2  /*!< Pad group for GPIO pads 46 to 53 */
} bcm2837PadGroup;

/*! \brief GPIO Pin Numbers

  Here we define Raspberry Pin GPIO pins on P1 in terms of the underlying BCM GPIO pin numbers.
  These can be passed as a pin number to any function requiring a pin.
  Not all pins on the RPi 26 bin IDE plug are connected to GPIO pins
  and some can adopt an alternate function.
  RPi version 2 has some slightly different pinouts, and these are values RPI_V2_*.
  RPi B+ has yet differnet pinouts and these are defined in RPI_BPLUS_*.
  At bootup, pins 8 and 10 are set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively
  When SPI0 is in use (ie after bcm2837_spi_begin()), SPI0 pins are dedicated to SPI
  and cant be controlled independently.
  If you are using the RPi Compute Module, just use the GPIO number: there is no need to use one of these
  symbolic names
*/
typedef enum
{
    RPI_GPIO_P1_03        =  0,  /*!< Version 1, Pin P1-03 */
    RPI_GPIO_P1_05        =  1,  /*!< Version 1, Pin P1-05 */
    RPI_GPIO_P1_07        =  4,  /*!< Version 1, Pin P1-07 */
    RPI_GPIO_P1_08        = 14,  /*!< Version 1, Pin P1-08, defaults to alt function 0 UART0_TXD */
    RPI_GPIO_P1_10        = 15,  /*!< Version 1, Pin P1-10, defaults to alt function 0 UART0_RXD */
    RPI_GPIO_P1_11        = 17,  /*!< Version 1, Pin P1-11 */
    RPI_GPIO_P1_12        = 18,  /*!< Version 1, Pin P1-12, can be PWM channel 0 in ALT FUN 5 */
    RPI_GPIO_P1_13        = 21,  /*!< Version 1, Pin P1-13 */
    RPI_GPIO_P1_15        = 22,  /*!< Version 1, Pin P1-15 */
    RPI_GPIO_P1_16        = 23,  /*!< Version 1, Pin P1-16 */
    RPI_GPIO_P1_18        = 24,  /*!< Version 1, Pin P1-18 */
    RPI_GPIO_P1_19        = 10,  /*!< Version 1, Pin P1-19, MOSI when SPI0 in use */
    RPI_GPIO_P1_21        =  9,  /*!< Version 1, Pin P1-21, MISO when SPI0 in use */
    RPI_GPIO_P1_22        = 25,  /*!< Version 1, Pin P1-22 */
    RPI_GPIO_P1_23        = 11,  /*!< Version 1, Pin P1-23, CLK when SPI0 in use */
    RPI_GPIO_P1_24        =  8,  /*!< Version 1, Pin P1-24, CE0 when SPI0 in use */
    RPI_GPIO_P1_26        =  7,  /*!< Version 1, Pin P1-26, CE1 when SPI0 in use */

    /* RPi Version 2 */
    RPI_V2_GPIO_P1_03     =  2,  /*!< Version 2, Pin P1-03 */
    RPI_V2_GPIO_P1_05     =  3,  /*!< Version 2, Pin P1-05 */
    RPI_V2_GPIO_P1_07     =  4,  /*!< Version 2, Pin P1-07 */
    RPI_V2_GPIO_P1_08     = 14,  /*!< Version 2, Pin P1-08, defaults to alt function 0 UART0_TXD */
    RPI_V2_GPIO_P1_10     = 15,  /*!< Version 2, Pin P1-10, defaults to alt function 0 UART0_RXD */
    RPI_V2_GPIO_P1_11     = 17,  /*!< Version 2, Pin P1-11 */
    RPI_V2_GPIO_P1_12     = 18,  /*!< Version 2, Pin P1-12, can be PWM channel 0 in ALT FUN 5 */
    RPI_V2_GPIO_P1_13     = 27,  /*!< Version 2, Pin P1-13 */
    RPI_V2_GPIO_P1_15     = 22,  /*!< Version 2, Pin P1-15 */
    RPI_V2_GPIO_P1_16     = 23,  /*!< Version 2, Pin P1-16 */
    RPI_V2_GPIO_P1_18     = 24,  /*!< Version 2, Pin P1-18 */
    RPI_V2_GPIO_P1_19     = 10,  /*!< Version 2, Pin P1-19, MOSI when SPI0 in use */
    RPI_V2_GPIO_P1_21     =  9,  /*!< Version 2, Pin P1-21, MISO when SPI0 in use */
    RPI_V2_GPIO_P1_22     = 25,  /*!< Version 2, Pin P1-22 */
    RPI_V2_GPIO_P1_23     = 11,  /*!< Version 2, Pin P1-23, CLK when SPI0 in use */
    RPI_V2_GPIO_P1_24     =  8,  /*!< Version 2, Pin P1-24, CE0 when SPI0 in use */
    RPI_V2_GPIO_P1_26     =  7,  /*!< Version 2, Pin P1-26, CE1 when SPI0 in use */
    RPI_V2_GPIO_P1_29     =  5,  /*!< Version 2, Pin P1-29 */
    RPI_V2_GPIO_P1_31     =  6,  /*!< Version 2, Pin P1-31 */
    RPI_V2_GPIO_P1_32     = 12,  /*!< Version 2, Pin P1-32 */
    RPI_V2_GPIO_P1_33     = 13,  /*!< Version 2, Pin P1-33 */
    RPI_V2_GPIO_P1_35     = 19,  /*!< Version 2, Pin P1-35, can be PWM channel 1 in ALT FUN 5  */
    RPI_V2_GPIO_P1_36     = 16,  /*!< Version 2, Pin P1-36 */
    RPI_V2_GPIO_P1_37     = 26,  /*!< Version 2, Pin P1-37 */
    RPI_V2_GPIO_P1_38     = 20,  /*!< Version 2, Pin P1-38 */
    RPI_V2_GPIO_P1_40     = 21,  /*!< Version 2, Pin P1-40 */

    /* RPi Version 2, new plug P5 */
    RPI_V2_GPIO_P5_03     = 28,  /*!< Version 2, Pin P5-03 */
    RPI_V2_GPIO_P5_04     = 29,  /*!< Version 2, Pin P5-04 */
    RPI_V2_GPIO_P5_05     = 30,  /*!< Version 2, Pin P5-05 */
    RPI_V2_GPIO_P5_06     = 31,  /*!< Version 2, Pin P5-06 */

    /* RPi B+ J8 header, also RPi 2 40 pin GPIO header */
    RPI_BPLUS_GPIO_J8_03     =  2,  /*!< B+, Pin J8-03 */
    RPI_BPLUS_GPIO_J8_05     =  3,  /*!< B+, Pin J8-05 */
    RPI_BPLUS_GPIO_J8_07     =  4,  /*!< B+, Pin J8-07 */
    RPI_BPLUS_GPIO_J8_08     = 14,  /*!< B+, Pin J8-08, defaults to alt function 0 UART0_TXD */
    RPI_BPLUS_GPIO_J8_10     = 15,  /*!< B+, Pin J8-10, defaults to alt function 0 UART0_RXD */
    RPI_BPLUS_GPIO_J8_11     = 17,  /*!< B+, Pin J8-11 */
    RPI_BPLUS_GPIO_J8_12     = 18,  /*!< B+, Pin J8-12, can be PWM channel 0 in ALT FUN 5 */
    RPI_BPLUS_GPIO_J8_13     = 27,  /*!< B+, Pin J8-13 */
    RPI_BPLUS_GPIO_J8_15     = 22,  /*!< B+, Pin J8-15 */
    RPI_BPLUS_GPIO_J8_16     = 23,  /*!< B+, Pin J8-16 */
    RPI_BPLUS_GPIO_J8_18     = 24,  /*!< B+, Pin J8-18 */
    RPI_BPLUS_GPIO_J8_19     = 10,  /*!< B+, Pin J8-19, MOSI when SPI0 in use */
    RPI_BPLUS_GPIO_J8_21     =  9,  /*!< B+, Pin J8-21, MISO when SPI0 in use */
    RPI_BPLUS_GPIO_J8_22     = 25,  /*!< B+, Pin J8-22 */
    RPI_BPLUS_GPIO_J8_23     = 11,  /*!< B+, Pin J8-23, CLK when SPI0 in use */
    RPI_BPLUS_GPIO_J8_24     =  8,  /*!< B+, Pin J8-24, CE0 when SPI0 in use */
    RPI_BPLUS_GPIO_J8_26     =  7,  /*!< B+, Pin J8-26, CE1 when SPI0 in use */
    RPI_BPLUS_GPIO_J8_29     =  5,  /*!< B+, Pin J8-29,  */
    RPI_BPLUS_GPIO_J8_31     =  6,  /*!< B+, Pin J8-31,  */
    RPI_BPLUS_GPIO_J8_32     = 12,  /*!< B+, Pin J8-32,  */
    RPI_BPLUS_GPIO_J8_33     = 13,  /*!< B+, Pin J8-33,  */
    RPI_BPLUS_GPIO_J8_35     = 19,  /*!< B+, Pin J8-35, can be PWM channel 1 in ALT FUN 5 */
    RPI_BPLUS_GPIO_J8_36     = 16,  /*!< B+, Pin J8-36,  */
    RPI_BPLUS_GPIO_J8_37     = 26,  /*!< B+, Pin J8-37,  */
    RPI_BPLUS_GPIO_J8_38     = 20,  /*!< B+, Pin J8-38,  */
    RPI_BPLUS_GPIO_J8_40     = 21   /*!< B+, Pin J8-40,  */
} RPiGPIOPin;

/* Defines for AUX
  GPIO register offsets from BCM2837_AUX_BASE.
*/
#define BCM2837_AUX_IRQ         0x0000  /*!< xxx */
#define BCM2837_AUX_ENABLE      0x0004  /*!< */

#define BCM2837_AUX_ENABLE_UART1    0x01    /*!<  */
#define BCM2837_AUX_ENABLE_SPI0     0x02    /*!< SPI0 (SPI1 in the device) */
#define BCM2837_AUX_ENABLE_SPI1     0x04    /*!< SPI1 (SPI2 in the device) */


#define BCM2837_AUX_SPI_CNTL0       0x0000  /*!< */
#define BCM2837_AUX_SPI_CNTL1       0x0004  /*!< */
#define BCM2837_AUX_SPI_STAT        0x0008  /*!< */
#define BCM2837_AUX_SPI_PEEK        0x000C  /*!< Read but do not take from FF */
#define BCM2837_AUX_SPI_IO      0x0020  /*!< Write = TX, read=RX */
#define BCM2837_AUX_SPI_TXHOLD      0x0030  /*!< Write = TX keep CS, read=RX */

#define BCM2837_AUX_SPI_CLOCK_MIN   30500       /*!< 30,5kHz */
#define BCM2837_AUX_SPI_CLOCK_MAX   125000000   /*!< 125Mhz */

#define BCM2837_AUX_SPI_CNTL0_SPEED 0xFFF00000  /*!< */
#define BCM2837_AUX_SPI_CNTL0_SPEED_MAX 0xFFF      /*!< */
#define BCM2837_AUX_SPI_CNTL0_SPEED_SHIFT 20        /*!< */

#define BCM2837_AUX_SPI_CNTL0_CS0_N     0x000C0000 /*!< CS 0 low */
#define BCM2837_AUX_SPI_CNTL0_CS1_N     0x000A0000 /*!< CS 1 low */
#define BCM2837_AUX_SPI_CNTL0_CS2_N     0x00060000 /*!< CS 2 low */

#define BCM2837_AUX_SPI_CNTL0_POSTINPUT 0x00010000  /*!< */
#define BCM2837_AUX_SPI_CNTL0_VAR_CS    0x00008000  /*!< */
#define BCM2837_AUX_SPI_CNTL0_VAR_WIDTH 0x00004000  /*!< */
#define BCM2837_AUX_SPI_CNTL0_DOUTHOLD  0x00003000  /*!< */
#define BCM2837_AUX_SPI_CNTL0_ENABLE    0x00000800  /*!< */
#define BCM2837_AUX_SPI_CNTL0_CPHA_IN   0x00000400  /*!< */
#define BCM2837_AUX_SPI_CNTL0_CLEARFIFO 0x00000200  /*!< */
#define BCM2837_AUX_SPI_CNTL0_CPHA_OUT  0x00000100  /*!< */
#define BCM2837_AUX_SPI_CNTL0_CPOL  0x00000080  /*!< */
#define BCM2837_AUX_SPI_CNTL0_MSBF_OUT  0x00000040  /*!< */
#define BCM2837_AUX_SPI_CNTL0_SHIFTLEN  0x0000003F  /*!< */

#define BCM2837_AUX_SPI_CNTL1_CSHIGH    0x00000700  /*!< */
#define BCM2837_AUX_SPI_CNTL1_IDLE  0x00000080  /*!< */
#define BCM2837_AUX_SPI_CNTL1_TXEMPTY   0x00000040  /*!< */
#define BCM2837_AUX_SPI_CNTL1_MSBF_IN   0x00000002  /*!< */
#define BCM2837_AUX_SPI_CNTL1_KEEP_IN   0x00000001  /*!< */

#define BCM2837_AUX_SPI_STAT_TX_LVL 0xFF000000  /*!< */
#define BCM2837_AUX_SPI_STAT_RX_LVL 0x00FF0000  /*!< */
#define BCM2837_AUX_SPI_STAT_TX_FULL    0x00000400  /*!< */
#define BCM2837_AUX_SPI_STAT_TX_EMPTY   0x00000200  /*!< */
#define BCM2837_AUX_SPI_STAT_RX_FULL    0x00000100  /*!< */
#define BCM2837_AUX_SPI_STAT_RX_EMPTY   0x00000080  /*!< */
#define BCM2837_AUX_SPI_STAT_BUSY   0x00000040  /*!< */
#define BCM2837_AUX_SPI_STAT_BITCOUNT   0x0000003F  /*!< */


extern void bcm2837_gpio_init(void* vaddr);

/*! Reads 32 bit value from a peripheral address WITH a memory barrier before and after each read.
  This is safe, but slow.  The MB before protects this read from any in-flight reads that didn't
  use a MB.  The MB after protects subsequent reads from another peripheral.

  \param[in] paddr Physical address to read from. See BCM2837_GPIO_BASE etc.
  \return the value read from the 32 bit register
  \sa Physical Addresses
*/
extern uint32_t bcm2837_peri_read(volatile uint32_t* paddr);

/*! Reads 32 bit value from a peripheral address WITHOUT the read barriers
  You should only use this when:
  o your code has previously called bcm2837_peri_read() for a register
  within the same peripheral, and no read or write to another peripheral has occurred since.
  o your code has called bcm2837_memory_barrier() since the last access to ANOTHER peripheral.

  \param[in] paddr Physical address to read from. See BCM2837_GPIO_BASE etc.
  \return the value read from the 32 bit register
  \sa Physical Addresses
*/
extern uint32_t bcm2837_peri_read_nb(volatile uint32_t* paddr);


/*! Writes 32 bit value from a peripheral address WITH a memory barrier before and after each write
  This is safe, but slow.  The MB before ensures that any in-flight write to another peripheral
  completes before this write is issued.  The MB after ensures that subsequent reads and writes
  to another peripheral will see the effect of this write.

  This is a tricky optimization; if you aren't sure, use the barrier version.

  \param[in] paddr Physical address to read from. See BCM2837_GPIO_BASE etc.
  \param[in] value The 32 bit value to write
  \sa Physical Addresses
*/
extern void bcm2837_peri_write(volatile uint32_t* paddr, uint32_t value);

/*! Writes 32 bit value from a peripheral address without the write barrier
  You should only use this when:
  o your code has previously called bcm2837_peri_write() for a register
  within the same peripheral, and no other peripheral access has occurred since.
  o your code has called bcm2837_memory_barrier() since the last access to ANOTHER peripheral.

  This is a tricky optimization; if you aren't sure, use the barrier version.

  \param[in] paddr Physical address to read from. See BCM2837_GPIO_BASE etc.
  \param[in] value The 32 bit value to write
  \sa Physical Addresses
*/
extern void bcm2837_peri_write_nb(volatile uint32_t* paddr, uint32_t value);

/*! Alters a number of bits in a 32 peripheral regsiter.
  It reads the current valu and then alters the bits defines as 1 in mask,
  according to the bit value in value.
  All other bits that are 0 in the mask are unaffected.
  Use this to alter a subset of the bits in a register.
  Memory barriers are used.  Note that this is not atomic; an interrupt
  routine can cause unexpected results.
  \param[in] paddr Physical address to read from. See BCM2837_GPIO_BASE etc.
  \param[in] value The 32 bit value to write, masked in by mask.
  \param[in] mask Bitmask that defines the bits that will be altered in the register.
  \sa Physical Addresses
*/
extern void bcm2837_peri_set_bits(volatile uint32_t* paddr, uint32_t value,
                                  uint32_t mask);
/*! @}    end of lowlevel */

/*! \defgroup gpio GPIO register access
  These functions allow you to control the GPIO interface. You can set the
  function of each GPIO pin, read the input state and set the output state.
  @{
*/

/*! Sets the Function Select register for the given pin, which configures
  the pin as Input, Output or one of the 6 alternate functions.
  \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
  \param[in] mode Mode to set the pin to, one of BCM2837_GPIO_FSEL_* from \ref bcm2837FunctionSelect
*/
extern void bcm2837_gpio_fsel(uint8_t pin, uint8_t mode);

/*! Sets the specified pin output to
  HIGH.
  \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
  \sa bcm2837_gpio_write()
*/
extern void bcm2837_gpio_set(uint8_t pin);

/*! Sets the specified pin output to
  LOW.
  \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
  \sa bcm2837_gpio_write()
*/
extern void bcm2837_gpio_clr(uint8_t pin);

/*! Sets any of the first 32 GPIO output pins specified in the mask to
  HIGH.
  \param[in] mask Mask of pins to affect. Use eg: (1 << RPI_GPIO_P1_03) | (1 << RPI_GPIO_P1_05)
  \sa bcm2837_gpio_write_multi()
*/
extern void bcm2837_gpio_set_multi(uint32_t mask);

/*! Sets any of the first 32 GPIO output pins specified in the mask to
  LOW.
  \param[in] mask Mask of pins to affect. Use eg: (1 << RPI_GPIO_P1_03) | (1 << RPI_GPIO_P1_05)
  \sa bcm2837_gpio_write_multi()
*/
extern void bcm2837_gpio_clr_multi(uint32_t mask);

/*! Reads the current level on the specified
  pin and returns either HIGH or LOW. Works whether or not the pin
  is an input or an output.
  \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
  \return the current level  either HIGH or LOW
*/
extern uint8_t bcm2837_gpio_lev(uint8_t pin);

/*! Event Detect Status.
  Tests whether the specified pin has detected a level or edge
  as requested by bcm2837_gpio_ren(), bcm2837_gpio_fen(), bcm2837_gpio_hen(),
  bcm2837_gpio_len(), bcm2837_gpio_aren(), bcm2837_gpio_afen().
  Clear the flag for a given pin by calling bcm2837_gpio_set_eds(pin);
  \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
  \return HIGH if the event detect status for the given pin is true.
*/
extern uint8_t bcm2837_gpio_eds(uint8_t pin);

/*! Same as bcm2837_gpio_eds() but checks if any of the pins specified in
  the mask have detected a level or edge.
  \param[in] mask Mask of pins to check. Use eg: (1 << RPI_GPIO_P1_03) | (1 << RPI_GPIO_P1_05)
  \return Mask of pins HIGH if the event detect status for the given pin is true.
*/
extern uint32_t bcm2837_gpio_eds_multi(uint32_t mask);

/*! Sets the Event Detect Status register for a given pin to 1,
  which has the effect of clearing the flag. Use this afer seeing
  an Event Detect Status on the pin.
  \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
*/
extern void bcm2837_gpio_set_eds(uint8_t pin);

/*! Same as bcm2837_gpio_set_eds() but clears the flag for any pin which
  is set in the mask.
  \param[in] mask Mask of pins to clear. Use eg: (1 << RPI_GPIO_P1_03) | (1 << RPI_GPIO_P1_05)
*/
extern void bcm2837_gpio_set_eds_multi(uint32_t mask);

/*! Enable Rising Edge Detect Enable for the specified pin.
  When a rising edge is detected, sets the appropriate pin in Event Detect Status.
  The GPRENn registers use
  synchronous edge detection. This means the input signal is sampled using the
  system clock and then it is looking for a ?011? pattern on the sampled signal. This
  has the effect of suppressing glitches.
  \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
*/
extern void bcm2837_gpio_ren(uint8_t pin);

/*! Disable Rising Edge Detect Enable for the specified pin.
  \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
*/
extern void bcm2837_gpio_clr_ren(uint8_t pin);

/*! Enable Falling Edge Detect Enable for the specified pin.
  When a falling edge is detected, sets the appropriate pin in Event Detect Status.
  The GPRENn registers use
  synchronous edge detection. This means the input signal is sampled using the
  system clock and then it is looking for a ?100? pattern on the sampled signal. This
  has the effect of suppressing glitches.
  \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
*/
extern void bcm2837_gpio_fen(uint8_t pin);

/*! Disable Falling Edge Detect Enable for the specified pin.
  \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
*/
extern void bcm2837_gpio_clr_fen(uint8_t pin);

/*! Enable High Detect Enable for the specified pin.
  When a HIGH level is detected on the pin, sets the appropriate pin in Event Detect Status.
  \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
*/
extern void bcm2837_gpio_hen(uint8_t pin);

/*! Disable High Detect Enable for the specified pin.
  \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
*/
extern void bcm2837_gpio_clr_hen(uint8_t pin);

/*! Enable Low Detect Enable for the specified pin.
  When a LOW level is detected on the pin, sets the appropriate pin in Event Detect Status.
  \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
*/
extern void bcm2837_gpio_len(uint8_t pin);

/*! Disable Low Detect Enable for the specified pin.
  \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
*/
extern void bcm2837_gpio_clr_len(uint8_t pin);

/*! Enable Asynchronous Rising Edge Detect Enable for the specified pin.
  When a rising edge is detected, sets the appropriate pin in Event Detect Status.
  Asynchronous means the incoming signal is not sampled by the system clock. As such
  rising edges of very short duration can be detected.
  \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
*/
extern void bcm2837_gpio_aren(uint8_t pin);

/*! Disable Asynchronous Rising Edge Detect Enable for the specified pin.
  \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
*/
extern void bcm2837_gpio_clr_aren(uint8_t pin);

/*! Enable Asynchronous Falling Edge Detect Enable for the specified pin.
  When a falling edge is detected, sets the appropriate pin in Event Detect Status.
  Asynchronous means the incoming signal is not sampled by the system clock. As such
  falling edges of very short duration can be detected.
  \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
*/
extern void bcm2837_gpio_afen(uint8_t pin);

/*! Disable Asynchronous Falling Edge Detect Enable for the specified pin.
  \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
*/
extern void bcm2837_gpio_clr_afen(uint8_t pin);

/*! Sets the Pull-up/down register for the given pin. This is
  used with bcm2837_gpio_pudclk() to set the  Pull-up/down resistor for the given pin.
  However, it is usually more convenient to use bcm2837_gpio_set_pud().
  \param[in] pud The desired Pull-up/down mode. One of BCM2837_GPIO_PUD_* from bcm2837PUDControl
  \sa bcm2837_gpio_set_pud()
*/
extern void bcm2837_gpio_pud(uint8_t pud);

/*! Clocks the Pull-up/down value set earlier by bcm2837_gpio_pud() into the pin.
  \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
  \param[in] on HIGH to clock the value from bcm2837_gpio_pud() into the pin.
  LOW to remove the clock.
  \sa bcm2837_gpio_set_pud()
*/
extern void bcm2837_gpio_pudclk(uint8_t pin, uint8_t on);

/*! Reads and returns the Pad Control for the given GPIO group.
  Caution: requires root access.
  \param[in] group The GPIO pad group number, one of BCM2837_PAD_GROUP_GPIO_*
  \return Mask of bits from BCM2837_PAD_* from \ref bcm2837PadGroup
*/
extern uint32_t bcm2837_gpio_pad(uint8_t group);

/*! Sets the Pad Control for the given GPIO group.
  Caution: requires root access.
  \param[in] group The GPIO pad group number, one of BCM2837_PAD_GROUP_GPIO_*
  \param[in] control Mask of bits from BCM2837_PAD_* from \ref bcm2837PadGroup. Note
  that it is not necessary to include BCM2837_PAD_PASSWRD in the mask as this
  is automatically included.
*/
extern void bcm2837_gpio_set_pad(uint8_t group, uint32_t control);

/*! Sets the output state of the specified pin
  \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
  \param[in] on HIGH sets the output to HIGH and LOW to LOW.
*/
extern void bcm2837_gpio_write(uint8_t pin, uint8_t on);

/*! Sets any of the first 32 GPIO output pins specified in the mask to the state given by on
  \param[in] mask Mask of pins to affect. Use eg: (1 << RPI_GPIO_P1_03) | (1 << RPI_GPIO_P1_05)
  \param[in] on HIGH sets the output to HIGH and LOW to LOW.
*/
extern void bcm2837_gpio_write_multi(uint32_t mask, uint8_t on);

/*! Sets the first 32 GPIO output pins specified in the mask to the value given by value
  \param[in] value values required for each bit masked in by mask, eg: (1 << RPI_GPIO_P1_03) | (1 << RPI_GPIO_P1_05)
  \param[in] mask Mask of pins to affect. Use eg: (1 << RPI_GPIO_P1_03) | (1 << RPI_GPIO_P1_05)
*/
extern void bcm2837_gpio_write_mask(uint32_t value, uint32_t mask);

/*! Sets the Pull-up/down mode for the specified pin. This is more convenient than
  clocking the mode in with bcm2837_gpio_pud() and bcm2837_gpio_pudclk().
  \param[in] pin GPIO number, or one of RPI_GPIO_P1_* from \ref RPiGPIOPin.
  \param[in] pud The desired Pull-up/down mode. One of BCM2837_GPIO_PUD_* from bcm2837PUDControl
*/
extern void bcm2837_gpio_set_pud(uint8_t pin, uint8_t pud);
