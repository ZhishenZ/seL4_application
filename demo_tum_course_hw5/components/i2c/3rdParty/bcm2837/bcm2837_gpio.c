/*
 * Copyright (C) 2011-2013 Mike McCauley
 * Copyright (C) 2020-2021, HENSOLDT Cyber GmbH
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include "bcm2837_gpio.h"

#include <stddef.h>

static volatile uint32_t* bcm2837_gpio = NULL;
static volatile uint32_t* bcm2837_pads = NULL;

/*---------------HELPER FUNCTIONS-------------------*/

/* Read with memory barriers from peripheral
 *
 */
uint32_t bcm2837_peri_read(volatile uint32_t* paddr)
{
    uint32_t ret;

    __sync_synchronize();
    ret = *paddr;
    __sync_synchronize();
    return ret;
}

/* read from peripheral without the read barrier
 * This can only be used if more reads to THE SAME peripheral
 * will follow.  The sequence must terminate with memory barrier
 * before any read or write to another peripheral can occur.
 * The MB can be explicit, or one of the barrier read/write calls.
 */
uint32_t bcm2837_peri_read_nb(volatile uint32_t* paddr)
{
    return *paddr;
}

/* Write with memory barriers to peripheral
 */

void bcm2837_peri_write(volatile uint32_t* paddr, uint32_t value)
{
    __sync_synchronize();
    *paddr = value;
    __sync_synchronize();
}

/* write to peripheral without the write barrier */
void bcm2837_peri_write_nb(volatile uint32_t* paddr, uint32_t value)
{
    *paddr = value;
}

/* Set/clear only the bits in value covered by the mask
 * This is not atomic - can be interrupted.
 */
void bcm2837_peri_set_bits(volatile uint32_t* paddr, uint32_t value,
                           uint32_t mask)
{
    uint32_t v = bcm2837_peri_read(paddr);
    v = (v & ~mask) | (value & mask);
    bcm2837_peri_write(paddr, v);
}

/*-----------------GPIO INTERFACE----------------------------*/

/* initialize the gpio base address */
void bcm2837_gpio_init(void* vaddr)
{
    bcm2837_gpio = (uint32_t*)vaddr;
    // ToDo: bcm2837_pads = ...
}

/* Function select
// pin is a BCM2837 GPIO pin number NOT RPi pin number
//      There are 6 control registers, each control the functions of a block
//      of 10 pins.
//      Each control register has 10 sets of 3 bits per GPIO pin:
//
//      000 = GPIO Pin X is an input
//      001 = GPIO Pin X is an output
//      100 = GPIO Pin X takes alternate function 0
//      101 = GPIO Pin X takes alternate function 1
//      110 = GPIO Pin X takes alternate function 2
//      111 = GPIO Pin X takes alternate function 3
//      011 = GPIO Pin X takes alternate function 4
//      010 = GPIO Pin X takes alternate function 5
//
// So the 3 bits for port X are:
//      X / 10 + ((X % 10) * 3)
*/

void bcm2837_gpio_fsel(uint8_t pin, uint8_t mode)
{
    /* Function selects are 10 pins per 32 bit word, 3 bits per pin */
    volatile uint32_t* paddr = bcm2837_gpio + BCM2837_GPFSEL0 / 4 + (pin / 10);
    uint8_t   shift = (pin % 10) * 3;
    uint32_t  mask = BCM2837_GPIO_FSEL_MASK << shift;
    uint32_t  value = mode << shift;

    bcm2837_peri_set_bits(paddr, value, mask);
}

/* Set output pin */
void bcm2837_gpio_set(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2837_gpio + BCM2837_GPSET0 / 4 + pin / 32;
    uint8_t shift = pin % 32;
    bcm2837_peri_write(paddr, 1 << shift);
}

/* Clear output pin */
void bcm2837_gpio_clr(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2837_gpio + BCM2837_GPCLR0 / 4 + pin / 32;
    uint8_t shift = pin % 32;
    bcm2837_peri_write(paddr, 1 << shift);
}

/* Set all output pins in the mask */
void bcm2837_gpio_set_multi(uint32_t mask)
{
    volatile uint32_t* paddr = bcm2837_gpio + BCM2837_GPSET0 / 4;
    bcm2837_peri_write(paddr, mask);
}

/* Clear all output pins in the mask */
void bcm2837_gpio_clr_multi(uint32_t mask)
{
    volatile uint32_t* paddr = bcm2837_gpio + BCM2837_GPCLR0 / 4;
    bcm2837_peri_write(paddr, mask);
}

/* Read input pin */
uint8_t bcm2837_gpio_lev(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2837_gpio + BCM2837_GPLEV0 / 4 + pin / 32;
    uint8_t shift = pin % 32;
    uint32_t value = bcm2837_peri_read(paddr);
    return (value & (1 << shift)) ? 1 : 0;
}

/* See if an event detection bit is set
// Sigh cant support interrupts yet
*/
uint8_t bcm2837_gpio_eds(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2837_gpio + BCM2837_GPEDS0 / 4 + pin / 32;
    uint8_t shift = pin % 32;
    uint32_t value = bcm2837_peri_read(paddr);
    return (value & (1 << shift)) ? 1 : 0;
}

uint32_t bcm2837_gpio_eds_multi(uint32_t mask)
{
    volatile uint32_t* paddr = bcm2837_gpio + BCM2837_GPEDS0 / 4;
    uint32_t value = bcm2837_peri_read(paddr);
    return (value & mask);
}

/* Write a 1 to clear the bit in EDS */
void bcm2837_gpio_set_eds(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2837_gpio + BCM2837_GPEDS0 / 4 + pin / 32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    bcm2837_peri_write(paddr, value);
}

void bcm2837_gpio_set_eds_multi(uint32_t mask)
{
    volatile uint32_t* paddr = bcm2837_gpio + BCM2837_GPEDS0 / 4;
    bcm2837_peri_write(paddr, mask);
}

/* Rising edge detect enable */
void bcm2837_gpio_ren(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2837_gpio + BCM2837_GPREN0 / 4 + pin / 32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    bcm2837_peri_set_bits(paddr, value, value);
}
void bcm2837_gpio_clr_ren(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2837_gpio + BCM2837_GPREN0 / 4 + pin / 32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    bcm2837_peri_set_bits(paddr, 0, value);
}

/* Falling edge detect enable */
void bcm2837_gpio_fen(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2837_gpio + BCM2837_GPFEN0 / 4 + pin / 32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    bcm2837_peri_set_bits(paddr, value, value);
}
void bcm2837_gpio_clr_fen(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2837_gpio + BCM2837_GPFEN0 / 4 + pin / 32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    bcm2837_peri_set_bits(paddr, 0, value);
}

/* High detect enable */
void bcm2837_gpio_hen(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2837_gpio + BCM2837_GPHEN0 / 4 + pin / 32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    bcm2837_peri_set_bits(paddr, value, value);
}
void bcm2837_gpio_clr_hen(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2837_gpio + BCM2837_GPHEN0 / 4 + pin / 32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    bcm2837_peri_set_bits(paddr, 0, value);
}

/* Low detect enable */
void bcm2837_gpio_len(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2837_gpio + BCM2837_GPLEN0 / 4 + pin / 32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    bcm2837_peri_set_bits(paddr, value, value);
}
void bcm2837_gpio_clr_len(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2837_gpio + BCM2837_GPLEN0 / 4 + pin / 32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    bcm2837_peri_set_bits(paddr, 0, value);
}

/* Async rising edge detect enable */
void bcm2837_gpio_aren(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2837_gpio + BCM2837_GPAREN0 / 4 + pin / 32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    bcm2837_peri_set_bits(paddr, value, value);
}
void bcm2837_gpio_clr_aren(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2837_gpio + BCM2837_GPAREN0 / 4 + pin / 32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    bcm2837_peri_set_bits(paddr, 0, value);
}

/* Async falling edge detect enable */
void bcm2837_gpio_afen(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2837_gpio + BCM2837_GPAFEN0 / 4 + pin / 32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    bcm2837_peri_set_bits(paddr, value, value);
}
void bcm2837_gpio_clr_afen(uint8_t pin)
{
    volatile uint32_t* paddr = bcm2837_gpio + BCM2837_GPAFEN0 / 4 + pin / 32;
    uint8_t shift = pin % 32;
    uint32_t value = 1 << shift;
    bcm2837_peri_set_bits(paddr, 0, value);
}

/* Set pullup/down */
void bcm2837_gpio_pud(uint8_t pud)
{
    volatile uint32_t* paddr = bcm2837_gpio + BCM2837_GPPUD / 4;
    bcm2837_peri_write(paddr, pud);
}

/* Pullup/down clock
// Clocks the value of pud into the GPIO pin
*/
void bcm2837_gpio_pudclk(uint8_t pin, uint8_t on)
{
    volatile uint32_t* paddr = bcm2837_gpio + BCM2837_GPPUDCLK0 / 4 + pin / 32;
    uint8_t shift = pin % 32;
    bcm2837_peri_write(paddr, (on ? 1 : 0) << shift);
}

/* Read GPIO pad behaviour for groups of GPIOs */
uint32_t bcm2837_gpio_pad(uint8_t group)
{
    if (NULL == bcm2837_pads)
    {
        return 0;
    }

    volatile uint32_t* paddr = bcm2837_pads + BCM2837_PADS_GPIO_0_27 / 4 + group;
    return bcm2837_peri_read(paddr);
}

/* Set GPIO pad behaviour for groups of GPIOs
// powerup value for all pads is
// BCM2837_PAD_SLEW_RATE_UNLIMITED | BCM2837_PAD_HYSTERESIS_ENABLED | BCM2837_PAD_DRIVE_8mA
*/
void bcm2837_gpio_set_pad(uint8_t group, uint32_t control)
{
    if (NULL == bcm2837_pads)
    {
        return;
    }

    volatile uint32_t* paddr = bcm2837_pads + BCM2837_PADS_GPIO_0_27 / 4 + group;
    bcm2837_peri_write(paddr, control | BCM2837_PAD_PASSWRD);
}
