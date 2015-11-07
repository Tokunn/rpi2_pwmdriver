#ifndef _RPI_GPIO_H
#define _RPI_GPIO_H 1

#include <stdint-gcc.h>

/* レジスタアドレス */
#define RPI_REG_BASE	 0x3F000000
#define RPI_GPIO_BASE	 (RPI_REG_BASE + 0x200000)
#define RPI_BLOCK_SIZE	4096

/* GPIOレジスタオフセット */
#define RPI_GPFSEL0_OFFSET	0x00
#define RPI_GPFSEL1_OFFSET	0x04
#define RPI_GPFSEL2_OFFSET	0x08
#define RPI_GPFSEL3_OFFSET	0x0c

#define RPI_GPSET0_OFFSET	0x1c
#define RPI_GPCLR0_OFFSET	0x28
#define RPI_GPLEV0_OFFSET	0x34
#define RPI_GPPUD_OFFSET	0x94
#define RPI_GPPUDCLK0_OFFSET	0x98

/* GPIOレジスタインデックス */
#define RPI_GPFSEL0_INDEX	0
#define RPI_GPFSEL1_INDEX	1
#define RPI_GPFSEL2_INDEX	2
#define RPI_GPFSEL3_INDEX	3

#define RPI_GPSET0_INDEX	7
#define RPI_GPCLR0_INDEX	10
#define RPI_GPLEV0_INDEX	13
#define RPI_GPPUD_INDEX	37
#define RPI_GPPUDCLK0_INDEX	38

/* GPIO Functions	*/
#define	RPI_GPF_INPUT	0x00
#define	RPI_GPF_OUTPUT	0x01
#define	RPI_GPF_ALT0	0x04
#define	RPI_GPF_ALT1	0x05
#define	RPI_GPF_ALT2	0x06
#define	RPI_GPF_ALT3	0x07
#define	RPI_GPF_ALT4	0x03
#define	RPI_GPF_ALT5	0x02

/* PULLUP/PULLDOWN */
#define RPI_GPIO_PULLNONE	0x00
#define RPI_GPIO_PULLDOWN	0x01
#define	RPI_GPIO_PULLUP		0x02

/* GPIO Mask */
#define RPI_GPIO_P1MASK	(uint32_t) ((0x01<<2) | (0x01<<3) | (0x01<<4) | \
									(0x01<<7) | (0x01<<8) | (0x01<<9) | \
									(0x01<<10)| (0x01<<11)| (0x01<<14)| \
									(0x01<<15)| (0x01<<17)| (0x01<<18)| \
									(0x01<<22)| (0x01<<23)| (0x01<<24)| \
									(0x01<<25)| (0x01<<27)\
								   )
#define RPI_GPIO_P5MASK	(uint32_t) ((0x01<<28)| (0x01<<29)| 0x01<<30) | (0x01<<31))

#endif

