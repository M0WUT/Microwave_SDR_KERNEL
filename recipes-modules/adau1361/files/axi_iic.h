#ifndef IIC_H
#define IIC_H

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>

#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

////////////////
// DEBUG MODE //
////////////////
#define DEBUG

/////////////////////////////////////////
// Registers offsets from base address //
/////////////////////////////////////////

#define OFFSET_GIE 0x01C
#define OFFSET_ISR 0x020
#define OFFSET_IER 0x028
#define OFFSET_SOFTR 0x040
#define OFFSET_CR 0x100
#define OFFSET_SR 0x104
#define OFFSET_TX_FIFO 0x108
#define OFFSET_RX_FIFO 0x10C
#define OFFSET_ADR 0x110
#define OFFSET_TX_FIFO_OCY 0x114
#define OFFSET_RX_FIFO_OCY 0x118
#define OFFSET_TEN_ADR 0x11C
#define OFFSET_RX_FIFO_PIRQ 0x120
#define OFFSET_GPO 0x124
#define OFFSET_TSUSTA 0x128
#define OFFSET_TSUSTO 0x12C
#define OFFSET_THDSTA 0x130
#define OFFSET_TSUDAT 0x134
#define OFFSET_TBUF 0x138
#define OFFSET_THIGH 0x13C
#define OFFSET_TLOW 0x140
#define OFFSET_THDDAT 0x144

/////////////////////
// Interrupt Masks //
/////////////////////
#define INTERRUPT_TX_FIFO_HALF_EMPTY 0x80
#define INTERRUPT_NOT_ADDRESSED_AS_SLAVE 0x40
#define INTERRUPT_ADDRESSED_AS_SLAVE 0x20
#define INTERRUPT_IIC_BUS_NOT_EMPTY 0x10
#define INTERRUPT_RX_FIFO_FULL 0x08
#define INTERRUPT_TX_FIFO_EMPTY 0x04
#define INTERRUPT_TX_ERROR 0x02
#define INTERRUPT_ARBITRATION_LOST 0x01




/////////////////////////
// Function prototypes //
/////////////////////////

// Struct for I2C master
struct iic_local {
	int irq;
	unsigned long mem_start;
	unsigned long mem_end;
	void __iomem *base_addr;
    uint32_t slave_address;  // Slave address of I2C device
};

// Initialisation function for I2C peripheral
void iic_init(struct iic_local* dev);

irqreturn_t iic_irq(int irq, void *lp);

// Basic read/write to I2C device registers
uint32_t iic_write(struct iic_local* dev, uint16_t reg_address, uint8_t data);
uint32_t iic_read(struct iic_local* dev, uint16_t reg_address);






#endif  // define IIC_H