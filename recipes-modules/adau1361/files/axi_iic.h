#ifndef IIC_H
#define IIC_H

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

////////////////
// DEBUG MODE //
////////////////
//#define DEBUG

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
#define INTERRUPT_TX_FIFO_HALF_EMPTY_MASK 0x80
#define INTERRUPT_NOT_ADDRESSED_AS_SLAVE_MASK 0x40
#define INTERRUPT_ADDRESSED_AS_SLAVE_MASK 0x20
#define INTERRUPT_BUS_NOT_BUSY_MASK 0x10
#define INTERRUPT_RX_FIFO_FULL_MASK 0x08
#define INTERRUPT_TX_FIFO_EMPTY_MASK 0x04
#define INTERRUPT_TX_ERROR_MASK 0x02
#define INTERRUPT_ARBITRATION_LOST_MASK 0x01

///////////////////////
// Masks for CR bits //
///////////////////////
#define CR_GC_EN_MASK 0x40
#define CR_RSTA_MASK 0x20
#define CR_TXAK_MASK 0x10
#define CR_TX_MASK 0x08
#define CR_MSMS_MASK 0x04
#define CR_TX_FIFO_RST_MASK 0x02
#define CR_EN_MASK 0x01



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
	struct i2c_msg *tx_msg_p;  // Pointer to message currently being sent
	int tx_msg_num;  // How many I2C messages are to be sent
	int tx_msg_pos;  // Next byte of current message to be written to the TX FIFO
	struct mutex tx_lock;  // Lock for accessing the TX FIFO
	uint32_t tx_messages;  // Number of messages that still require sending
	uint8_t rx_data;  // Byte received from the I2C bus

};

// Initialisation function for I2C peripheral
void iic_init(struct iic_local *dev);

irqreturn_t iic_irq(int irq, void *dev_p);
irqreturn_t iic_irq_process(int irq, void *dev_p);

// Basic read/write to I2C device registers
void iic_write_single(struct iic_local *dev, uint16_t reg_address, uint8_t data);
void iic_write_block(struct iic_local *dev, uint16_t reg_address, uint8_t *data, uint8_t data_length);
uint8_t iic_read_single(struct iic_local *dev, uint16_t reg_address);


#endif  // define IIC_H
