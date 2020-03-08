#include "axi_iic.h"

#define TIMEOUT 100

volatile int interrupt_value = 0;

// Write to a control register for IIC peripheral
static void reg_write(struct iic_local* dev, uint32_t offset, uint32_t data){
    iowrite32(data, dev->base_addr + offset);
    #ifdef DEBUG 
		printk(KERN_INFO "ADAU1361: Writing data %u to address %#06x\n", data, offset);
    #endif
}

// Reads from a control register for IIC peripheral
static uint32_t reg_read(struct iic_local* dev, uint32_t offset){
    uint32_t x = ioread32(dev->base_addr + offset);
    #ifdef DEBUG
		printk(KERN_INFO "ADAU1361: Read data %u from address %#06x\n", x, offset);
	#endif
    return x;
}

// Initialises I2C peripheral
void iic_init(struct iic_local* dev){
    reg_write(dev, OFFSET_SOFTR, 0x0A);  // Soft reset the peripheral
    reg_write(dev, OFFSET_RX_FIFO_PIRQ, 15);  // Set RX FIFO to maximum size before interrupt
    reg_write(dev, OFFSET_CR, 0x02);  // Reset TX FIFO
    reg_write(dev, OFFSET_CR, 0x01);  // Enable I2C peripheral and un-reset TX FIFO
    reg_write(dev, OFFSET_IER, 0x0E);  // Enable FIFO Full/Empty and Error interrupts
    reg_write(dev, OFFSET_GIE, (1 << 31));  // Globally enable Interrupts
    reg_write(dev, OFFSET_GPO, 0);  // Set any GPIO to 0
    return;    
}

// I2C interrupt handler
irqreturn_t iic_irq(int irq, void *dev_p)
{
    int i = 0;
    struct iic_local *dev = dev_p;
    #ifdef DEBUG
        printk(KERN_INFO "Interrupt!");
    #endif
    
    printk(KERN_INFO "Handling interrupt");
    i = reg_read(dev, OFFSET_ISR) & 0x0F;  // Mask off the interrupts that aren't enabled
    reg_write(dev, OFFSET_ISR, i);  // Clear the interrupt
    printk(KERN_INFO "Handled interrupt %d", i);
    interrupt_value = i;
    return IRQ_HANDLED;
}

// Writes to a register address on the I2C slave device
uint32_t iic_write(struct iic_local* dev, uint16_t reg_address, uint8_t data) {
    uint32_t i = 0;
    reg_write(dev, OFFSET_TX_FIFO, 0x100 | (uint16_t)(dev->slave_address) << 1 | 0x01);  // Send Start bit and Slave address with read bit set

    reg_write(dev, OFFSET_TX_FIFO, (reg_address >> 8) & 0xFF);  // Send MSB of register address
    reg_write(dev, OFFSET_TX_FIFO, reg_address & 0xFF);  // Send LSB of register address
    for(i = 0; i < TIMEOUT; i++) {
        #ifdef DEBUG
            printk(KERN_INFO "Waiting for the interrupt");
        #endif

        if(interrupt_value != 0){
            break;
        } else if (i == TIMEOUT - 1) {
            printk(KERN_ERR "Timed out waiting for I2C interrupt");
            return -1;
        }
    }
    if (interrupt_value == INTERRUPT_TX_FIFO_EMPTY){
        printk(KERN_INFO "ADAU1361: TX FIFO empty");
    } else if (interrupt_value == INTERRUPT_TX_ERROR){
        printk(KERN_INFO "ADAU1361: No ACK received");
    } else {
        printk(KERN_INFO "ADAU1361: Unexpected interrupt: %d", interrupt_value);
    }

    reg_write(dev, OFFSET_TX_FIFO, 0x200 | data);
    

   
    return 0;
}

// Reads a register on the I2C slave device
uint32_t iic_read(struct iic_local* dev, uint16_t reg_address) {
    //iic_access_register(dev, reg_address);
    return 0;
}

