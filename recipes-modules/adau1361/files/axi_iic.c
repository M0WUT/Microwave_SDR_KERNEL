#include "axi_iic.h"

static int iic_tx_fifo_space(struct iic_local *dev);
static int iic_remaining_tx_len(struct iic_local *dev);
static void iic_fill_tx_fifo(struct iic_local *dev);
static void iic_begin_write(struct iic_local *dev);

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
    reg_write(dev, OFFSET_GIE, (1 << 31));  // Globally enable Interrupts
    reg_write(dev, OFFSET_IER, 0x0E);  // Enable FIFO Full/Empty and Error interrupts
    reg_write(dev, OFFSET_GPO, 0);  // Set any GPIO to 0
    return;    
}

// I2C interrupt handler, just checks if there are any interrupts to service, then calls iic_irq_process
irqreturn_t iic_irq(int irq, void *dev_p)
{
    uint32_t isr, ier;
    struct iic_local *dev = dev_p;
    #ifdef DEBUG
        printk(KERN_INFO "Interrupt!");
    #endif

    // Check which interrupt has called this
    isr = reg_read(dev, OFFSET_ISR);
    // Check which interrupts we have enabled
    ier = reg_read(dev, OFFSET_IER);

    // If there is an interrupt present that we have enabled, process it
    return ((isr & ier) ? IRQ_WAKE_THREAD : IRQ_NONE);
}

// Function that actually processes the interrupt - note there may be more than one thread at once
irqreturn_t iic_irq_process(int irq, void *dev_p){
    uint32_t isr, ier, intr, clr_mask;
    struct iic_local *dev = dev_p;

    // Lock write access to the I2C TX controls
    mutex_lock(&dev->tx_lock);

    // Check which interrupt has called this
    isr = reg_read(dev, OFFSET_ISR);
    // Check which interrupts we have enabled
    ier = reg_read(dev, OFFSET_IER);

    // The interrupts that we are bothering to handle
    intr = isr & ier;

    #ifdef DEBUG
        printk(KERN_INFO "Handling Interrupt! ISR: 0x%x\tIER: 0x%x\tMasked ISR: 0x%x",
            isr,
            ier,
            intr
        );
    #endif

    // Contains bits of the ISR that we will clear at the end of the function
    clr_mask = 0;

    // Check for each interrupt and handle it

    if(intr & INTERRUPT_TX_ERROR_MASK){
        clr_mask |= INTERRUPT_TX_ERROR_MASK;
    }

    if(intr & INTERRUPT_TX_FIFO_EMPTY_MASK){
        // TX FIFO is empty...
        clr_mask |= INTERRUPT_TX_FIFO_EMPTY_MASK;

        // so fill it!
        iic_fill_tx_fifo(dev);

        // Check for end of message
        if(iic_remaining_tx_len(dev) == 0 ){
            // Do we have another message and space to write to the FIFO? 
            // (at least 2 bytes becuase FIFO occupancy looks a bit weird)
            if(dev->tx_msg_num > 1 && iic_tx_fifo_space(dev) > 2){
                // If so, move onto the next message
                dev->tx_msg_p++;  // Increment message point
                dev->tx_msg_num--;  // Decrease number of remaining messages
                iic_begin_write(dev);  // Start sending next message
            } else if(dev->tx_msg_num == 1){
                // This is the last message, all done!
                #ifdef DEBUG
                    printk(KERN_INFO "IIC: Finished sending messages!");
                #endif
                ;
            }
        }
    }

    if(intr & INTERRUPT_RX_FIFO_FULL_MASK){
        clr_mask |= INTERRUPT_RX_FIFO_FULL_MASK;
    }

    // Clear handled interrupts
    isr = reg_read(dev, OFFSET_ISR);
    reg_write(dev, OFFSET_ISR, isr | clr_mask);

    // Unlock TX registers
    mutex_unlock(&dev->tx_lock);

    return IRQ_HANDLED;
}

// Returns number of bytes that the FIFO can be loaded with without overflow
static int iic_tx_fifo_space(struct iic_local* dev){
    // Full FIFO is 16 bytes, TX_FIFO_OCY contain number of bytes - 1
    // Do not use to check FIFO empty
    return 15 - reg_read(dev, OFFSET_TX_FIFO_OCY);
}

// Returns how many bytes of tx_msg still need to be sent
static int iic_remaining_tx_len(struct iic_local* dev){
    return dev->tx_msg_p->len - dev->tx_msg_pos;
}

// Put as many bytes of the current message as possible
static void iic_fill_tx_fifo(struct iic_local *dev){
    int cr;
    if(iic_remaining_tx_len(dev) > 1){
        // We have more than one byte left to send
        // Put as much data as we can (minus the last byte) into the FIFO
         int fifo_space = iic_tx_fifo_space(dev);
        // Can't write last byte of message to FIFO, so report length as 1 short
        int msg_len = iic_remaining_tx_len(dev) - 1;
        uint8_t data;

        // Limit number of bytes to what the FIFO can handle
        if (msg_len > fifo_space)
            msg_len = fifo_space;

        #ifdef DEBUG
            printk(KERN_INFO "Writing %d bytes to the TX FIFO\n", msg_len);
        #endif

        // Write those bytes to the FIFO
        while(msg_len--){
            data = dev->tx_msg_p->buf[dev->tx_msg_pos++];
            reg_write(dev, OFFSET_TX_FIFO, data);
        }
        return;
    }

    if(iic_remaining_tx_len(dev) == 1){
        // Have the last byte to send
        if(dev->tx_msg_num == 1){
            // If this is the last message, add a STOP condition
            // by clearing MSMS bit
            cr = reg_read(dev, OFFSET_CR);
            reg_write(dev, OFFSET_CR, cr & ~CR_MSMS_MASK);
        }

        // Send last data byte
        reg_write(dev, OFFSET_TX_FIFO, dev->tx_msg_p->buf[dev->tx_msg_pos++]);
    }
}

// Begins I2C write, subsequent writes are handled by interrupts
static void iic_begin_write(struct iic_local *dev){
    int cr;
    mutex_lock(&dev->tx_lock);
    


    // Write slave address to the TX FIFO with 0 in LSB for write
    reg_write(dev, OFFSET_TX_FIFO, (dev->tx_msg_p->addr << 1));

    // and here we go!
    cr = reg_read(dev, OFFSET_CR);
    if (cr & CR_MSMS_MASK){
        // Device is already in Master mode, repeated start
        reg_write(dev, OFFSET_CR, cr | CR_RSTA_MASK | CR_TX_MASK);
    } else {
        // Put device into master mode
        reg_write(dev, OFFSET_CR, cr | CR_MSMS_MASK | CR_TX_MASK);
    }

    mutex_unlock(&dev->tx_lock);
    return;
}


// Takes an "i2c_msg" array with "msg_num" items and sends them
static void iic_write_msg(struct iic_local *dev, struct i2c_msg *msg, int msg_num){
    dev->tx_msg_p = msg;
    dev->tx_msg_num = msg_num;
    dev->tx_msg_pos = 0;
    iic_begin_write(dev);
}


// Writes to a register address on the I2C slave device
uint32_t iic_write(struct iic_local *dev, uint16_t reg_address, uint8_t data) {
    // DEBUG
    struct i2c_msg msg;
    uint8_t x[3] = {0x40, 0x00, 0x01};
    msg.addr = 0x38;
    msg.buf = &x[0];
    msg.len = 3;
    iic_write_msg(dev, &msg, 1);
    msleep(1000);

    return 0;
}

// Reads a register on the I2C slave device
uint32_t iic_read(struct iic_local *dev, uint16_t reg_address) {
    //iic_access_register(dev, reg_address);
    return 0;
}

